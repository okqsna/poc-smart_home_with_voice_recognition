/******************************************************************************
* File Name:   main.cpp
*******************************************************************************/

extern "C"{
	#include "cyhal.h"
	#include "cybsp.h"
	#include "cy_retarget_io.h"
	#include "stdlib.h"
}

#include "voice-recognition-cpp-mcu-v3/edge-impulse-sdk/dsp/numpy.hpp"
#include "voice-recognition-cpp-mcu-v3/edge-impulse-sdk/classifier/ei_run_classifier.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define FRAME_SIZE              (1024)
#define TOTAL_SAMPLES               EI_CLASSIFIER_RAW_SAMPLE_COUNT 
#define NUM_DMA_TRANSFERS           16u 


/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ              16000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u
/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

/*******************************************************************************
* Function Prototypes
********************************************************************************/
extern "C" { 
    void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
    void clock_init(void);
}
/*******************************************************************************
* Function Prototypes from Edge Impulse
********************************************************************************/
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Interrupt flags */
volatile bool pdm_pcm_flag = false;
/* Audio buffer */
int16_t audio_frame[TOTAL_SAMPLES] = {0}; 
volatile uint32_t current_audio_offset = 0; 
volatile uint32_t dma_transfer_count = 0;


/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = 
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_STEREO, 
    .word_length     = 16,  /* bits */
    .left_gain       = 0,   /* dB */
    .right_gain      = 0,   /* dB */
};

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    // converting input data from audio buffer
	return ei::numpy::int16_to_float(audio_frame + offset, out_ptr, length);
};


/*******************************************************************************
* Function Name: main
********************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Init the clocks */
    clock_init();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the PDM/PCM block */
    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);
    
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("****************** \
    PDM/PCM Edge Impulse Integration \
    ****************** \r\n\n");
    
    // first audio recording
    current_audio_offset = 0;
    dma_transfer_count = 0;
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
    printf("DEBUG: Starting reading audio from microphone\r\n");

    cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);

    for(;;)
    {
        /* Check if the full 16000 samples are ready */
        if (pdm_pcm_flag)
        {
            // audio recorded
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            printf("DEBUG: Ending reading audio\r\n"); 
            
            /* Clear the PDM/PCM flag */
            pdm_pcm_flag = false;

            signal_t signal;
            ei_impulse_result_t ei_result; 
	        signal.total_length = TOTAL_SAMPLES;
	        signal.get_data = &raw_feature_get_data;
	
            printf("DEBUG: Starting classifier\r\n"); 
	      
            EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &ei_result, false); 

            printf("DEBUG: Classifier returned successfully. Results printing...\r\n"); 
	
			printf("****************** \
			RESULTS of classifier \
			****************** \r\n\n");
	        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
	            printf("%s: %.3f\r\n", 
	                ei_result.classification[i].label,
	                ei_result.classification[i].value);
	        }

            // new audio recording
            current_audio_offset = 0;
            dma_transfer_count = 0;
            
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
            cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
        }

        cyhal_syspm_sleep();

    }
}


/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
* PDM/PCM ISR handler.
*******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;

    dma_transfer_count++;

    if (dma_transfer_count >= NUM_DMA_TRANSFERS) 
    {
        pdm_pcm_flag = true;
    }
    else
    {
        current_audio_offset = dma_transfer_count * FRAME_SIZE;
        cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame + current_audio_offset, FRAME_SIZE);
    }
}

/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
* Initialize the clocks in the system.
*******************************************************************************/
void clock_init(void)
{
    /* Initialize the PLL */
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1]) 
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}
/* [] END OF FILE */