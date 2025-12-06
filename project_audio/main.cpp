/******************************************************************************
* File Name:   main.cpp
*******************************************************************************/

#include <cstdint>
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
#define FRAME_SIZE              (1000)
#define TOTAL_SAMPLES               EI_CLASSIFIER_RAW_SAMPLE_COUNT 
#define NUM_DMA_TRANSFERS           16u 


/* Desired sample rate */
#define SAMPLE_RATE_HZ              16000u
/* Decimation Rate of the PDM/PCM block */
#define DECIMATION_RATE             64u
/* Audio Subsystem Clock */
#define AUDIO_SYS_CLOCK_HZ          24576000u
/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4
#define RGB_LED_GREEN 				P1_1
#define CORRECT_CLASSIFICATION 		0.5

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
bool pdm_pcm_flag = false;
bool led_lighted = false;
/* Audio buffer */
int16_t audio_frame[TOTAL_SAMPLES] = {0};
uint32_t current_audio_offset = 0; 
uint32_t dma_transfer_count = 0;
/* E-ink display */
uint8_t previous_frame[PV_EINK_IMAGE_SIZE] = {0};
uint8_t current_frame[PV_EINK_IMAGE_SIZE] = {0};


/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;
cyhal_spi_t spi;

/* HAL Config PDM PCM */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = 
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_LEFT, 
    .word_length     = 16,  /* bits */
    .left_gain       = 0,   /* dB */
    .right_gain      = 0,   /* dB */
};


/* E-ink shield display */
const mtb_e2271cs021_pins_t pins =
{
    .spi_mosi  = CYBSP_D11,
    .spi_miso  = CYBSP_D12,
    .spi_sclk  = CYBSP_D13,
    .spi_cs    = CYBSP_D10,
    .reset     = CYBSP_D2,
    .busy      = CYBSP_D3,
    .discharge = CYBSP_D5,
    .enable    = CYBSP_D4,
    .border    = CYBSP_D6,
    .io_enable = CYBSP_D7
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

    /* Initialize the User LED and LED 8*/
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(RGB_LED_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    

    /* Initialize the PDM/PCM block */
    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);
    

    for(;;)
    {	
		if (!pdm_pcm_flag && dma_transfer_count == 0)
        {
	        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
	        cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
    	}
		
        if (pdm_pcm_flag)
        {
            // audio recorded
            pdm_pcm_flag = false;
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            printf("Ending reading audio\r\n"); 


            signal_t signal;
            ei_impulse_result_t ei_result; 
	        signal.total_length = TOTAL_SAMPLES;
	        signal.get_data = &raw_feature_get_data;
	
            printf("Starting classifier\r\n"); 
	      
            EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &ei_result, false); 
            if (ei_error != EI_IMPULSE_OK) {
                printf("ERROR: run_classifier failed with code %d\r\n", ei_error); 
            } else if (ei_error == EI_IMPULSE_OK){
				printf("DEBUG: running \r\n");
			}

            printf("Classifier returned successfully. Results printing...\r\n"); 
	
			printf("\n****************** \
			RESULTS of classifier \
			****************** \r\n\n");
	        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
	            printf("%s: %.3f\r\n", 
	                ei_result.classification[i].label,
	                ei_result.classification[i].value);
	        }
	        
	        if (ei_result.classification[0].value >= CORRECT_CLASSIFICATION){
				cyhal_gpio_write(RGB_LED_GREEN, CYBSP_LED_STATE_ON);
				led_lighted = !led_lighted;
			} else if (ei_result.classification[2].value >= CORRECT_CLASSIFICATION){
				cyhal_gpio_write(RGB_LED_GREEN, CYBSP_LED_STATE_OFF);
				led_lighted = !led_lighted;
			}
			
			printf("Waiting for next timer...\r\n");
			
			dma_transfer_count = 0;
			current_audio_offset = 0;
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