extern "C" {
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "stdlib.h"
}
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Define how many samples in a frame */
#define FRAME_SIZE                  (3960)
/* Volume ratio for noise and print purposes */
#define VOLUME_RATIO                (4*FRAME_SIZE)
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
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);


float input_buffer[FRAME_SIZE];
/*******************************************************************************
* Function from Edge Impulse, retrieves slices of data required by the DSP process.
********************************************************************************/
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
	memcpy(out_ptr, input_buffer + offset, length * sizeof(float));
	return 0;
}

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Interrupt flags */
volatile bool pdm_pcm_flag = true;

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
	.left_gain       = 24,   /* dB */
	.right_gain      = 24,   /* dB */
};

int main(void)
{
    cy_rslt_t result;
    int16_t  audio_frame[FRAME_SIZE] = {0};

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Init the clocks */
	printf("Init clocks...\r\n");
    clock_init();
	printf("Clock init done\r\n");

	cyhal_system_delay_ms(200);   
    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	cyhal_system_delay_ms(50);            

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the PDM/PCM block */
    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
	cy_rslt_t pdm_result = cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
	printf("PDM init result = 0x%08lX\r\n", pdm_result);

    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);
	printf("PDM/PCM started. Waiting for audio frames...\r\n");


    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** \
    PDM/PCM Example \
    ****************** \r\n\n");

    for(;;)
    {
        /* Check if any microphone has data to process */
        if (pdm_pcm_flag)
        {
            /* Clear the PDM/PCM flag */
            pdm_pcm_flag = 0;
        	// wrapper for data
        	signal_t signal;
        	// to store inference output, result of classification
        	ei_impulse_result_t result_inference;
        	// return code from inference
        	EI_IMPULSE_ERROR res;

        	for (uint32_t index = 0; index < FRAME_SIZE; ++index)
        	{
        		input_buffer[index] = (float)audio_frame[index]/ 32768.0f;
        	}

        	// checking if buffer size is the same as input, the correctness of the transmitted data
        	size_t buf_len = sizeof(input_buffer) / sizeof(input_buffer[0]);

        	if (buf_len != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        		printf("ERROR: The size of the input buffer is not correct.\n");
        		printf("Expected %d items, but got %d\n",
					   EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, (int)buf_len);
        		cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
        		continue;
        	}

        	// information about data for buffer
        	signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        	signal.get_data = &raw_feature_get_data;

        	cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        	// running classifier itself
        	res = run_classifier(&signal, &result_inference, false);
        	cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

        	printf("\nPredictions made by classifier:\n");
        	for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        		printf("%s: %.5f\n",
					   result_inference.classification[ix].label, // class
					   result_inference.classification[ix].value); // probability
        	}

        	printf("\nTime spent: DSP %d ms, classification %d ms\n",
				  result_inference.timing.dsp, result_inference.timing.classification);


            /* Setup to read the next frame */
            cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
        }
        cyhal_syspm_sleep();

    }
}


/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
*  PDM/PCM ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;

    pdm_pcm_flag = true;
	printf("ISR: new audio frame ready\r\n");
}

/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
*  Initialize the clocks in the system.
*
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
