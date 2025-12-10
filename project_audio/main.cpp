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

#define SAMPLE_RATE_HZ              16000u
#define DECIMATION_RATE             64u
#define AUDIO_SYS_CLOCK_HZ          24576000u

#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4
#define CORRECT_CLASSIFICATION      0.5

#define EXT_LED_RED             P9_6
#define EXT_LED_GREEN           P10_6
#define EXT_LED_BLUE            P9_7
#define EXT_LED_WHITE           P6_2   // був YELLOW — тепер це "білий"

#define RGB_LED_GREEN           P1_1
#define RGB_LED_RED             P0_3
#define RGB_LED_BLUE            P11_1

/*******************************************************************************
* Function Prototypes
********************************************************************************/
extern "C" {
     void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
     void timer_isr_handler(void *callback_arg, cyhal_timer_event_t event);
     void blinking_timer_isr_handler(void *callback_arg, cyhal_timer_event_t event);
     void clock_init(void);
     void timer_init(void);
     void blinking_timer_init(void);
     void led_init(void);

     void set_red(bool command);
     void set_green(bool command);
     void set_blue(bool command);
     void set_white(bool command);
     void blinking_mode(void);
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);

/*******************************************************************************
* Global Variables
********************************************************************************/
bool pdm_pcm_flag = false;
bool timer_interrupt_flag = false;
bool blink_interrupt_flag = false;

int16_t audio_frame[TOTAL_SAMPLES] = {0};
uint32_t current_audio_offset = 0;
uint32_t dma_transfer_count = 0;

typedef struct {
    bool red;
    bool green;
    bool blue;
    bool white;
    bool blink;
} led_controller;

led_controller ledStates = {false, false, false, false, false};

cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;

cyhal_timer_t timer_obj;
cyhal_timer_t blink_timer_obj;

/* PWM for white LED */
cyhal_pwm_t white_pwm;

/* PDM config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg =
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_LEFT,
    .word_length     = 16,
    .left_gain       = 0,
    .right_gain      = 0,
};

const cyhal_timer_cfg_t timer_cfg =
{
    .is_continuous = true,
    .direction     = CYHAL_TIMER_DIR_UP,
    .is_compare    = false,
    .period        = 70000,
    .compare_value = 0,
    .value         = 0
};

const cyhal_timer_cfg_t blinking_timer_cfg =
{
    .is_continuous = true,
    .direction     = CYHAL_TIMER_DIR_UP,
    .is_compare    = false,
    .period        = 5000,
    .compare_value = 0,
    .value         = 0
};

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    return ei::numpy::int16_to_float(audio_frame + offset, out_ptr, length);
};

/*******************************************************************************
* PWM fade functions
*******************************************************************************/
void fade_white(bool on)
{
    if(on) {
        for(int d = 0; d <= 100; d++){
            cyhal_pwm_set_duty_cycle(&white_pwm, d, 1000);
            cyhal_system_delay_ms(5);
        }
    } else {
        for(int d = 100; d >= 0; d--){
            cyhal_pwm_set_duty_cycle(&white_pwm, d, 1000);
            cyhal_system_delay_ms(5);
        }
    }
    ledStates.white = on;
}

/*******************************************************************************
* main
********************************************************************************/
int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS){ CY_ASSERT(0); }
    __enable_irq();

    clock_init();
    timer_init();
    blinking_timer_init();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    led_init();
	/* ---------------------------
	 *  WHITE LED FADE TEST
	 * --------------------------*/
	printf("Running WHITE LED fade test...\r\n");
	
	// плавно вмикаємо
	fade_white(true);
	cyhal_system_delay_ms(600);
	
	// плавно вимикаємо
	fade_white(false);
	cyhal_system_delay_ms(600);
	
	// робимо три цикли "дихання"
	for(int i = 0; i < 3; i++){
	    fade_white(true);
	    fade_white(false);
	}
	
	printf("Fade test DONE. Starting classifier mode.\r\n");


    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);

    for(;;)
    {
        if (timer_interrupt_flag){
            timer_interrupt_flag = false;
            current_audio_offset = 0;
            dma_transfer_count = 0;

            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
            printf("Starting reading audio\r\n");
            cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
        }

        if (pdm_pcm_flag)
        {
            pdm_pcm_flag = false;
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

            signal_t signal;
            ei_impulse_result_t ei_result;

            signal.total_length = TOTAL_SAMPLES;
            signal.get_data = &raw_feature_get_data;

            run_classifier(&signal, &ei_result, false);

            printf("\nRESULTS:\r\n");
            for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                printf("%s: %.3f\r\n",
                    ei_result.classification[i].label,
                    ei_result.classification[i].value);
            }


            if (ei_result.classification[2].value >= CORRECT_CLASSIFICATION){
                set_red(false);
                set_green(false);
                set_blue(false);
                set_white(true);

            } else if (ei_result.classification[4].value >= CORRECT_CLASSIFICATION){
                set_red(false);
                set_green(false);
                set_blue(false);
                set_white(false);

            } else if (ei_result.classification[5].value >= CORRECT_CLASSIFICATION){
                set_red(true);
                set_green(false);
                set_blue(false);
                set_white(false);

            } else if (ei_result.classification[0].value >= CORRECT_CLASSIFICATION){
                set_red(false);
                set_green(false);
                set_blue(true);
                set_white(false);

            } else if (ei_result.classification[1].value >= CORRECT_CLASSIFICATION){
                set_red(false);
                set_green(true);
                set_blue(false);
                set_white(false);
            }

            else if (ei_result.classification[3].value >= CORRECT_CLASSIFICATION){
                
                bool any_on = ledStates.red || ledStates.green || ledStates.blue || ledStates.white;

                set_red(false);
                set_green(false);
                set_blue(false);

                if(any_on && !ledStates.white){
                    fade_white(false);
                }

                fade_white(true);
            }
        }

        if(blink_interrupt_flag){
            blink_interrupt_flag = false;
            blinking_mode();
        }

        cyhal_syspm_sleep();
    }
}


/*******************************************************************************
* Interrupts
*******************************************************************************/
void timer_isr_handler(void *callback_arg, cyhal_timer_event_t event){
    timer_interrupt_flag = true;
}

void blinking_timer_isr_handler(void *callback_arg, cyhal_timer_event_t event){
    blink_interrupt_flag = true;
}

void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    dma_transfer_count++;

    if (dma_transfer_count >= NUM_DMA_TRANSFERS){
        pdm_pcm_flag = true;
    }
    else {
        current_audio_offset = dma_transfer_count * FRAME_SIZE;
        cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame + current_audio_offset, FRAME_SIZE);
    }
}

/*******************************************************************************
* LED init
*******************************************************************************/
void led_init(void)
{
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    cyhal_gpio_init(RGB_LED_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(RGB_LED_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(RGB_LED_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    cyhal_gpio_init(EXT_LED_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    cyhal_gpio_init(EXT_LED_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    cyhal_gpio_init(EXT_LED_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);

    // PWM for white LED
    cyhal_pwm_init(&white_pwm, EXT_LED_WHITE, NULL);
    cyhal_pwm_set_duty_cycle(&white_pwm, 0, 1000);
    cyhal_pwm_start(&white_pwm);
}

/*******************************************************************************
* Clock + Timer + LED functions
*******************************************************************************/
void timer_init(void)
{
    cyhal_timer_init(&timer_obj, NC, NULL);
    cyhal_timer_configure(&timer_obj, &timer_cfg);
    cyhal_timer_set_frequency(&timer_obj, 10000);
    cyhal_timer_register_callback(&timer_obj, timer_isr_handler, NULL);
    cyhal_timer_enable_event(&timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);
    cyhal_timer_start(&timer_obj);
}

void blinking_timer_init(void)
{
    cyhal_timer_init(&blink_timer_obj, NC, NULL);
    cyhal_timer_configure(&blink_timer_obj, &blinking_timer_cfg);
    cyhal_timer_set_frequency(&blink_timer_obj, 10000);
    cyhal_timer_register_callback(&blink_timer_obj, blinking_timer_isr_handler, NULL);
    cyhal_timer_enable_event(&blink_timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);
    cyhal_timer_start(&blink_timer_obj);
}

void clock_init(void)
{
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

void set_red(bool command){
    ledStates.red = command;
    cyhal_gpio_write(RGB_LED_RED, command);
    cyhal_gpio_write(EXT_LED_RED, !command);
}

void set_blue(bool command){
    ledStates.blue = command;
    cyhal_gpio_write(RGB_LED_BLUE, command);
    cyhal_gpio_write(EXT_LED_BLUE, !command);
}

void set_green(bool command){
    ledStates.green = command;
    cyhal_gpio_write(RGB_LED_GREEN, command);
    cyhal_gpio_write(EXT_LED_GREEN, !command);
}

void set_white(bool command){
    fade_white(command);
}

void blinking_mode()
{
    if (ledStates.blink)
    {
        if (ledStates.red)
        {
            cyhal_gpio_toggle(RGB_LED_RED);
        }

        if (ledStates.green)
        {
            cyhal_gpio_toggle(RGB_LED_GREEN);
        }

        if (ledStates.blue)
        {
            cyhal_gpio_toggle(RGB_LED_BLUE);
        }
    }
}
