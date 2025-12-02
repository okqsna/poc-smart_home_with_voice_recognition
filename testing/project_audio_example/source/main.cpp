/******************************************************************************
* File Name:   main.cpp
*******************************************************************************/

extern "C"{
    #include "cyhal.h"
    #include "cybsp.h"
    #include "cy_retarget_io.h"
    #include "stdlib.h"
}

#include "test_sample.h"
#include <string.h>

#include "voice-recognition-cpp-mcu-v3/edge-impulse-sdk/dsp/numpy.hpp"
#include "voice-recognition-cpp-mcu-v3/edge-impulse-sdk/classifier/ei_run_classifier.h"

/*******************************************************************************
* Global audio buffer
********************************************************************************/
int16_t audio_frame[EI_CLASSIFIER_RAW_SAMPLE_COUNT] = {0};

/*******************************************************************************
* Feature getter
********************************************************************************/
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    return ei::numpy::int16_to_float(audio_frame + offset, out_ptr, length);
}

/*******************************************************************************
* MAIN
********************************************************************************/
int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    printf("\x1b[2J\x1b[;H");
    printf("********** TESTING CLASSIFIER WITH PRELOADED SAMPLE **********\r\n");

    /* Copy test audio (already 16000 samples) */
    memcpy(audio_frame, test_audio_16000, sizeof(audio_frame));
    printf("DEBUG: Copied test sample\r\n");

    /* Disable DSP pre-processing (very important) */


    /* Prepare Edge Impulse signal */
    signal_t signal;
    ei_impulse_result_t ei_result;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &raw_feature_get_data;

    printf("DEBUG: Running classifier...\r\n");

    EI_IMPULSE_ERROR ei_error =
        run_classifier(&signal, &ei_result, true);

    printf("DEBUG: run_classifier returned %d\r\n", ei_error);

    if (ei_error == 0)
    {
        printf("\r\nRESULTS:\r\n");
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
        {
            printf("%s: %.3f\r\n",
                   ei_result.classification[i].label,
                   ei_result.classification[i].value);
        }
    }

    while (1)
    {
        cyhal_syspm_sleep();
    }
}
