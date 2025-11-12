//
// Microphone initialization
//

// includes
extern "C"{
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_clock.h"
#include "cy_retarget_io.h"
#include "cyhal_pdmpcm.h"
}

#include "edge-impulse/edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <stdint.h>
#include <stdbool.h>

// constants
#define SAMPLE_RATE_HZ      16000u
#define DECIMATION_RATE     64u
#define PDM_DATA            P10_5
#define PDM_CLK             P10_4
#define AUDIO_SYS_CLOCK_HZ  24576000u

static cyhal_clock_t audio_clock;
static cyhal_pdm_pcm_t pdm_pcm;
static volatile bool pdm_flag = false;

static cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = {
	.sample_rate     = SAMPLE_RATE_HZ,
	.decimation_rate = DECIMATION_RATE,
	.mode            = CYHAL_PDM_PCM_MODE_LEFT,
	.word_length     = 16,
	.left_gain       = 20,
	.right_gain      = 0,
};

static void pdm_isr(void *arg, cyhal_pdm_pcm_event_t event)
{
	if (event & CYHAL_PDM_PCM_ASYNC_COMPLETE)
	{
		pdm_flag = true;
	}
}

bool microphone_clock_init(void)
{
	cy_rslt_t result;
	cyhal_clock_t pll_clock;

	/* Initialize the PLL */
	result = cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: failed to reserve PLL clock (0x%04lx)\n", result);
		return false;
	}

	result = cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: failed to Set PLL freq (0x%04lx)\n", result);
		return false;
	}

	result = cyhal_clock_set_enabled(&pll_clock, true, true);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: failed to Enable PLL (0x%04lx)\n", result);
		return false;
	}

	/* Initialize the audio subsystem clock HF[1] */
	result = cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: failed to Reserve HF[1] (0x%04lx)\n", result);
		return false;
	}

	/* Source the audio subsystem clock from PLL */
	result = cyhal_clock_set_source(&audio_clock, &pll_clock);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: failed to Set HF[1] source = PLL (0x%04lx)\n", result);
		return false;
	}

	result = cyhal_clock_set_enabled(&audio_clock, true, true);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: failed to Enable HF[1] (0x%04lx)\n", result);
		return false;
	}

	ei_printf("Audio clock configured at %lu Hz\n", AUDIO_SYS_CLOCK_HZ);

	return true;
}

bool microphone_pdm_init(void)
{
	cy_rslt_t result;
	ei_printf("Init PDM: DATA=P10_5, CLK=P10_4, Rate=%lu Hz, Decim=%lu\n",
	          pdm_pcm_cfg.sample_rate, pdm_pcm_cfg.decimation_rate);
	
	result = cyhal_pdm_pcm_init(&pdm_pcm, P10_5, P10_4, &audio_clock, &pdm_pcm_cfg);
	if (result != CY_RSLT_SUCCESS) {
	    ei_printf("ERR: cyhal_pdm_pcm_init failed (0x%08lx)\n", result);
	    return false;
	}


	cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_isr, NULL);
	cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);

	result = cyhal_pdm_pcm_start(&pdm_pcm);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: failed to start PDM interface (0x%04lx)\n", result);
		return false;
	}

	ei_printf("PDM/PCM block configured and started successfully at %lu Hz\n",
				 pdm_pcm_cfg.sample_rate);
	return true;
}

bool microphone_state(void)
{
	return pdm_flag;
}

bool microphone_start(int16_t *buffer, size_t samples)
{
	pdm_flag = false;
	cy_rslt_t result;

	result = cyhal_pdm_pcm_read_async(&pdm_pcm, buffer, samples);
	if (result != CY_RSLT_SUCCESS) {
		ei_printf("ERR: read_async failed (0x%04lx)\n", result);
		return false;
	}
	return true;
}

void microphone_stop(void)
{
	cyhal_pdm_pcm_abort_async(&pdm_pcm);
	cyhal_pdm_pcm_stop(&pdm_pcm);
	cyhal_pdm_pcm_free(&pdm_pcm);
}