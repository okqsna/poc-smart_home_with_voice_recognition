extern "C"{
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_clock.h"
#include "cyhal_pdmpcm.h"
#include "cy_retarget_io.h"
}

#include "edge-impulse/edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <stdint.h>
#include <stdbool.h>

extern bool microphone_clock_init(void);
extern bool microphone_pdm_init();
extern bool microphone_start(int16_t *buffer, size_t samples);
extern bool microphone_state(void);
extern void microphone_stop(void);

#define FRAME_SIZE 16000
static int16_t buffer[FRAME_SIZE];

int main(void)
{
	cybsp_init();

	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 115200);

	if (!microphone_clock_init()) {
		ei_printf("Clock init failed!\n");
		return 1;
	}

	cyhal_system_delay_ms(300);

	if (!microphone_pdm_init()) {
		ei_printf("PDM block init failed!\n");
		return 1;
	}

	microphone_start(buffer, FRAME_SIZE);

	while (!microphone_state()) {
		cyhal_system_delay_ms(10);
	}

	for (int i = 0; i < 10; i++) {
		ei_printf("%d ", buffer[i]);
	}

	microphone_stop();

	while (1) {
		cyhal_system_delay_ms(1000);
	}
}
