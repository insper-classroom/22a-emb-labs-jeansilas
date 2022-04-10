#include "help.c"

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
static float get_time_rtt();
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void TC1_Handler(void);
void RTC_Handler(void);
void RTT_Handler(void);
void init_periph(Pio *pio, uint32_t id_pio, uint32_t mask_id_pio  ,char in_out, uint32_t debounce_default, char handler_,void handler_function,char fall_edge ,uint32_t NVIC_priority);
void init_led();