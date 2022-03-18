#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;
 

// Configurações do Led OLED e da placa

#define LED_PIO             PIOC   
#define LED1_PIO			PIOA
#define LED2_PIO			PIOC
#define LED3_PIO			PIOB


#define LED_PIO_ID			ID_PIOC
#define LED1_PIO_ID			ID_PIOA
#define LED2_PIO_ID			ID_PIOC
#define LED3_PIO_ID			ID_PIOB

#define LED_PIO_IDX			8	
#define LED1_PIO_IDX		0
#define LED2_PIO_IDX		30
#define LED3_PIO_IDX		2

		
#define LED_PIO_IDX_MASK    (1 << LED_PIO_IDX)
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

// Configurações do botão OLED e da placa


#define BUT_PIO				PIOA
#define BUT1_PIO			PIOD
#define BUT2_PIO			PIOC
#define BUT3_PIO			PIOA

#define BUT_PIO_ID			ID_PIOA
#define BUT1_PIO_ID			ID_PIOD
#define BUT2_PIO_ID			ID_PIOC
#define BUT3_PIO_ID			ID_PIOA


#define BUT_PIO_IDX			11
#define BUT1_PIO_IDX		28
#define BUT2_PIO_IDX		31
#define BUT3_PIO_IDX		19


#define BUT_PIO_IDX_MASK (1 << BUT_PIO_IDX) 
#define BUT1_PIO_IDX_MASK  (1 << BUT1_PIO_IDX)
#define BUT2_PIO_IDX_MASK  (1 << BUT2_PIO_IDX)
#define BUT3_PIO_IDX_MASK  (1 << BUT3_PIO_IDX)

// flags volatiles

volatile char flag_rtc_alarm = 0;
volatile char flag_tc2_button = 0;
volatile char itera = 0;

// Prototype

void pin_toggle(Pio *pio, uint32_t mask);
void pisca_led (int n, int t);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
// Handlers

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}


void but1_callback () {
	
	flag_tc2_button = 1;
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);  
}

void TC7_Handler(void) {
	
	itera++;
	
	
	volatile uint32_t status = tc_get_status(TC2, 1);

	/** Muda o estado do LED (pisca) **/
	//pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
	
}

void TC6_Handler(void) {
	
	
	
	volatile uint32_t status = tc_get_status(TC2, 0);

	
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		// o código para irq de segundo vem aqui
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o código para irq de alame vem aqui
		flag_rtc_alarm = 1;
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}


void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

// 	/* IRQ due to Alarm */
// 	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
// 		RTT_init(4, 0, RTT_MR_RTTINCIEN);
// 	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);    // BLINK Led
	}

}

void pisca_led (int n, int t) {
	for (int i=0;i<n;i++){
		pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
		delay_ms(t);
	}
}


// Funções relacionadas ao TC

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

static float get_time_rtt(){
  uint ul_previous_time = rtt_read_timer_value(RTT); 
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
		rtt_enable_interrupt(RTT, rttIRQSource);
  else
		rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}






// Inicialização dos leds

void io_init(void)
{
	
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_handler_set(BUT1_PIO,BUT1_PIO_ID,BUT1_PIO_IDX_MASK,PIO_IT_RISE_EDGE,but1_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 3); // Prioridade 4

	// Configura Leds
	
	// placa
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED_PIO,LED_PIO_IDX_MASK);
	
	// 1 Oled
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED1_PIO,LED1_PIO_IDX_MASK);
	
	// 2 Oled
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED2_PIO,LED2_PIO_IDX_MASK);
	// 3 Oled
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED3_PIO,LED3_PIO_IDX_MASK);

}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	io_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;

  // Init OLED
	gfx_mono_ssd1306_init();
	
	char data[30];
	
  
  
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
    //gfx_mono_draw_string("mundo", 50,16, &sysfont);
	
	
	TC_init(TC0, ID_TC1, 1, 4);
	tc_start(TC0, 1);
	
	// TC do dispplay
	
	TC_init(TC3,ID_TC6,0,1);
	tc_start(TC2,0);
	

	
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
		
		/* Leitura do valor atual do RTC */
		uint32_t current_hour, current_min, current_sec;
		uint32_t current_year, current_month, current_day, current_week;
	
	// RTC
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	
	sprintf(data, "%d:%d:%d",current_hour,current_min,current_sec);
	gfx_mono_draw_string(data, 25,16, &sysfont);
	
	
		

	
	 /* configura alarme do RTC para daqui 20 segundos */
	 rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
	 rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 2);
	 
	 //RTT
	 RTT_init(0.25, 0, RTT_MR_RTTINCIEN);  
	 
  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		
			  if (itera >= 20) {
				  
				  pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
				   rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
				   rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
				   sprintf(data, "%d:%d:%d  ",current_hour,current_min,current_sec);
				   gfx_mono_draw_string(data, 25,16, &sysfont);
				  delay_ms(1000);
				  pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
				   rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
				   rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
				   sprintf(data, "%d:%d:%d  ",current_hour,current_min,current_sec);
				   gfx_mono_draw_string(data, 25,16, &sysfont);
				  delay_ms(1000);
				  
				  tc_stop(TC2,0);
				  itera = 0;
				  
			  }
				if (flag_tc2_button){
				  
				  TC_init(TC2,ID_TC7,1, 1);
				  tc_start(TC2,1);
				 
				  flag_tc2_button = 0;
			  }
			  
			  rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			  rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			  sprintf(data, "%d:%d:%d  ",current_hour,current_min,current_sec);
			  gfx_mono_draw_string(data, 25,16, &sysfont);

	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

		
			
	}
}
