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

// Informações dos pinos X e Y

#define TRIG_PIO				PIOA //PIOC
#define ECHO_PIO				PIOA //PIOD

#define ECHO_PIO_ID			    ID_PIOA //ID_PIOD
#define TRIG_PIO_ID				ID_PIOA //ID_PIOC

#define ECHO_PIO_IDX			13 //28
#define TRIG_PIO_IDX			3//31

#define ECHO_PIO_IDX_MASK		(1 << ECHO_PIO_IDX)
#define TRIG_PIO_IDX_MASK		(1 << TRIG_PIO_IDX)



// Funções de INIT do TC, RTT e RTC

// TC

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


// RTC

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


// RTT

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

// Handlers do TCC*, RTT e RTC.   * No caso do TC, o handler precisa se personalizado.

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);
	
	// Aqui vc coloca a ação que vc quer que aconteça no HANDLER
	
	
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

	 	/* IRQ due to Alarm */
	 	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
	 		RTT_init(4, 0, RTT_MR_RTTINCIEN);
	 	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		//pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);    // BLINK Led
	}

}

void init_periph(Pio *pio, uint32_t id_pio, uint32_t mask_id_pio  ,char in_out, uint32_t debounce_default, char handler_,void handler_function,char fall_edge ,uint32_t NVIC_priority) {
		// *pio -> O  tipo de PIO do Periférico. Ex: PIOA, PIOB, PIOC...
		// id_pio -> O ID do periférico no PIO. EX: ID_PIOA, ID_PIOB, ID_PIOC ...
		// mask_id_pio ->  a Máscará do ID do periférico. EX: LED_PIO_IDX_MASK
		// in_out -> Define se o periférico vai ser para saída ou input: 1 se for INPUT e 0 se for OUTPUT
		// debouce_default -> Define se o periférico vai ter debounce ou não: 1 para debounce e 0 para default ( sem debounce)
		// handler_ -> diz se o periférico vai ter um handler associado a ele. 1 para Handler e 0 para sem Handler.
		// handler_function - > caso tenha handler, passar a função que será chamada no handler.
		// fall_edge -> Quando o handler é acionado, na borda de descida, subida ou ambos. 0 -> Rise; 1 -> Fall; 2 -> Edge
		// NVIC_priority -> define a prioridade na fila de interrupção.
		
		
		
		pmc_enable_periph_clk(id_pio);
		if (in_out) {
			pio_configure(*pio, PIO_INPUT, mask_id_pio, PIO_DEFAULT);
			}else {
			pio_configure(*pio, PIO_OUTPUT_0, mask_id_pio, PIO_DEFAULT);
		}
		
		if (debounce_default) {
			pio_set_debounce_filter(*pio, mask_id_pio, 60);
		}
		
		if (handler_) {
			if (fall_edge == 0) {
				pio_handler_set(*pio,id_pio,mask_id_pio,PIO_IT_RISE_EDGE,handler_function);
			}else if (fall_edge == 1) {
				pio_handler_set(*pio,id_pio,mask_id_pio,PIO_IT_FALL_EDGE,handler_function);
			}else if (fall_edge == 2) {
				pio_handler_set(*pio,id_pio,mask_id_pio,PIO_IT_EDGE,handler_function);
			}
		
			pio_enable_interrupt(*pio, mask_id_pio);
			pio_get_interrupt_status(*pio);
			NVIC_EnableIRQ(id_pio);
			NVIC_SetPriority(id_pio, NVIC_priority); 
		}
		
	
	
}

void init_led() {
	init_periph(LED_PIO,LED_PIO_ID,LED_PIO_IDX_MASK,0,0,0,0,0,0);
	pio_set(LED_PIO,LED_PIO_IDX_MASK);
	init_periph(LED1_PIO,LED1_PIO_ID,LED1_PIO_IDX_MASK,0,0,0,0,0,0);
	pio_set(LED_PIO,LED_PIO_IDX_MASK);
	init_periph(LED2_PIO,LED2_PIO_ID,LED2_PIO_IDX_MASK,0,0,0,0,0,0);
	pio_set(LED_PIO,LED_PIO_IDX_MASK);
	init_periph(LED3_PIO,LED3_PIO_ID,LED3_PIO_IDX_MASK,0,0,0,0,0,0);
}