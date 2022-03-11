#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define n_piscadas 30

#define LED_PIO             PIOC        //Periférico que controla o LED
#define LED_PIO_ID			ID_PIOC
#define LED_PIO_IDX			8			// ID do periférico PIOC ( controla o LED)
#define LED_PIO_IDX_MASK    (1 << LED_PIO_IDX) // Mascara para controlarmos o LED
// Configurações do botão

#define BUT_PIO				PIOA
#define BUT_PIO_ID			ID_PIOA
#define BUT_PIO_IDX			11
#define BUT_PIO_IDX_MASK (1 << BUT_PIO_IDX) 

// Configurações ds botões OLED

#define BUT1_PIO			PIOD
#define BUT2_PIO			PIOC
#define BUT3_PIO			PIOA

#define BUT1_PIO_ID			ID_PIOD
#define BUT2_PIO_ID			ID_PIOC
#define BUT3_PIO_ID			ID_PIOA

#define BUT1_PIO_IDX		28
#define BUT2_PIO_IDX		31
#define BUT3_PIO_IDX		19

#define BUT1_PIO_IDX_MASK  (1 << BUT1_PIO_IDX)
#define BUT2_PIO_IDX_MASK  (1 << BUT2_PIO_IDX)
#define BUT3_PIO_IDX_MASK  (1 << BUT3_PIO_IDX)

// Configurações do Led OLED

#define LED1_PIO			PIOA
#define LED2_PIO			PIOC
#define LED3_PIO			PIOB


#define LED1_PIO_ID			ID_PIOA
#define LED2_PIO_ID			ID_PIOC
#define LED3_PIO_ID			ID_PIOB

#define LED1_PIO_IDX		0
#define LED2_PIO_IDX		30
#define LED3_PIO_IDX		2

#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

volatile char but_flag = 0 ;

volatile char but1_flag_up = 0 ;

volatile char but1_flag_down = 0 ;

volatile char but2_flag = 0 ;

volatile char but3_flag = 0 ;

void but_callback() {
	
	but_flag = 1;
}

void but1_callback() {
		
		if(pio_get(BUT1_PIO,PIO_INPUT,BUT1_PIO_IDX_MASK)) {
			
			but1_flag_down = 0 ;
			but1_flag_up = 1;
		}
		
		else {
			
			but1_flag_down = 1 ;
			but1_flag_up = 0;
			
		}
	
}

void but2_callback() {
	but2_flag = !but2_flag;
}

void but3_callback() {
	but3_flag = 1;
}

void io_init(void)
{

	// Configura led
	
	// principal
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEFAULT);
	
	// 1 Oled
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	
	// 2 Oled
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	
	// 3 Oled
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	
	

	// Inicializa clock do periférico PIO responsavel pelo botao
	// Principal
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	// 1 Oled
	pmc_enable_periph_clk(BUT1_PIO_ID);
	
	// 2 Oled
	pmc_enable_periph_clk(BUT2_PIO_ID);
	
	// 3 Oled
	pmc_enable_periph_clk(BUT3_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	//pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	
	// principal
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_IDX_MASK, 60);
	
	// 1 Oled
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	// 2 Oled
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	
	// 3 Oled
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	
	// principal
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_callback);
	
	// 1 Oled
	
	// quick press
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but1_callback);
	
// 	long press
// 		pio_handler_set(BUT1_PIO,
// 		BUT1_PIO_ID,
// 		BUT1_PIO_IDX_MASK,
// 		PIO_IT_FALL_EDGE,
// 		but1long_callback);
	
	// 2 Oled
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but2_callback);
	
	// 3 Oled
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but3_callback);
	
	


	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	
	// principal
	pio_enable_interrupt(BUT_PIO, BUT_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_PIO);
	
	// 1 Oled
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// 2 Oled
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	
	// 3 Oled
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	
	// Principal
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
	
	// 1 Oled
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 2); // Prioridade 2
	
	// 2 Oled
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 1); // Prioridade 4
	
	// 3 Oled
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
}

void increase_freq(int *freq) {
	if (*freq <= 100) {
		*freq -= 10;
	}
	else {
		
		*freq -= 100;
	}
	
	//*freq =  1/ *freq ;
	
	char freq_str[128];
	sprintf(freq_str, "%d ms", *freq); //
	
	gfx_mono_draw_string("       ", 32, 16, &sysfont);
	gfx_mono_draw_string(freq_str, 32, 16, &sysfont);
}

void decrease_freq (int *freq) {
	
	if (*freq <= 100) {
		*freq += 10;
	}
	else {
		
		*freq += 100;
	}
	
	//*freq =  1/ *freq ;
	
	char freq_str[128];
	sprintf(freq_str, "%d ms", *freq); //
	
	gfx_mono_draw_string("       ", 32, 16, &sysfont);
	gfx_mono_draw_string(freq_str, 32, 16, &sysfont);
}

void progress_bar (int n, int clean) {
	
	int bar_complete = 120;
	int bar_height = 10;
	
	
	if (n < 0){
		n = 1;
	}
	
	
	int per_complete = (bar_complete*n)/30;
	
	if (n == 29) {
		 per_complete = 120;
	}
	
	if (!clean) {
		
	
	gfx_mono_generic_draw_filled_rect(0,1,per_complete,bar_height,1);
	gfx_mono_generic_draw_rect(0,1,bar_complete,bar_height,1);
	}
	
	else {
		gfx_mono_draw_string("             ", 0,1, &sysfont);
	}
	
}

// pisca led N vez no periodo T
void pisca_led(int n, int *t){
	progress_bar(30,1);
	for (int i=0;i<n;i++){
		progress_bar(i,0);
		if (but1_flag_up) {
			increase_freq(t);
			
			but1_flag_up = 0;
			
			break;
			
			
		}
		
		if (but1_flag_down) {
			
			delay_ms(200);
			while (but1_flag_down) {
				delay_ms(200);
				decrease_freq(t);
			}
			
			
			
		}
		
		if (but2_flag) {
			
			
			break ;
		}
		
		if (but3_flag) {
			decrease_freq(t);
			
			but3_flag = 0;
			
			break;
			
			
		}
		
		pio_clear(LED_PIO, LED_PIO_IDX_MASK);
		delay_ms(*t);
		if (but1_flag_up) {
			increase_freq(t);
			
			but1_flag_up = 0;
			
			break;
			
			
		}
		
		if (but1_flag_down) {
			
			delay_ms(200);
			while (but1_flag_down) {
				delay_ms(200);
				decrease_freq(t);
			}
			
		}
		
		if (but2_flag) {
			break ;
		}
		
		if (but3_flag) {
			decrease_freq(t);
			
			but3_flag = 0;
			
			break;
			
			
		}
		pio_set(LED_PIO, LED_PIO_IDX_MASK);
		delay_ms(*t);
		
		
	}
}




int main (void)
{
	WDT->WDT_MR = WDT_MR_WDDIS;

	board_init();
	sysclk_init();
	delay_init();
	
	io_init();

  // Init OLED
	gfx_mono_ssd1306_init();
	
	int freq = 1000;
	char freq_str[128]; 
  // Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
  //gfx_mono_draw_string("Jean", 50,16, &sysfont);
  


  sprintf(freq_str, "%d ms", freq); //

  gfx_mono_draw_string(freq_str, 32, 16, &sysfont);

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		
		pisca_led(30,&freq);
		
// 		if (but_flag) {
// 			
// 			but_flag = 0;
// 		}
// 		
// 		if (but1_flag) {
// 			
// 			increase_freq(&freq);
// 			
// 			but1_flag = 0;
// 		} 
// 		
// 		if (but1long_flag) {
// 			
// 			delay_ms(1000);
// 			
// 			if (!but1_flag) {
// 			decrease_freq(&freq);
// 			}
// 		}
// 		
// 		if (but2_flag) {
// 			
// 			pisca_led(n_piscadas,&freq);
// 			
// 		}
// 		
// 		if (but3_flag) {
// 			
// 			but3_flag = 0;
// 		}
		
		
		 pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		

	}
}
