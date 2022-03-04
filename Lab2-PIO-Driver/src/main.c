/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO             PIOC        //Periférico que controla o LED  
#define LED_PIO_ID			ID_PIOC      
#define LED_PIO_IDX			8			// ID do periférico PIOC ( controla o LED)  
#define LED_PIO_IDX_MASK    (1 << LED_PIO_IDX) // Mascara para controlarmos o LED
// Configurações do botão

#define BUT_PIO				PIOA
#define BUT_PIO_ID			ID_PIOA
#define BUT_PIO_IDX			11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto

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

#define BUT1_PIO_IDX_MASK  (1u << BUT1_PIO_IDX)
#define BUT2_PIO_IDX_MASK  (1u << BUT2_PIO_IDX)
#define BUT3_PIO_IDX_MASK  (1u << BUT3_PIO_IDX)

// Configurações do Led

#define LED1_PIO			PIOA
#define LED2_PIO			PIOC
#define LED3_PIO			PIOB


#define LED1_PIO_ID			ID_PIOA
#define LED2_PIO_ID			ID_PIOC
#define LED3_PIO_ID			ID_PIOB	

#define LED1_PIO_IDX		0
#define LED2_PIO_IDX		30
#define LED3_PIO_IDX		2

#define LED1_PIO_IDX_MASK  (1u << LED1_PIO_IDX)
#define LED2_PIO_IDX_MASK  (1u << LED2_PIO_IDX)
#define LED3_PIO_IDX_MASK  (1u << LED3_PIO_IDX)

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)



/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}


void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable){
	
	if (ul_pull_up_enable){
		p_pio->PIO_PUER = ul_mask;
	}
	else {
		p_pio->PIO_PUDR = ul_mask;
	}
}

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,
const uint32_t ul_attribute)
{
	if (ul_attribute && PIO_DEGLITCH ) {
		p_pio->PIO_IFSCDR = ul_mask;
	}
	
	else {
		p_pio->PIO_IFSCER = ul_mask;
	}
}

void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,
const uint32_t ul_default_level,
const uint32_t ul_multidrive_enable,
const uint32_t ul_pull_up_enable)
{
	if (ul_default_level) {
		_pio_set(p_pio,ul_mask);
	}
	
	else {
		_pio_clear(p_pio,ul_mask);
	}
	
	if (ul_multidrive_enable) {
		p_pio->PIO_MDER = ul_mask ;
	}
	
	else {
		p_pio->PIO_MDDR = ul_mask ;
	}
	p_pio->PIO_PER = ul_mask ;
	p_pio->PIO_OER = ul_mask ;
	
	_pio_pull_up(p_pio,ul_mask,ul_pull_up_enable);

}



uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type,
const uint32_t ul_mask)
{
	uint32_t input_output_mask;

	if ( ul_type == PIO_OUTPUT_0 ) {
		
		input_output_mask = p_pio->PIO_ODSR;
		
		} 

	else if (ul_type == PIO_INPUT) {
		input_output_mask = p_pio->PIO_PDSR;
	}

	if ((input_output_mask & ul_mask) == 0) {
		return 0;
		} 
	
	else {
		return 1;
	}
}

void _delay_ms(int n) {
	
	for (int i =0; i < 150000*n;i++) {
		asm("NOP");
	}
}

// Função de inicialização do uC
void init(void)
{
	// Inicializar o board clock
	
	sysclk_init();
	
	// Desativa WatchDog Timer
	WDT ->WDT_MR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	
	// Inicializa PIO do botão
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	//Inicializa PC8 como saída
	_pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	
	// configura pino ligado ao botão como entrada com um pull-up.
	_pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	

	
	
	



}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	  char button_pressed = !_pio_get(BUT_PIO,PIO_INPUT,BUT_PIO_IDX_MASK) ;
	  char button1_pressed = !_pio_get(BUT1_PIO,PIO_INPUT,BUT1_PIO_IDX_MASK) ;
	  char button2_pressed = !_pio_get(BUT2_PIO,PIO_INPUT,BUT2_PIO_IDX_MASK) ;
	  char button3_pressed = !_pio_get(BUT3_PIO,PIO_INPUT,BUT3_PIO_IDX_MASK) ;
	  
	  int n = 0; 
	  int n1 = 0;
	  int n2 = 0;
	  int n3 = 0;
	  
	  _pio_set(PIOC,LED2_PIO_IDX_MASK);
	  _pio_set(PIOC,LED_PIO_IDX_MASK);
	  _pio_set(PIOA,LED1_PIO_IDX_MASK);
	  _pio_set(PIOB,LED3_PIO_IDX_MASK);
	  
	  // botao-led principal
	  
	  while (n <10 && button_pressed) {
	  _pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
	  delay_ms(1000);                        // Delay por software de 200 ms
	  _pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
	  n++;
	  _delay_ms(1000);                        // Delay por software de 200 ms
	  }
	  
	  //botao-led 1
	  
	  while (n1 <10 && button1_pressed) {
		  _pio_clear(PIOA, LED1_PIO_IDX_MASK);      // Coloca 1 no pino LED
		  delay_ms(1000);                        // Delay por software de 200 ms
		  _pio_set(PIOA, LED1_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		  n1++;
		  _delay_ms(1000);                        // Delay por software de 200 ms
	  }
	  
	  //botao-led 2
	  
	  while (n2 <10 && button2_pressed) {
		  _pio_clear(PIOC, LED2_PIO_IDX_MASK);      // Coloca 1 no pino LED
		  _delay_ms(1000);                        // Delay por software de 200 ms
		  _pio_set(PIOC, LED2_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		  n2++;
		  _delay_ms(1000);                        // Delay por software de 200 ms
	  }
	  
	  //botao-led 3
	  
	  while (n3 <10 && button3_pressed) {
		  _pio_clear(PIOB, LED3_PIO_IDX_MASK);      // Coloca 1 no pino LED
		  _delay_ms(1000);                        // Delay por software de 200 ms
		  _pio_set(PIOB, LED3_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		  n3++;
		  _delay_ms(1000);                        // Delay por software de 200 ms
	  }
  }
  return 0;
}
