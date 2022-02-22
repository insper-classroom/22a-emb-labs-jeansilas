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
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	
	// configura pino ligado ao botão como entrada com um pull-up.
	pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK,PIO_DEFAULT);
	
	// Ativa o pull-up no PIO
	pio_pull_up(BUT_PIO,BUT_PIO_IDX_MASK,1);
	pio_pull_up(BUT1_PIO,BUT1_PIO_IDX_MASK,1);
	pio_pull_up(BUT2_PIO,BUT2_PIO_IDX_MASK,1);
	pio_pull_up(BUT3_PIO,BUT3_PIO_IDX_MASK,1);

	
	
	



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
	  char button_pressed = !pio_get(BUT_PIO,PIO_INPUT,BUT_PIO_IDX_MASK) ;
	  char button1_pressed = !pio_get(BUT1_PIO,PIO_INPUT,BUT1_PIO_IDX_MASK) ;
	  char button2_pressed = !pio_get(BUT2_PIO,PIO_INPUT,BUT2_PIO_IDX_MASK) ;
	  char button3_pressed = !pio_get(BUT3_PIO,PIO_INPUT,BUT3_PIO_IDX_MASK) ;
	  
	  int n = 0; 
	  int n1 = 0;
	  int n2 = 0;
	  int n3 = 0;
	  
	  pio_set(PIOC,LED2_PIO_IDX_MASK);
	  pio_set(PIOC,LED_PIO_IDX_MASK);
	  pio_set(PIOA,LED1_PIO_IDX_MASK);
	  pio_set(PIOB,LED3_PIO_IDX_MASK);
	  
	  // botao-led principal
	  
	  while (n <10 && button_pressed) {
	  pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
	  delay_ms(1000);                        // Delay por software de 200 ms
	  pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
	  n++;
	  delay_ms(1000);                        // Delay por software de 200 ms
	  }
	  
	  //botao-led 1
	  
	  while (n1 <10 && button1_pressed) {
		  pio_clear(PIOA, LED1_PIO_IDX_MASK);      // Coloca 1 no pino LED
		  delay_ms(1000);                        // Delay por software de 200 ms
		  pio_set(PIOA, LED1_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		  n1++;
		  delay_ms(1000);                        // Delay por software de 200 ms
	  }
	  
	  //botao-led 2
	  
	  while (n2 <10 && button2_pressed) {
		  pio_clear(PIOC, LED2_PIO_IDX_MASK);      // Coloca 1 no pino LED
		  delay_ms(1000);                        // Delay por software de 200 ms
		  pio_set(PIOC, LED2_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		  n2++;
		  delay_ms(1000);                        // Delay por software de 200 ms
	  }
	  
	  //botao-led 3
	  
	  while (n3 <10 && button3_pressed) {
		  pio_clear(PIOB, LED3_PIO_IDX_MASK);      // Coloca 1 no pino LED
		  delay_ms(1000);                        // Delay por software de 200 ms
		  pio_set(PIOB, LED3_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		  n3++;
		  delay_ms(1000);                        // Delay por software de 200 ms
	  }
  }
  return 0;
}
