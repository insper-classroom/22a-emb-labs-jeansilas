#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

 

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






// Inicialização dos leds

void io_init(void)
{

	// Configura Leds
	
	// placa
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

}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;

  // Init OLED
	gfx_mono_ssd1306_init();
	
  
  
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
    gfx_mono_draw_string("mundo", 50,16, &sysfont);
  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {

	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

		
			
	}
}
