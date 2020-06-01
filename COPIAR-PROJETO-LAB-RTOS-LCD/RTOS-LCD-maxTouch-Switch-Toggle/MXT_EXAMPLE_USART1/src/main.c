#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_uart_serial.h"
#include "maxTouch/maxTouch.h"


#include "fonts/tfont.h"
#include "fonts/calibri_36.h"
#include "fonts/calibri_36_dark.h"
#include "fonts/calibri_23.h"
#include "fonts/calibri_23_button.h"
#include "fonts/calibri_23_dark.h"


#include "images/play_image.h"
#include "images/play_image_dark.h"
#include "images/pause_image.h"
#include "images/pause_image_dark.h"
#include "images/restart_image.h"
#include "images/restart_image_dark.h"
#include "images/dark.h"
#include "images/light.h"

#include "other/botoes.h"




/************************************************************************/
/* structs                                                         */
/************************************************************************/
typedef struct {
	uint x;
	uint y;
} touchData;

typedef struct
{
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

typedef struct {
	uint32_t width;     
	uint32_t height;	 
	uint32_t border;	
	uint8_t status;
	void (*callback)(t_but_image);
	uint32_t x;         
	uint32_t y;         
} t_but_image;

// Button
// typedef struct {
// 	uint32_t width;     
// 	uint32_t height;  
// 	uint32_t color;
// 	uint32_t x;         
// 	uint32_t y;         
// } t_but;



/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        10

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

//Prototypes
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void but_play_callback(t_but_image *b);
void but_color_mode_callback(t_but_image *b);
void but_reset_callback(t_but_image *b);
void button_screen_callback(void);
void but_board_callback(void);
void animate_speed(void);


//Variables
int pulses = 0;
int total_pulses = 0;
double distance = 0;
int status = 0;
int second = 0;
int second_sleep = 0;
int change = 0;
double radius = 0.3;
int screen_drawed = 0;
volatile char chronometer_status = 0;
volatile char flag_sleep = 0;
volatile char reseted = 0;
volatile char button_b = 0;
volatile Bool flag_rtt = false;
int screen = 1;
int color_mode = 1;
SemaphoreHandle_t xSemaphore_1;
QueueHandle_t xQueueTouch;
volatile char flag_return = 0;


#define PASS 4
int pass[PASS] = { 1, 5, 9, 0};
int pass_temp[PASS] = {};
int counter = 0;

/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define BUTB_PIO PIOA
#define BUTB_PIO_ID ID_PIOA
#define BUTB_PIO_IDX 11
#define BUTB_PIO_IDX_MASK (1u << BUTB_PIO_IDX)

#define MATH_PI 3.14159265358979323846


/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
  * identify which task has overflowed its stack.
  */
  for (;;) {
  }
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void)
{
  pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/


static void configure_lcd(void){
  /* Initialize display parameter */
  g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
  g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
  g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
  g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

  /* Initialize LCD */
  ili9488_init(&g_ili9488_display_opt);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void draw_screen(int number) {
	if (number == 2) {
	  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
	  ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	} else if (number == 0) {
		 ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		 ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	} else if (number == 1) {
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_RED));
		ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	}
}

void draw_screen_asterisk() {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 430, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}


void draw_screen_sleep(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void draw_button_square(t_but but){
	if (but.status == 0) {
		ili9488_set_foreground_color(COLOR_CONVERT(but.colorOff));
		ili9488_draw_filled_rectangle(but.x-but.width/2, but.y-but.height/2,
		but.x+but.width/2, but.y+but.height/2);
	} else {
		ili9488_set_foreground_color(COLOR_CONVERT(but.colorOn));
		ili9488_draw_filled_rectangle(but.x-but.width/2, but.y-but.height/2,
		but.x+but.width/2, but.y+but.height/2);
	}

	if (but.number == 0) {
		font_draw_text(&calibri_23, "0", but0.x, but0.y, 1);
	} else if (but.number == 1) {
		font_draw_text(&calibri_23, "1", but1.x, but1.y, 1);
	} else if (but.number == 2) {
		font_draw_text(&calibri_23, "2", but2.x, but2.y, 1);
	} else if(but.number == 3) {
		font_draw_text(&calibri_23, "3", but3.x, but3.y, 1);
	} else if(but.number == 4) {
		font_draw_text(&calibri_23, "4", but4.x, but4.y, 1);
	} else if(but.number == 5) {
		font_draw_text(&calibri_23, "5", but5.x, but5.y, 1);
	} else if(but.number == 6) {
		font_draw_text(&calibri_23, "6", but6.x, but6.y, 1);
	} else if(but.number == 7) {
		font_draw_text(&calibri_23, "7", but7.x, but7.y, 1);
	} else if(but.number == 8) {
		font_draw_text(&calibri_23, "8", but8.x, but8.y, 1);
	} else if(but.number == 9) {
		font_draw_text(&calibri_23, "9", but9.x, but9.y, 1);
	} else if(but.number == 10) {
		font_draw_text(&calibri_23, "*", butx.x, butx.y, 1);
	} else if(but.number == 11) {
		font_draw_text(&calibri_23, "Clear", 50, butx.y, 1);
	}

}

void draw_button_image(t_but_image but, int image_number){
	color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(but.x, but.y, but.x + but.width, but.y + but.height);
	if (image_number == 1) {
		if(but.status)
		color_mode ? ili9488_draw_pixmap(but.x, but.y, play_image.width, play_image.height, play_image.data) : ili9488_draw_pixmap(but.x, but.y, play_image_dark.width, play_image_dark.height, play_image_dark.data);
		else
		color_mode ? ili9488_draw_pixmap(but.x, but.y, pause_image.width, pause_image.height, pause_image.data) : ili9488_draw_pixmap(but.x, but.y, pause_image_dark.width, pause_image_dark.height, pause_image_dark.data);
	}
	if(image_number == 0){
		color_mode ? ili9488_draw_pixmap(but.x, but.y, restart_image.width, restart_image.height, restart_image.data) : ili9488_draw_pixmap(but.x, but.y, restart_image_dark.width, restart_image_dark.height, restart_image_dark.data);
	}  
	if(image_number == 2){
		if(but.status)
		ili9488_draw_pixmap(but.x, but.y, dark.width, dark.height, dark.data);
		else
		ili9488_draw_pixmap(but.x, but.y, light.width, light.height, light.data);
	}

}

uint32_t convert_axis_system_x(uint32_t touch_y) {
  // entrada: 4096 - 0 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
  // entrada: 0 - 4096 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_HEIGHT*touch_x/4096;
}




void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
  /* USART tx buffer initialized to 0 */
  uint8_t i = 0; /* Iterator */

  /* Temporary touch event data struct */
  struct mxt_touch_event touch_event;
  
  /* first touch only */
  uint first = 0;

  /* Collect touch events and put the data in a string,
  * maximum 2 events at the time */
  do {

    /* Read next next touch event in the queue, discard if read fails */
    if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
      continue;
    }
    
    /************************************************************************/
    /* Envia dados via fila RTOS                                            */
    /************************************************************************/
    if(first == 0 ){
      *x = convert_axis_system_x(touch_event.y);
      *y = convert_axis_system_y(touch_event.x);
      first = 1;
    }
    
    i++;

    /* Check if there is still messages in the queue and
    * if we have reached the maximum numbers of events */
  } while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}

/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){
  
  struct mxt_device device; /* Device data container */
  mxt_init(&device);       	/* Initialize the mXT touch device */
  touchData touch;          /* touch queue data type*/
  
  while (true) {
    /* Check for any pending messages and run message handler if any
    * message is found in the queue */
    if (mxt_is_message_pending(&device)) {
      mxt_handler(&device, &touch.x, &touch.y);
      xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
      vTaskDelay(200);
      
      // limpa touch
      while (mxt_is_message_pending(&device)){
        mxt_handler(&device, NULL, NULL);
        vTaskDelay(50);
      }
    }
    
    vTaskDelay(300);
  }
}

void RTC_Handler(void){
	uint32_t ul_status = rtc_get_status(RTC);

	if (flag_return) {
		second ++;
	}

	second_sleep ++;

	/* Sec IRQ */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphore_1, &xHigherPriorityTaskWoken);
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}

	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM){
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	}

	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	

}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc, irq_type);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {		
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		flag_rtt = true; 
	}
}


static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

void but_play_callback(t_but_image *b){
	chronometer_status = !chronometer_status;
	b->status = !b->status;
	status = !status;
	draw_button_image(*b, 1);
}


void but_reset_callback(t_but_image *b){
	status = 0;
	total_pulses = 0;
	distance = 0;
	chronometer_status = 0;
	second = 0;
	b->status = 1;
	draw_button_image(*b, 1);
}

void but_color_mode_callback(t_but_image *b){
	color_mode = !color_mode;
	b->status = !b->status;
	screen_drawed = 0;
	draw_button_image(*b, 2);
// 	draw_screen();
}

void button_screen_callback(void){
	if (screen == 1) {
		screen = 2;
		} else {
		screen = 1;
	}
	change = 1;
}

void but_board_callback(void){
	second_sleep = 0;
	if (flag_sleep == 1) {
		flag_sleep = 0;
	} else if (flag_sleep == 2){
		flag_sleep = 3;
	}
	pulses++;
	if(status){
		total_pulses++;
		distance = 2*MATH_PI*radius*total_pulses;
	}

}

int process_touch_image(t_but_image but_play, touchData touch){
	if((touch.x > but_play.x) && touch.x < (but_play.x + play_image.width)){
		if((touch.y > but_play.y) && touch.y < (but_play.y + play_image.height)){
			return 1;
		}
	}
	return 0;
}

int process_touch(t_but button, touchData touch){
	if (button.y - (button.height/2) <= touch.y && button.y + (button.height/2) >= touch.y) {
		if (button.x - (button.width/2) <= touch.x && button.x + (button.width/2) >= touch.x){
			return 1;
		}
	}
	return 0;
}

void change_view_callback(){
// 	draw_screen();
	screen_drawed = 0;
	change = 0;
}

void animate_speed(void){
		color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
		ili9488_draw_filled_rectangle(0, 440, ILI9488_LCD_WIDTH, ILI9488_LCD_HEIGHT);
		color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		int max_width = ILI9488_LCD_WIDTH - 45;
		for(int i = 0; i < pulses/3; i++){
			int width = 45 + (27*i);
			if (width < max_width) ili9488_draw_filled_circle(width, 450, 5);
		}
		
}




void task_lcd(void){
	xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
	configure_lcd();
	draw_screen(0);
	
// 	xSemaphore_1 = xSemaphoreCreateBinary();
// 	int h, m, s;
//   	
// 	calendar rtc_initial = {2020, 4, 27, 4, 17, 10 ,0};
// 	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
// 	
	touchData touch;
// 	
// 	t_but_image but_play = {.width = play_image.width, .height = play_image.height, .border = 2, .status= 1, .callback = but_play_callback,
// 	.x = (ILI9488_LCD_WIDTH/2) + 30, .y = 250 };
// 	
// 	t_but_image but_reset = {.width = restart_image.width, .height = restart_image.height, .border = 2, .status= 1, .callback = but_reset_callback,
// 	.x = (ILI9488_LCD_WIDTH/2) + 30, .y = 265};
// 	
// 	t_but_image but_color_mode = {.width = dark.width, .height= dark.height, .x = 260, .y = 40, .border = 2, .status = 1, .callback= but_color_mode_callback};
// 		
// 	t_but button_screen_1 = {.width = 130, .height = play_image.height - 30,
// 	.x = 100, .y = 370 };
// 	
// 	
// 	t_but button_screen_2 = {.width = 230, .height = play_image.height - 30,
// 	.x = 160, .y = 415 };
// 	
// 	char date[300];
// 	char date_chronometer[300];
// 	
// 	double speeds[2] = {0.0, 0.0};
// 	int speed_index = 0;
// 		

		draw_button_square(but0);
		draw_button_square(but1);
		draw_button_square(but2);
		draw_button_square(but3);
		draw_button_square(but4);
		draw_button_square(but5);
		draw_button_square(but6);
		draw_button_square(but7);
		draw_button_square(but8);
		draw_button_square(but9);
		draw_button_square(butx);
		draw_button_square(butclear);

		// font_draw_text(&calibri_23, "0", but0.x, but0.y, 1);
		// font_draw_text(&calibri_23, "1", but1.x, but1.y, 1);
		// font_draw_text(&calibri_23, "2", but2.x, but2.y, 1);
		// font_draw_text(&calibri_23, "3", but3.x, but3.y, 1);
		// font_draw_text(&calibri_23, "4", but4.x, but4.y, 1);
		// font_draw_text(&calibri_23, "5", but5.x, but5.y, 1);
		// font_draw_text(&calibri_23, "6", but6.x, but6.y, 1);
		// font_draw_text(&calibri_23, "7", but7.x, but7.y, 1);
		// font_draw_text(&calibri_23, "8", but8.x, but8.y, 1);
		// font_draw_text(&calibri_23, "9", but9.x, but9.y, 1);
		// font_draw_text(&calibri_23, "*", butx.x, butx.y, 1);
		
	while (true) {
		
// 		printf("%d",flag_return);
// 		printf("%d" ,second);

		
		if (flag_return == 1) {
			
			vTaskDelay(1000);
			
			draw_screen(0);

			draw_button_square(but0);
			draw_button_square(but1);
			draw_button_square(but2);
			draw_button_square(but3);
			draw_button_square(but4);
			draw_button_square(but5);
			draw_button_square(but6);
			draw_button_square(but7);
			draw_button_square(but8);
			draw_button_square(but9);
			draw_button_square(butx);
			draw_button_square(butclear);

			// font_draw_text(&calibri_23, "0", but0.x, but0.y, 1);
			// font_draw_text(&calibri_23, "1", but1.x, but1.y, 1);
			// font_draw_text(&calibri_23, "2", but2.x, but2.y, 1);
			// font_draw_text(&calibri_23, "3", but3.x, but3.y, 1);
			// font_draw_text(&calibri_23, "4", but4.x, but4.y, 1);
			// font_draw_text(&calibri_23, "5", but5.x, but5.y, 1);
			// font_draw_text(&calibri_23, "6", but6.x, but6.y, 1);
			// font_draw_text(&calibri_23, "7", but7.x, but7.y, 1);
			// font_draw_text(&calibri_23, "8", but8.x, but8.y, 1);
			// font_draw_text(&calibri_23, "9", but9.x, but9.y, 1);
			// font_draw_text(&calibri_23, "*", butx.x, butx.y, 1);

			flag_return = 0;

		}
		
		// printf("%d \n", flag_sleep);
	
		//Queues and Semaphores
		if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
// 			second_sleep = 0;
// 			if (flag_sleep == 1) {
// 				flag_sleep = 0;
// 				} else if (flag_sleep == 2){
// 				flag_sleep = 3;
// 			}
// 			if(process_touch_image(but_play, touch)){
// 				but_play.callback(&but_play);
// 			}
			
			if(process_touch(but0, touch)) {
				but0.status = 1;
				draw_button_square(but0);
				vTaskDelay(100);
				but0.status = 0;
				draw_button_square(but0);
			
				
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 0;
					counter++;
				}
				
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
					} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
					} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
					} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			else if(process_touch(but1, touch)) {
				but1.status = 1;
				draw_button_square(but1);
				vTaskDelay(100);
				but1.status = 0;
				draw_button_square(but1);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 1;
					counter++;
				}
				
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			
			else if(process_touch(but2, touch)) {
				but2.status = 1;
				draw_button_square(but2);
				vTaskDelay(100);
				but2.status = 0;
				draw_button_square(but2);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 2;
					counter++;
				}

				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			
			else if(process_touch(but3, touch)) {
				but3.status = 1;
				draw_button_square(but3);
				vTaskDelay(100);
				but3.status = 0;
				draw_button_square(but3);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 3;
					counter++;
				}
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			else if(process_touch(but4, touch)) {
				but4.status = 1;
				draw_button_square(but4);
				vTaskDelay(100);
				but4.status = 0;
				draw_button_square(but4);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 4;
					counter++;
				}
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			
			else if(process_touch(but5, touch)) {
				but5.status = 1;
				draw_button_square(but5);
				vTaskDelay(100);
				but5.status = 0;
				draw_button_square(but5);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 5;
					counter++;
				}
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			
			else if(process_touch(but6, touch)) {
				but6.status = 1;
				draw_button_square(but6);
				vTaskDelay(100);
				but6.status = 0;
				draw_button_square(but6);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
					} else {
					pass_temp[counter] = 6;
					counter++;
				}
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			
			else if(process_touch(but7, touch)) {

				
				but7.status = 1;
				draw_button_square(but7);
				vTaskDelay(100);
				but7.status = 0;
				draw_button_square(but7);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 7;
					counter++;
				}
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			
			else if(process_touch(but8, touch)) {
				
				but8.status = 1;
				draw_button_square(but8);
				vTaskDelay(100);
				but8.status = 0;
				draw_button_square(but8);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
					} else {
					pass_temp[counter] = 8;
					counter++;
				}
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			
			else if(process_touch(but9, touch)) {
				
				but9.status = 1;
				draw_button_square(but9);
				vTaskDelay(100);
				but9.status = 0;
				draw_button_square(but9);
				if (counter == 4) {
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				} else {
					pass_temp[counter] = 9;
					counter++;
				}
				draw_screen_asterisk();

				if (counter == 1) {
					font_draw_text(&calibri_23, "*", 130, 450, 1);
				} else if (counter == 2) {
					font_draw_text(&calibri_23, "**", 130, 450, 1);
				} else if (counter == 3) {
					font_draw_text(&calibri_23, "***", 130, 450, 1);
				} else if (counter == 4) {
					font_draw_text(&calibri_23, "****", 130, 450, 1);
				}
			}
			else if(process_touch(butclear, touch)) {
							
				butclear.status = 1;
				draw_button_square(butclear);
				vTaskDelay(100);
				butclear.status = 0;
				draw_button_square(butclear);
				for (int i = 0; i < counter; i++) {
					pass_temp[i] = 0;
				}
				counter = 0;
				draw_screen_asterisk();

			}
			else if(process_touch(butx, touch)) {
				if (counter == 4) {
					if (pass_temp[0] == pass[0]) {
						if(pass_temp[1] == pass[1]) {
							if (pass_temp[2] == pass[2]) {
								if (pass_temp[3] == pass[3]){
									draw_screen(2);
									flag_return = 1;
									for (int i = 0; i < counter; i++) {
										pass_temp[i] = 0;
									}
									counter = 0;
								} else {
									draw_screen(1);
									flag_return = 1;
									for (int i = 0; i < counter; i++) {
										pass_temp[i] = 0;
									}
									counter = 0;
									
								}
							} else {
								draw_screen(1);
								flag_return = 1;
								for (int i = 0; i < counter; i++) {
									pass_temp[i] = 0;
								}
								counter = 0;
							}
						} else {
							draw_screen(1);
							flag_return = 1;
							for (int i = 0; i < counter; i++) {
								pass_temp[i] = 0;
							}
							counter = 0;
						}
					} else {
						draw_screen(1);
						flag_return = 1;
						for (int i = 0; i < counter; i++) {
							pass_temp[i] = 0;
						}
						counter = 0;
					}
				} else {
					draw_screen(1);
					flag_return = 1;
					for (int i = 0; i < counter; i++) {
						pass_temp[i] = 0;
					}
					counter = 0;
				}
			}
			
						
// 			if (screen == 1) {
// 				if (process_touch(button_screen_1, touch)) {
// 					button_screen_callback();
// 				}
// 				
// 				if(process_touch_image(but_color_mode, touch)){
// 					but_color_mode.callback(&but_color_mode);
// 				}
// 			} else {				
// 				if(process_touch_image(but_reset, touch)){
// 					but_reset.callback(&but_play);
// 				}
// 					
// 				if (process_touch(button_screen_2, touch)) {
// 					button_screen_callback();
// 				}
// 			}
		}
		
// 		if( xSemaphoreTakeFromISR(xSemaphore_1, ( TickType_t ) 500) == pdTRUE ){
// 			color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
// 			rtc_get_time(RTC, &h, &m, &s);
// 			sprintf(date, "%2d:%2d:%2d", h, m, s);
// 			if (flag_sleep != 2) {
// 				ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH, 40);
// 				color_mode ? font_draw_text(&calibri_23, date, 110, 0, 1) : font_draw_text(&calibri_23_dark, date, 110, 0, 1);
// 			}
// 
// 			int second_temp = second;
// 			int minute_temp = 0;
// 			int hour_temp = 0;
// 
// 			if (second_temp/3600 > 0) {
// 				hour_temp = second_temp/3600;
// 				second_temp = second_temp - (hour_temp * 3600);
// 			}
// 			if (second_temp/60 > 0) {
// 				minute_temp = second_temp/60;
// 				second_temp = second_temp - (minute_temp * 60);
// 			}
// 				
// 				
// 			// sprintf(date_chronometer, "%2d:%2d:%2d", hour_temp, minute_temp, second_temp);
// 			// if(screen == 1 && flag_sleep != 2){
// 			// 	//Elapsed time screen 1
// 			// 	color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
//  			// 	ili9488_draw_filled_rectangle(180, 260, ILI9488_LCD_WIDTH, 290);
// 			// 	color_mode ? font_draw_text(&calibri_23, date_chronometer, 210, 260, 1) : font_draw_text(&calibri_23_dark, date_chronometer, 210, 260, 1);
// 			// } else if (flag_sleep != 2) {
// 			// 	//Elapsed time screen 2
// 			// 	color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
// 			// 	ili9488_draw_filled_rectangle(190, 145, ILI9488_LCD_WIDTH, 175);
// 			// 	color_mode ? font_draw_text(&calibri_23, date_chronometer, 190, 155, 1) : font_draw_text(&calibri_23_dark, date_chronometer, 190, 155, 1);
// 			// }
// 		}
		
		//Screens
// 		if(screen == 1) {
// 			if (change) {
// 				change_view_callback();
// 			}
// 			
// 			if(!screen_drawed || flag_sleep == 3){
// // 				if (flag_sleep == 3) {
// // 					draw_screen();
// // 				}
// 
// 				draw_button_image(but_color_mode, 2);
// 				draw_button_square(button_screen_1);



				//Play or Pause button
// 				but_play.x = (ILI9488_LCD_WIDTH/2) + 30;
// 				but_play.y = 330;
// 				draw_button_image(but_play, 1);
// 				
// 				color_mode ? font_draw_text(&calibri_36, "Speed", 110, 50, 1) : font_draw_text(&calibri_36_dark, "Speed", 110, 50, 1);
// 				color_mode ? font_draw_text(&calibri_36, "Acceleration", 70, 170, 1) : font_draw_text(&calibri_36_dark, "Acceleration", 70, 170, 1);
// 				color_mode ? font_draw_text(&calibri_23, "Elapsed Time: ", 40, 260, 1) : font_draw_text(&calibri_23_dark, "Elapsed Time: ", 40, 260, 1);
// 				font_draw_text(&calibri_23_button, "Routes", 65, 355, 1);
// 				screen_drawed = 1;
// 				flag_sleep = 0;
// 			}
			
			
// 			if(flag_rtt && flag_sleep == 0){
// 				printf("%d\n", pulses);
// 				double speed, acceleration, distance;
// 				char speed_string[10];
// 				char signal[2];
// 				
// 				//Speed
// 				speed = (((2*MATH_PI*pulses)/1)*radius)/3.6;
// 				speeds[speed_index] = speed;
// 				speed_index = !speed_index;
// 				color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
// 				ili9488_draw_filled_rectangle(0, 100, ILI9488_LCD_WIDTH, 150);
// 				sprintf(speed_string, "%.2f km/h", speed); 
// 				color_mode ? font_draw_text(&calibri_36, speed_string, 85, 100, 1) : font_draw_text(&calibri_36_dark, speed_string, 85, 100, 1) ;
// 				animate_speed();
// 				
// 				//Acceleration
// 				acceleration = (speeds[1] - speeds[0])/1;
// 				acceleration > 0 ? sprintf(signal, "++") : sprintf(signal, "--");
//  				color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
//  				ili9488_draw_filled_rectangle(140, 220, ILI9488_LCD_WIDTH, 260);
// 				color_mode ? font_draw_text(&calibri_36, signal, 150, 210, 1) : font_draw_text(&calibri_36_dark, signal, 150, 210, 1);
// 				
// 				uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
// 				uint32_t irqRTTvalue = 4;
// 				RTT_init(pllPreScale, irqRTTvalue);
// 				pulses = 0;
// 				flag_rtt = false;
// 			}
// 		}

// 		if (screen == 2) {
// 			if (change) {
// 				change_view_callback();
// 			}
				
// 			if(!screen_drawed ||  flag_sleep == 3){		
// 				if (flag_sleep == 3) {
// // 					draw_screen();
// 				}		
// 				color_mode ? font_draw_text(&calibri_23, "Average speed:", 25, 55, 1) : font_draw_text(&calibri_23_dark, "Average speed:", 25, 55, 1);
// 				color_mode ? font_draw_text(&calibri_23, "Distance:", 25, 105, 1) : font_draw_text(&calibri_23_dark, "Distance:", 25, 105, 1);
// 				color_mode ? font_draw_text(&calibri_23, "Elapsed Time:", 25, 155, 1) : font_draw_text(&calibri_23_dark, "Elapsed Time:", 25, 155, 1);
// 				color_mode ? font_draw_text(&calibri_23, "Status:", 25, 205, 1) : font_draw_text(&calibri_23_dark, "Status:", 25, 205, 1);
// 				draw_button_square(button_screen_2);
// 				font_draw_text(&calibri_23_button, "Home", 130, 400, 1);
// 				but_play.x = 40;
// 				but_play.y = 260;
// 				draw_button_image(but_play, 1);
// 				draw_button_image(but_reset, 0);
// 				screen_drawed = 1;
// 				flag_sleep = 0;
// 			}
			
			
			// if(flag_rtt && flag_sleep == 0){
			// 	double speed;
			// 	char status_string[10];
			// 	char average_speed_string[10];
			// 	char distance_string[10];
				
			// 	//Average speed
			// 	color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
			// 	ili9488_draw_filled_rectangle(190, 40, ILI9488_LCD_WIDTH, 75);
			// 	speed = distance/second;
				
			// 	//Check if speed is nan
			// 	(speed != speed)? sprintf(average_speed_string, "0.00 km/h") : sprintf(average_speed_string, "%.2f km/h", speed/3.6);
				
			// 	color_mode ? font_draw_text(&calibri_23, average_speed_string, 190, 55, 1) : font_draw_text(&calibri_23_dark, average_speed_string, 190, 55, 1);
					
			// 	//Distance
			// 	color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
			// 	ili9488_draw_filled_rectangle(190, 95, ILI9488_LCD_WIDTH, 125);
			// 	sprintf(distance_string, "%.2f km", distance/1000);
			// 	color_mode ? font_draw_text(&calibri_23, distance_string, 190, 105, 1) : font_draw_text(&calibri_23_dark, distance_string, 190, 105, 1);
					
			// 	//Status
			// 	color_mode ? ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE)) : ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
			// 	ili9488_draw_filled_rectangle(190, 195, ILI9488_LCD_WIDTH, 225);
			// 	status ? sprintf(status_string, "On the way") : sprintf(status_string, "Stopped");
			// 	color_mode ? font_draw_text(&calibri_23, status_string, 190, 205, 1) : font_draw_text(&calibri_23_dark, status_string, 190, 205, 1);
					
			// 	uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
			// 	uint32_t irqRTTvalue = 4;
			// 	RTT_init(pllPreScale, irqRTTvalue);
			// 	pulses = 0;
			// 	flag_rtt = false;
			// }
// 		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
  /* Initialize the USART configuration struct */
  const usart_serial_options_t usart_serial_options = {
    .baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
    .charlength   = USART_SERIAL_CHAR_LENGTH,
    .paritytype   = USART_SERIAL_PARITY,
    .stopbits     = USART_SERIAL_STOP_BIT
  };

  sysclk_init(); /* Initialize system clocks */
  board_init();  /* Initialize board */
  
  /* Initialize stdio on USART */
  stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
  
  pmc_enable_periph_clk(BUTB_PIO_ID);
  pio_set_input(BUTB_PIO, BUTB_PIO_IDX_MASK, PIO_PULLUP);
  
  
  pio_configure(BUTB_PIO, PIO_INPUT, BUTB_PIO_IDX_MASK, PIO_PULLUP);
  pio_handler_set(BUTB_PIO,
  BUTB_PIO_ID,
  BUTB_PIO_IDX_MASK,
  PIO_IT_FALL_EDGE,
  but_board_callback);
  
  pio_enable_interrupt(BUTB_PIO, BUTB_PIO_IDX_MASK);
  
  NVIC_EnableIRQ(BUTB_PIO_ID);
  NVIC_SetPriority(BUTB_PIO_ID, 7);
  
  flag_rtt = true;
  
  /* Create task to handler touch */
  if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  
  /* Create task to handler LCD */
  if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  
  /* Start the scheduler. */
  vTaskStartScheduler();

  while(1){

  }


  return 0;
}
