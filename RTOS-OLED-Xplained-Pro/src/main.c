#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

/* Botao da placa */
#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

//OLED
#define LED_PIO_1           PIOA
#define LED_PIO_ID_1        ID_PIOA
#define LED_PIO_IDX_1       0
#define LED_PIO_IDX_MASK_1  (1u << LED_PIO_IDX_1)

#define BUT_PIO_1			PIOD
#define BUT_PIO_ID_1		ID_PIOD
#define BUT_PIO_IDX_1		28
#define BUT_PIO_IDX_MASK_1 (1u << BUT_PIO_IDX_1)

#define LED_PIO_2           PIOC
#define LED_PIO_ID_2        ID_PIOC
#define LED_PIO_IDX_2       30
#define LED_PIO_IDX_MASK_2  (1u << LED_PIO_IDX_2)

#define BUT_PIO_2			PIOC
#define BUT_PIO_ID_2		ID_PIOC
#define BUT_PIO_IDX_2		31
#define BUT_PIO_IDX_MASK_2 (1u << BUT_PIO_IDX_2)

#define LED_PIO_3           PIOB
#define LED_PIO_ID_3        ID_PIOB
#define LED_PIO_IDX_3       2
#define LED_PIO_IDX_MASK_3  (1u << LED_PIO_IDX_3)

#define BUT_PIO_3			PIOA
#define BUT_PIO_ID_3		ID_PIOA
#define BUT_PIO_IDX_3		19
#define BUT_PIO_IDX_MASK_3 (1u << BUT_PIO_IDX_3)

SemaphoreHandle_t xSemaphoreRTT;
SemaphoreHandle_t xSemaphoreBtn;
SemaphoreHandle_t xSemaphoreSec;
SemaphoreHandle_t xSemaphoreAlm;

/** RTOS  */
#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);

void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow \n");
  for (;;) {
  }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreBtn, &xHigherPriorityTaskWoken);
}
	
void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_PIO_1, LED_PIO_IDX_MASK_1);  
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
	}
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xSemaphoreGiveFromISR(xSemaphoreSec, &xHigherPriorityTaskWoken);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o código para irq de alame vem aqui
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xSemaphoreGiveFromISR(xSemaphoreAlm, &xHigherPriorityTaskWoken);
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  gfx_mono_ssd1306_init();
  //gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
  //gfx_mono_draw_string("oii", 0, 20, &sysfont);

	TC_init(TC0, ID_TC1, 1, 2);
	tc_start(TC0, 1);
	
	RTT_init(400, 100, RTT_MR_ALMIEN);
	
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	
  for (;;) {
	  
	  if (xSemaphoreTake(xSemaphoreRTT, 1)){
		  pin_toggle(LED_PIO_2, LED_PIO_IDX_MASK_2);
		  RTT_init(400, 100, RTT_MR_ALMIEN);
	  }
	  
	  if (xSemaphoreTake(xSemaphoreBtn, 1)){
		 uint32_t current_hour, current_min, current_sec;
		 rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
		 
		 rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 4);
	  }
	  
	  if (xSemaphoreTake(xSemaphoreSec, 1)){
		  int current_hour, current_min, current_sec;
		  rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
		  
		  char hour[8];
		  sprintf(hour,"%d",current_hour);
		  
		  char min[8];
		  sprintf(min,"%d",current_min);
		  
		  char sec[8];
		  sprintf(sec,"%d",current_sec);
		  
		  char tempo[128];
		  sprintf(tempo,"%s:%s:%s",hour,min,sec);
		  gfx_mono_draw_string(tempo, 0, 0, &sysfont);
	  }
	  
	  if (xSemaphoreTake(xSemaphoreAlm, 1)){
		  pio_clear(LED_PIO_3, LED_PIO_IDX_MASK_3);
		  delay_ms(200);
		  pio_set(LED_PIO_3, LED_PIO_IDX_MASK_3);
		  delay_ms(200);
	  } 
  }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
  const usart_serial_options_t uart_serial_options = {
      .baudrate = CONF_UART_BAUDRATE,
      .charlength = CONF_UART_CHAR_LENGTH,
      .paritytype = CONF_UART_PARITY,
      .stopbits = CONF_UART_STOP_BITS,
  };

  /* Configure console UART. */
  stdio_serial_init(CONF_UART, &uart_serial_options);

  /* Specify that stdout should not be buffered. */
  setbuf(stdout, NULL);
}

static void BUT_init(void) {

  pmc_enable_periph_clk(BUT_PIO_ID_1);

  pio_configure(BUT_PIO_1, PIO_INPUT, BUT_PIO_IDX_MASK_1, PIO_PULLUP | PIO_DEBOUNCE);
  pio_set_debounce_filter(BUT_PIO_1, BUT_PIO_IDX_MASK_1, 60);
  
  pio_handler_set(BUT_PIO_1,
  BUT_PIO_ID_1,
  BUT_PIO_IDX_MASK_1,
  PIO_IT_FALL_EDGE,
  but_callback);

  pio_enable_interrupt(BUT_PIO_1, BUT_PIO_IDX_MASK_1);
  pio_get_interrupt_status(BUT_PIO_1);

  NVIC_EnableIRQ(BUT_PIO_ID_1);
  NVIC_SetPriority(BUT_PIO_ID_1, 4);
}

void LED_init(int estado) {
	pmc_enable_periph_clk(LED_PIO_ID_1);
	pio_set_output(LED_PIO_1, LED_PIO_IDX_MASK_1, estado, 0, 0);
	
	pmc_enable_periph_clk(LED_PIO_ID_2);
	pio_set_output(LED_PIO_2, LED_PIO_IDX_MASK_2, estado, 0, 0);
	
	pmc_enable_periph_clk(LED_PIO_ID_3);
	pio_set_output(LED_PIO_3, LED_PIO_IDX_MASK_3, estado, 0, 0);
};

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	
	/** ATIVA PMC PCK6 TIMER_CLOCK1  */
	if(ul_tcclks == 0 )
	pmc_enable_pck(PMC_PCK_6);
	
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
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
/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
  /* Initialize the SAM system */
  sysclk_init();
  board_init();
  
  LED_init(1);
  BUT_init();

  /* Initialize the console uart */
  configure_console();
  
  xSemaphoreRTT = xSemaphoreCreateBinary();
  if (xSemaphoreRTT == NULL)
  printf("falha em criar o semaforo \n");
  
  xSemaphoreBtn = xSemaphoreCreateBinary();
  if (xSemaphoreBtn == NULL)
  printf("falha em criar o semaforo \n");
  
  xSemaphoreSec = xSemaphoreCreateBinary();
  if (xSemaphoreSec == NULL)
  printf("falha em criar o semaforo \n");
  
  xSemaphoreAlm = xSemaphoreCreateBinary();
  if (xSemaphoreAlm == NULL)
  printf("falha em criar o semaforo \n");

  /* Create task to control oled */
  if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL,
                  TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create oled task\r\n");
  }

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
  while (1) {
  }

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}
