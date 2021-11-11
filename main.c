/* Includes		       */
#include <stdint.h>
#include "sysregs.h"
#include "gic-v2.h"
#include "string.h"
#include "serial.h"
#include "printf-stdarg.h"
#include "gpio.h"

/* Scheduler include files.   */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/*____________________________*/

/* Defines  		      */

#define TIMER_IRQ 27  //virtuller Timer IRQ
#define GPIO_IRQ 145
#define BEATS_PER_SEC configTICK_RATE_HZ
#define ARM_SLEEP asm volatile("wfi" : : : "memory")

#define UART_LOCK xSemaphoreTake(uart_mutex, portMAX_DELAY)
#define UART_UNLOCK xSemaphoreGive(uart_mutex)
#define UART_OUTPUT(args...) do { if(pdPASS == UART_LOCK) { printf(args); UART_UNLOCK;} } while(0)
/*___________________________*/
/* Prototypes                */
void FreeRTOS_Tick_Handler( void );
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationIRQHandler( unsigned ulICCIAR );
void __div0(void);
int printf(const char *format, ...);

/* Global variables              */
static TaskHandle_t uart_task_handle;
static TaskHandle_t gpio_task_handle;
static SemaphoreHandle_t uart_mutex;
sio_fd_t ser_dev;
char rx[80];

/*_____________________________*/

/* FreeRTOS debug hooks       */

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
  volatile unsigned long ul = 0;

  ( void ) pcFile;
  ( void ) ulLine;

  vTaskSuspendAll();
  taskENTER_CRITICAL();
  {
    /* Set ul to a non-zero value using the debugger to step out of this
       function. */
    printf("%s %s: line=%lu\n", __func__, pcFile, ulLine);
    while( ul == 0 ) {
      portNOP();
    }
  }
  taskEXIT_CRITICAL();
}

void vApplicationMallocFailedHook( void )
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
     free memory available in the FreeRTOS heap.  pvPortMalloc() is called
     internally by FreeRTOS API functions that create tasks, queues, software
     timers, and semaphores.  The size of the FreeRTOS heap is set by the
     configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  taskDISABLE_INTERRUPTS();
  printf("%s\n", __func__);
  while(1) {
    portNOP();
  }
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  vTaskSuspendAll();
  taskDISABLE_INTERRUPTS();
    printf("%s task=%s\n", __func__, pcTaskName);
  for( ;; )
    ARM_SLEEP;
}

void __div0(void)
{
  printf("PANIC: Div by zero error\n");
  ARM_SLEEP;
}

/*___________________________*/

/* Failure prevent methodes */

void resetFreeRTOS(void){
 //Fail after new Start the Cell
 //reset all values on default
 //eg. Variable for running Scheduler
}

/*__________________________*/

/* Timer control            */
static int32_t timer_value_for_period;
static unsigned timer_frq;

static inline void timer_on(void)
{
	arm_write_sysreg(CNTV_CTL_EL0, 1); //virtueller Timer enable EL0
}

#if 0
static inline void timer_off(void)
{
	arm_write_sysreg(CNTV_CTL_EL0, 0);
}

static u64 get_actual_ticks(void)
{
  u64 pct64;
  arm_read_sysreg(CNTVCT, pct64);
  return pct64;
}

static inline unsigned ticks_to_ns(unsigned ticks)
{
  return (ticks*1000) / ( timer_frq/1000/1000);
}
#endif

static inline void timer_set(int32_t val)
{
	arm_write_sysreg(CNTV_TVAL_EL0, val);  //virtueller Timer neuer Wert setzten
}

static inline void timer_set_next_event(void)
{
  int32_t time_drift;
  /* The timer indicates an overtime with a negative value inside this register */
  arm_read_sysreg(CNTV_TVAL_EL0, time_drift);
  /* If the drift is greater than timer_value_for_period we have lost a time period */
  //configASSERT(-time_drift < timer_value_for_period);
  /* Correct next period by this time drift. The drift is caused by the software */
	timer_set(timer_value_for_period + time_drift);
}

/* Function called by FreeRTOS_Tick_Handler as last action */
void vClearTickInterrupt(void)
{
  timer_set_next_event();
	//timer_on();
}

int timer_init(unsigned beats_per_second)
{
	timer_value_for_period = timer_frq / beats_per_second;
	timer_set(timer_value_for_period);
  timer_on();

	return 0;
}
/*___________________________*/

/* UART handling            */
static void serial_print(char *buf, int n)
{
  buf[n] = 0;
  UART_OUTPUT("Print: \t%d %s\n", n, buf);
}
/*_________________________*/

/* Interrupt Handling	     */

void vConfigureTickInterrupt( void )
{
  /* Register the standard FreeRTOS Cortex-A tick handler as the timer's
     interrupt handler.  The handler clears the interrupt using the
     configCLEAR_TICK_INTERRUPT() macro, which is defined in FreeRTOSConfig.h. */
  gic_v2_irq_set_prio(TIMER_IRQ, portLOWEST_USABLE_INTERRUPT_PRIORITY);
  gic_v2_irq_enable(TIMER_IRQ);
  timer_init(BEATS_PER_SEC);
}

static void handle_gpio_irq()
{
  BaseType_t do_yield = pdFALSE;
  xTaskNotifyFromISR(gpio_task_handle, NULL, eSetValueWithOverwrite, &do_yield); 
  portYIELD_FROM_ISR(do_yield);
}


static void handle_uart_irq(void)
{
  uint32_t v = serial_irq_getchar(ser_dev);
  BaseType_t do_yield = pdFALSE;
  xTaskNotifyFromISR(uart_task_handle, v, eSetValueWithOverwrite, &do_yield); 
  portYIELD_FROM_ISR(do_yield);
}

void vApplicationIRQHandler(unsigned int irqn)
{
  switch(irqn) {
    case TIMER_IRQ:
      //timer_off();
      FreeRTOS_Tick_Handler();
      break;
    case UART_IRQ:
      handle_uart_irq();
      //printf("Uart Interrupt \n");
      break;
    case GPIO_IRQ:
       handle_gpio_irq();
       gpio_clearIRQStatus(21);
      break;
    default:
      printf("Spurious irq %d\n", irqn);
      break;
  }
}

/*____________________________*/



/* FreeRTOS application tasks */

//Task print the Output form the buffer
void uartTask_print(){
 while(1){
  char s[]="Output:\n";
  int idx = sizeof(s);
  serial_print(s, idx);
  if(rx[0]!='\0'){
    int i =sizeof(rx);
    serial_print(rx,i);
    rx[0]='\0';
  }
  idx = 0;
  vTaskDelay(1000/ portTICK_PERIOD_MS); 
 } 
}
/*
 void read_uartTask(){
   uint32_t help = -1;
   while(1){
     for(int idx=0; idx <= 79; idx++){
     help = mini_uart_getchar();
      if(help == 0x0000)idx =79;
     rx[idx]=(char)help;
   }
      
    vTaskDelay(10000/ portTICK_PERIOD_MS);
   }
 }*/


//Task read the Uart inpput via a Interrupt and print it on the console
 static void uartTask(void *pvParameters)
{
  while(1){
  uint32_t c;
  char s[80];
  int idx = 0;
  while(pdTRUE) {
    if(pdTRUE == xTaskNotifyWait(0, 0, &c, pdMS_TO_TICKS(250))) {
      
      s[idx] = (char)c;
      if('\r' == s[idx] || idx >= sizeof(s)-1) {
        serial_print(s, idx);
        idx = 0;
      }
      else
        ++idx;
    }
    else if(idx) { /*Buffer not empty */
      serial_print(s, idx);
      idx = 0;
    }
  }
  }
}


static uint32_t mmio_read32(void *addr)
{
  return *((volatile uint32_t*)addr);
}

//Task print the status from the gpio pin
void gpioTask(void *pvParameters)
{
 while(1){
   if(pdTRUE == xTaskNotifyWait(0, 0, 0, pdMS_TO_TICKS(250))) {
  int input = gpio_input(21); //vllt inpout in irq reinmachen, damit man den save hat (für beispiel unwichitg)
  printf("Input GPIO: %x \n",input);
  printf("Status GPIO flanke : %x \n",mmio_read32((void*)0xfe200000 + 0x40));
  printf("Status GPIO flanke clear : %x \n",mmio_read32((void*)0xfe200000 + 0x40));
  vTaskDelay(500/portTICK_PERIOD_MS);
 }
 }
}


/*____________________________*/

/* Hardware init */
static void hardware_init(void){
static unsigned long io_dev_map[2];
 
io_dev_map[1] = (unsigned long)gic_v2_init();
vPortInstallFreeRTOSVectorTable();

//Mini Uart enable
serial_irq_rx_enable(UART_IRQ);
gic_v2_irq_enable(UART_IRQ);
 
//GPIO enable 21 Resitor Pull up/Input
gpio_pin_input_enable(21);
gpio_resistor(21,1);
gpio_pin_irq_rx_enable(21);
gic_v2_irq_enable(GPIO_IRQ);
gpio_clearIRQStatus(21);

arm_read_sysreg(CNTFRQ, timer_frq);
if(!timer_frq) {
  printf("Timer frequency is zero\n");
  ARM_SLEEP;
 }
}

/*___________________________ */

/*  main */
void inmate_main(void)
{
 resetFreeRTOS();
 hardware_init();
 uart_mutex = xSemaphoreCreateMutex();

 /*xTaskCreate(
   read_uartTask,
  "UART read",
  configMINIMAL_STACK_SIZE,
  NULL,
  configMAX_PRIORITIES-1,
  &uart_task_handle);
*/
 xTaskCreate(
	 uartTask_print,
	 "UART print",
	 configMINIMAL_STACK_SIZE,
	 NULL,
	 configMAX_PRIORITIES-2,  
   NULL );

  xTaskCreate(
    gpioTask,
    "GPIO get Input",
    configMINIMAL_STACK_SIZE,
	  NULL,
    configMAX_PRIORITIES-1,  
    &gpio_task_handle);

  xTaskCreate( uartTask, /* The function that implements the task. */
    "uartstat", /* The text name assigned to the task - for debug only; not used by the kernel. */
    configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
    NULL,                                                            /* The parameter passed to the task */
    configMAX_PRIORITIES-1, /* The priority assigned to the task. */
    &uart_task_handle );

  

 printf("vTaskStartScheduler goes active\n");
 vTaskStartScheduler();
 printf("vTaskStartScheduler terminated: strange!!!\n");
	while (1) {
    ARM_SLEEP;
  }


}
