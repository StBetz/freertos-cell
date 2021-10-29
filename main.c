/* Includes		       */
#include <stdint.h>
#include "sysregs.h"
#include "gic-v2.h"
#include "string.h"
#include "serial.h"
#include "printf-stdarg.h"

/* Scheduler include files.   */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/*____________________________*/

/* Defines  		      */

#define TIMER_IRQ 27  //virtuller Timer IRQ
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

/* {{{1 Global variables */
static TaskHandle_t uart_task_handle;
static SemaphoreHandle_t uart_mutex;
sio_fd_t ser_dev;
/* }}} */

/* {{{1 FreeRTOS debug hooks */

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

/* }}} */

/* {{{1 Timer control */
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
/* }}} */

/* {{{1 UART handling */
static void serial_print(char *buf, int n)
{
  buf[n] = 0;
  UART_OUTPUT("TUA\t%d %s\n", n, buf);
}

/*static __attribute__((unused)) void hyp_putchar(int c)
{
  asm volatile(
      "mov r0, #8;"
      "mov r1, %0;"
      "hvc #0x0;"
      :  outputs 
      : "r" (c)  inputs 
      : "r0", "r1" clobbered 
      );
}*/



/* }}} */

/* {{{1 Interrupt handling */

void vConfigureTickInterrupt( void )
{
  /* Register the standard FreeRTOS Cortex-A tick handler as the timer's
     interrupt handler.  The handler clears the interrupt using the
     configCLEAR_TICK_INTERRUPT() macro, which is defined in FreeRTOSConfig.h. */
  gic_v2_irq_set_prio(TIMER_IRQ, portLOWEST_USABLE_INTERRUPT_PRIORITY);
  gic_v2_irq_enable(TIMER_IRQ);
  timer_init(BEATS_PER_SEC);
}


/*static void handle_uart_irq(void)
{
  uint32_t v = serial_irq_getchar(ser_dev);
  BaseType_t do_yield = pdFALSE;
  xTaskNotifyFromISR(uart_task_handle, v, eSetValueWithOverwrite, &do_yield);
  portYIELD_FROM_ISR(do_yield);
}*/

void vApplicationIRQHandler(unsigned int irqn)
{
  switch(irqn) {
    case TIMER_IRQ:
      //timer_off();
      FreeRTOS_Tick_Handler();
      break;
    case UART_IRQ:
      //handle_uart_irq();
      break;
    case 0x3ff:
      /* This irq should be ignored. It is no longer relevant */
      break;
    default:
      printf("Spurious irq %d\n", irqn);
      break;
  }
}

/* }}} */

/*____________________________*/

/*____________________________*/

/* Interrupt Handling	      */

/*____________________________*/



/* FreeRTOS application tasks */

void uartTask(){
 while(1){
  //char s[80]="Geht\n";
  //int idx = 79;
  mini_uart_putchar((uint32_t)'a');
  //serial_print(s, idx);
  //idx = 0;
  vTaskDelay(1000/ portTICK_PERIOD_MS); 
 } 
}
/*____________________________*/

/* Hardware init */
static void hardware_init(void){
  static unsigned long io_dev_map[2];
 
 //ser_dev = serial_open();
 // io_dev_map[0] = (unsigned long)ser_dev;
 io_dev_map[1] = (unsigned long)gic_v2_init();
  vPortInstallFreeRTOSVectorTable();
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
 hardware_init();
 uart_mutex = xSemaphoreCreateMutex();
//hyp_putchar('a');

 xTaskCreate(
	uartTask,
	"UART print",
	configMINIMAL_STACK_SIZE,
	NULL,
	configMAX_PRIORITIES-1, /* The priority assigned to the task. */
  &uart_task_handle );



 printf("vTaskStartScheduler goes active\n");
 vTaskStartScheduler();



}
