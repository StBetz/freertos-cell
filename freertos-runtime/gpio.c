
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gpio.h"


#define GPIO_BASE 0xfe200000

#define GPFSEL0 0x00 //GPIO Function Select 0
#define GPFSEL1 0x04 //GPIO Function Select 1
#define GPFSEL2 0x08 //GPIO Function Select 2
#define GPFSEL3 0x0c //GPIO Function Select 3
#define GPFSEL4 0x10 //GPIO Function Select 4
#define GPFSEL5 0x14 //GPIO Function Select 5
#define register_size 0x04

#define GPSET0  0x1c //GPIO Pin Output Set 0
#define GPSET1  0x20 //GPIO Pin Output Set 1
#define GPCLR0  0x28 //GPIO Pin Output Clear 0
#define GPCLR1  0x2c //GPIO Pin Output Clear 1
#define GPLEV0  0x34 //GPIO Pin Level 0
#define GPLEV1  0x38 //GPIO Pin Level 1

#define GPEDS0 0x40 //GPIO Pin Event Detect Status 0
#define GPEDS1 0x44  //GPIO Pin Event Detect Status 1
#define GPREN0 0x4c  //GPIO Pin Rising Edge Detect Enable 0
#define GPREN1 0x50  //GPIO Pin Rising Edge Detect Enable 1
#define GPFEN0 0x58  //GPIO Pin Falling Edge Detect Enable 0
#define GPFEN1 0x5c  //GPIO Pin Falling Edge Detect Enable 1

#define GPHEN0 0x64  //GPIO Pin High Detect Enable 0
#define GPHEN1 0x68  //GPIO Pin High Detect Enable 1
#define GPLEN0 0x70  //GPIO Pin Low Detect Enable 0
#define GPLEN1 0x74  //GPIO Pin Low Detect Enable 1

#define GPAREN0 0x7c  //GPIO Pin Async. Rising Edge Detect 0
#define GPAREN1 0x80  //GPIO Pin Async. Rising Edge Detect 1
#define GPAFEN0 0x88  //GPIO Pin Async. Falling Edge Detect 0
#define GPAFEN1 0x8c  //GPIO Pin Async. Falling Edge Detect 1

#define GPIO_PUP_PDN_CNTRL_REG0 0xe4  //GPIO Pull-up / Pull-down Register 0
#define GPIO_PUP_PDN_CNTRL_REG1 0xe8  //GPIO Pull-up / Pull-down Register 1
#define GPIO_PUP_PDN_CNTRL_REG2 0xec  //GPIO Pull-up / Pull-down Register 2
#define GPIO_PUP_PDN_CNTRL_REG3 0xf0  //GPIO Pull-up / Pull-down Register 3

static uint32_t mmio_read32(void *addr)
{
  return *((volatile uint32_t*)addr);
}

static void mmio_write32(void *addr, uint32_t val)
{
  *((volatile uint32_t*)addr) = val;
}


/*GPIO number on output*/
int gpio_pin_output_enable(uint32_t number){
 uint32_t reg,offset,mask;
 uint32_t help_reg;
 sio_fd_t gpio_base = (void*)GPIO_BASE;
 reg=number/10;
 offset=number%10;
 mask= 7 << 3*offset;
 help_reg=mmio_read32(gpio_base + reg*register_size);
 help_reg =((help_reg & ~mask )| (1 << 3*offset & mask)); 
 mmio_write32(gpio_base+reg*register_size, help_reg );
 return NULL;                                               //Fehlerabfangen möglich,vllt falls bereits in ander enutzung oder ähnliches
}

/*GPIO number on input*/
int gpio_pin_input_enable(uint32_t number){
uint32_t reg,offset,mask;
 uint32_t help_reg;
 sio_fd_t gpio_base = (void*)GPIO_BASE;
 reg=number/10;
 offset=number%10;
 mask= 7 << 3*offset;
 help_reg=mmio_read32(gpio_base + reg*register_size);
 help_reg =(help_reg & ~mask ); 
 mmio_write32(gpio_base+reg*register_size, help_reg );
 printf("Input enable: %x", number);
 return NULL;
}

/* Set GpioPins to Input,that is the reset status  */
int gpio_pin_disable(uint32_t number){
uint32_t reg,offset,mask;
 uint32_t help_reg;
 sio_fd_t gpio_base = (void*)GPIO_BASE;
 reg=number/10;
 offset=number%10;
 mask= 7 << 3*offset;
 help_reg=mmio_read32(gpio_base + reg*register_size);
 help_reg =(help_reg & ~mask  ); 
 mmio_write32(gpio_base+reg*register_size, help_reg );
 return NULL;
}

/* falling and rising edges detect activate*/
void gpio_pin_irq_rx_enable(uint32_t number){ 
 uint32_t offset,reg;
 sio_fd_t gpio_base = (void*)GPIO_BASE;

 offset = number%32;
 reg = number/32;  
 mmio_write32(gpio_base + GPREN0 + reg*register_size, mmio_read32(gpio_base + GPREN0 + reg*register_size) | 1 << offset );
 mmio_write32(gpio_base + GPFEN0 + reg*register_size, mmio_read32(gpio_base + GPFEN0 + reg*register_size) | 1 << offset );
}

/*disable edge event   */
void gpio_pin_irq_rx_disanable(uint32_t number){
 uint32_t offset,reg;
 sio_fd_t gpio_base = (void*)GPIO_BASE;

 offset = number%32;
 reg = number/32;  
 mmio_write32(gpio_base + GPREN0 + reg*register_size, mmio_read32(gpio_base + GPREN0 + reg*register_size) & ~(1 << offset) );
 mmio_write32(gpio_base + GPFEN0 + reg*register_size, mmio_read32(gpio_base + GPFEN0 + reg*register_size) & ~(1 << offset) );
}


/*The state variable can be 0 or anything else. 0 => Pinoutput = 0
                                                other => 1 */
void gpio_output(uint32_t number, uint32_t state){
  uint32_t offset,reg;
  sio_fd_t gpio_base = (void*)GPIO_BASE;

  offset = number%32;
  reg = number/32;  
  if(state){
    mmio_write32(gpio_base + GPSET0 + reg*register_size, mmio_read32(gpio_base + GPSET0 + reg*register_size) | 1 << offset); 
  }else{
    mmio_write32(gpio_base + GPCLR0 + reg*register_size, mmio_read32(gpio_base + GPSET0 + reg*register_size) | 1 << offset);
  }

}

/*Get the GPIO INPUT*/
int gpio_input(uint32_t number){
 uint32_t state,offset,reg;
 sio_fd_t gpio_base = (void*)GPIO_BASE;
 offset = number%32;
 reg = number/32; 
 state = (mmio_read32(gpio_base + GPLEV0 + reg*register_size) & 1<< offset);
 if(state)state=1;
 return state;
}

int bit_count(uint32_t reg){
  int count=0;
 while(reg){
  if(0x00000001 & reg){
    count++;
  }
  reg = reg >> 1;
 }
 return count;
}



/* Find the right Gpio from abank and return the number from the Gpio pin   */
int gpio_irq_input_number(uint32_t bank_number){
 uint32_t reg,reg2;
 sio_fd_t gpio_base = (void*)GPIO_BASE;
 uint32_t num;
 switch (bank_number)
 {
 case 0:
     reg =mmio_read32(gpio_base + GPEDS0);
     if(reg &= 0x0FFFFFFF){
         if( bit_count(reg) ==1){
             while(reg){
                 reg = reg >> 1;
                 num++;
             }
            return num-1;
         }
         else{
                //Fehler mehrere bits gesetzt
         }
     }else{/*FAIL*/}

     break;
 case 1:
  
     reg =mmio_read32(gpio_base + GPEDS0);
     reg2 =mmio_read32(gpio_base + GPEDS1);
     if((reg &= 0xF0000000) || (reg2 &= 0x00003FFF)){
      if((bit_count(reg) | bit_count(reg2)) == 1 && bit_count(reg) ^  bit_count(reg2)){
         if(reg){
             reg =reg >> 28;
              while(reg){
                 reg = reg >> 1;
                 num++;
              }
            return num+27;
         }else{
             while(reg2){
                 reg2 = reg2 >> 1;
                 num++;
             }
            return num+31; 
         }
      }
     }
     break;
 case 2:
     reg =mmio_read32(gpio_base + GPEDS1);
     if(reg &= 0x03FFC000){
         if(bit_count(reg)==1){
             reg = reg >> 14 ;
             while (reg)
             {
                 reg = reg >> 1;
                 num++;
             }
             return num+45;
         }else{/*Fehler*/}
    }else{/*Fehler*/}
     break;   

 default:
    //erro keine richtige Gpio Bank

     break;
 }
 return -1; //fail
}

/*Clear Interrupt event bit*/
void gpio_clearIRQStatus(uint32_t number){
 uint32_t reg,offset;
 sio_fd_t gpio_base = (void*)GPIO_BASE;
 reg=number/32;
 offset=number%32;
 mmio_write32(gpio_base + GPEDS0 + reg*register_size,1<< offset);
}

/*Resistor adjust:pull up =1 other , pull down =0*/
void gpio_resistor(uint32_t number, uint32_t up_down){
 uint32_t offset,reg,mask;
 sio_fd_t gpio_base = (void*)GPIO_BASE;
 offset = number%16;
 reg = number/16; 
 mask = 3 << offset*2;
 if(up_down){
  mmio_write32(gpio_base + GPIO_PUP_PDN_CNTRL_REG0 + reg*register_size,((mmio_read32(gpio_base + GPIO_PUP_PDN_CNTRL_REG0 + reg*register_size) & ~mask ) | (01 << 2*offset & mask)));
 }else{
  mmio_write32(gpio_base + GPIO_PUP_PDN_CNTRL_REG0 + reg*register_size,((mmio_read32(gpio_base + GPIO_PUP_PDN_CNTRL_REG0 + reg*register_size) & ~mask ) | (2 << 2*offset & mask)));
 }
 printf("Widerstand enable");
}