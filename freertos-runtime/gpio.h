#ifndef _GPIO_
#define _GPIO_



typedef void * sio_fd_t;

int gpio_pin_output_enable(uint32_t number);
int gpio_pin_input_enable(uint32_t number);

int gpio_pin_disable(uint32_t number);


void gpio_pin_irq_rx_enable(uint32_t number);
//void gpio_pin_irq_wx_enable(uint32_t number);

void gpio_pin_irq_rx_disanable(uint32_t number);
//void gpio_pin_irq_wx_disable(uint32_t number); 

void gpio_output(uint32_t number, uint32_t state);
int gpio_input(uint32_t number);

int gpio_irq_input_number(uint32_t bank_number);




#endif