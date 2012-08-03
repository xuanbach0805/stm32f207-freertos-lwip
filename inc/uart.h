#ifndef UART_H
#define UART_H


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stm32f2xx.h>
#include <sys/types.h>
#include <reent.h>

typedef struct
{
  USART_TypeDef* usart;
  uint8_t irq;
  uint32_t  clk;
  uint16_t  tx_pin;
  uint16_t  rx_pin;
  GPIO_TypeDef* tx_gpio_port;
  GPIO_TypeDef* rx_gpio_port;
  uint32_t  tx_port_clk;
  uint32_t  rx_port_clk;
  uint8_t tx_source;
  uint8_t rx_source;
  uint8_t tx_af;
  uint8_t rx_af;
  uint8_t irq_prio;
  uint8_t is_rs485;
  GPIO_TypeDef* de_port;
  uint32_t  de_port_clk;
  uint16_t  de_pin;

} USART_PinsTypeDef;

extern void  uart_init(int baudrate);
extern void  uart_poll_send(const char *s);
extern int   uart_chars_avail(void);
void con_uart_init(void);
void USART_Init_With_Irq(USART_InitTypeDef *init, USART_PinsTypeDef *pins);

extern ssize_t uart_write_r(struct _reent *r, int fd, const void *ptr, size_t len);
extern ssize_t uart_read_r(struct _reent *r, int fd, void *ptr, size_t len);

#endif
