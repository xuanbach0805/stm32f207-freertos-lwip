#include <uart.h>
/*#include <ringbuf.h>*/
#include <errno.h>
/* Scheduler includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* lwip includes */
#include <lwip/sys.h>


#define CON_UART    USART6

xQueueHandle xDebugQueue;
#define DBG_BUF_SIZE  1024



USART_InitTypeDef USART_Init_struct [] = 
{
  {
    .USART_BaudRate = 115200,
    .USART_WordLength = USART_WordLength_8b,
    .USART_StopBits = USART_StopBits_1,
    .USART_Parity = USART_Parity_No,
    .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None
  },
  {
    .USART_BaudRate = 57600,
    .USART_WordLength = USART_WordLength_9b,
    .USART_StopBits = USART_StopBits_1,
    .USART_Parity = USART_Parity_Even,
    .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None
  },
};

USART_PinsTypeDef USART_Pins [] =
{
  {
    .usart = USART1,
    .irq = USART1_IRQn,
    .clk = RCC_APB2Periph_USART1,
    .tx_pin = GPIO_Pin_6,
    .rx_pin = GPIO_Pin_7,
    .tx_gpio_port = GPIOB,
    .rx_gpio_port = GPIOB,
    .tx_port_clk = RCC_AHB1Periph_GPIOB,
    .rx_port_clk = RCC_AHB1Periph_GPIOB,
    .tx_source = GPIO_PinSource6,
    .rx_source = GPIO_PinSource7,
    .tx_af = GPIO_AF_USART1,
    .rx_af = GPIO_AF_USART1,
    .irq_prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+7,
    .is_rs485 = 0,
    .de_port = GPIOD,
    .de_port_clk = RCC_AHB1Periph_GPIOB,
    .de_pin = GPIO_Pin_6

  },
  {
    .usart = USART2,
    .irq = USART2_IRQn,
    .clk = RCC_APB1Periph_USART2,
    .tx_pin = GPIO_Pin_5,
    .rx_pin = GPIO_Pin_6,
    .tx_gpio_port = GPIOD,
    .rx_gpio_port = GPIOD,
    .tx_port_clk = RCC_AHB1Periph_GPIOD,
    .rx_port_clk = RCC_AHB1Periph_GPIOD,
    .tx_source = GPIO_PinSource5,
    .rx_source = GPIO_PinSource6,
    .tx_af = GPIO_AF_USART2,
    .rx_af = GPIO_AF_USART2,
    .irq_prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+6,
    .is_rs485 = 1,
    .de_port = GPIOD,
    .de_port_clk = RCC_AHB1Periph_GPIOD,
    .de_pin = GPIO_Pin_4
  },
  {
    .usart = USART3,
    .irq = USART3_IRQn,
    .clk = RCC_APB1Periph_USART3,
    .tx_pin = GPIO_Pin_10,
    .rx_pin = GPIO_Pin_11,
    .tx_gpio_port = GPIOB,
    .rx_gpio_port = GPIOB,
    .tx_port_clk = RCC_AHB1Periph_GPIOB,
    .rx_port_clk = RCC_AHB1Periph_GPIOB,
    .tx_source = GPIO_PinSource10,
    .rx_source = GPIO_PinSource11,
    .tx_af = GPIO_AF_USART3,
    .rx_af = GPIO_AF_USART3,
    .irq_prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+7,
    .is_rs485 = 0,
    .de_port = GPIOD,
    .de_port_clk = RCC_AHB1Periph_GPIOB,
    .de_pin = GPIO_Pin_6
  },
  {
    .usart = USART6,
    .irq = USART6_IRQn,
    .clk = RCC_APB2Periph_USART6,
    .tx_pin = GPIO_Pin_6,
    .rx_pin = GPIO_Pin_7,
    .tx_gpio_port = GPIOC,
    .rx_gpio_port = GPIOC,
    .tx_port_clk = RCC_AHB1Periph_GPIOC,
    .rx_port_clk = RCC_AHB1Periph_GPIOC,
    .tx_source = GPIO_PinSource6,
    .rx_source = GPIO_PinSource7,
    .tx_af = GPIO_AF_USART6,
    .rx_af = GPIO_AF_USART6,
    .irq_prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+7,
    .is_rs485 = 0,
    .de_port = GPIOD,
    .de_port_clk = RCC_AHB1Periph_GPIOB,
    .de_pin = GPIO_Pin_6
  }


};

void USART_Init_With_Irq(USART_InitTypeDef *init, USART_PinsTypeDef *pins)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable UART clock */
  if ((pins->usart == USART1) || (pins->usart == USART6))
  {
    RCC_APB2PeriphClockCmd(pins->clk, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(pins->clk, ENABLE);
  }
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(pins->tx_port_clk | pins->rx_port_clk, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(pins->tx_gpio_port,pins->tx_source,pins->tx_af);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(pins->rx_gpio_port,pins->rx_source,pins->rx_af);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = pins->tx_pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(pins->tx_gpio_port, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = pins->rx_pin;
  GPIO_Init(pins->rx_gpio_port, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(pins->usart, init);

  NVIC_InitStructure.NVIC_IRQChannel = pins->irq;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = pins->irq_prio;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /*USART_ITConfig(pins->usart,USART_IT_RXNE,ENABLE);*/
  if (pins->is_rs485)
  {
    printf("rs485\r\n");
    RCC_APB1PeriphClockCmd(pins->de_port_clk, ENABLE);
    GPIO_InitStructure.GPIO_Pin = pins->de_pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT,
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP,
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP,
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz,
    GPIO_Init(pins->de_port, &GPIO_InitStructure);
  }

  /* Enable USART */
  USART_Cmd(pins->usart, ENABLE);
}

#ifndef NULL_DBG
void USART6_IRQHandler(void)
{
  portBASE_TYPE xTaskWokenByPost = pdFALSE;

  if (USART_GetITStatus(CON_UART,USART_IT_TXE) == SET) {
      char c;
      if (xQueueReceiveFromISR(xDebugQueue, &c, &xTaskWokenByPost) == pdTRUE) {
          // send a queued byte
          //
          USART_SendData(CON_UART,c);
      }
      else {
          // nothing to send, disable interrupt
          //
          USART_ITConfig(CON_UART,USART_IT_TXE,DISABLE);
      }
  }
  if( xTaskWokenByPost != pdFALSE )
  {
    taskYIELD ();
  }
}
#endif

ssize_t uart_write_r(struct _reent *r, int fd, const void *ptr, size_t len)
{
    const char *c = (const char*) ptr;

    for (int i = 0; i < len; i++) {
        
        if( xQueueSend( xDebugQueue, ( void * ) c, ( portTickType ) 10 ) != pdTRUE )
        {
            // Failed to post the message, even after 10 ticks.
            if (i) USART_ITConfig(CON_UART,USART_IT_TXE,ENABLE);
            return i;
        }
        c++;
    }
    // Enable TX empty interrupt
    USART_ITConfig(CON_UART,USART_IT_TXE,ENABLE);
    return len;
}

ssize_t uart_read_r(struct _reent *r, int fd, void *ptr, size_t len)
{
  return 0;
}

void con_uart_init(void)
{
#ifndef NULL_DBG
  xDebugQueue = xQueueCreate( DBG_BUF_SIZE, sizeof( uint8_t ));
  USART_Init_With_Irq(&USART_Init_struct[0],&USART_Pins[3]);
#endif

}
