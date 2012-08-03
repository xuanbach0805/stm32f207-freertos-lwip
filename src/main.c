#include <stm32f2xx.h>
#include "uart.h"
#include "ustime.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <uart.h>



void led_init(void)
{
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
GPIO_Init(GPIOD, &(GPIO_InitTypeDef) {
    .GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15/* | GPIO_Pin_4*/,
    .GPIO_Mode  = GPIO_Mode_OUT,
    .GPIO_OType = GPIO_OType_PP,
    .GPIO_PuPd  = GPIO_PuPd_UP,
    .GPIO_Speed = GPIO_Speed_100MHz,
});
}

void set_leds(int leds)
{
    GPIOD->ODR = (GPIOD->ODR & ~0xF000) | ((leds << 12) & 0xF000);
}


void init_task(void *pvParameters)
{
    
    while(1)
    {
        vTaskDelay(477);
        GPIOD->ODR ^= GPIO_Pin_14;

    }
    

    for(;;);
}

int main(void)
{
    // FreeRTOS assumes 4 preemption- and 0 subpriority-bits
    //
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    led_init();
    con_uart_init();
    printf("FreeRtos \r\n");
    
    ETH_BSP_Config();
    LwIP_Init();
    
    



    
    

    // Create init task and start the scheduler
    //
    xTaskCreate(init_task, (signed char*)"init", 1024, NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    
}
