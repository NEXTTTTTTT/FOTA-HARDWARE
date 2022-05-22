/**********************************************************
* Author : Ehab Mohamed 
* Version: V1.0
* Date: 8 Nov 2021
***********************************************************/
#ifndef RCC_INTERFACE_H
    #define RCC_INTERFACE_H

    enum Bus
    {
        AHB1, AHB2, AHB3, APB1, APB1
    };

    enum AHB1_PerID
    {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG 
    };

    enum AHB2_PerID
    {
        DCMI
    };

    enum AHB3_PerID
    {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,GPIOG 
    };

    enum APB1_PerID
    {
        TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM12,TIM13,
        TIM14, WWDG = 11, SPI2 = 14, SPI3, USART2 = 17,
        USART3, USART4, USART5, I2C1, I2C2, I2C3
    };

    enum APB2_PerID
    {
        TIM1, TIM8, USART1 = 4, USART6
    };

    /* Public Functions Prototypes */
    void RCC_EnableClock(uint8, uint8);

#endif