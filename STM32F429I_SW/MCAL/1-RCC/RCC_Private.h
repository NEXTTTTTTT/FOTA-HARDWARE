/**********************************************************
* Author : Ehab Mohamed 
* Version: V1.0
* Date: 8 Nov 2021
***********************************************************/
#ifndef RCC_PRIVATE_H
    #define RCC_PRIVATE_H

    #define RCC_AHB1ENR (*((volatile uint32*)0x400238030))
    #define RCC_AHB2ENR (*((volatile uint32*)0x400238034))
    #define RCC_AHB3ENR (*((volatile uint32*)0x400238038))
    #define RCC_APB1ENR (*((volatile uint32*)0x400238040))
    #define RCC_APB2ENR (*((volatile uint32*)0x400238044))

#endif


