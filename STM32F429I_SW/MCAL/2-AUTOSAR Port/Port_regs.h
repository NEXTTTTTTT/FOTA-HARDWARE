 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Regs.h
 *
 * Description: Header file for STM32F249I Microcontroller - Port Driver Registers
 *
 * Author: Ehab Mohamed
 ******************************************************************************/

#ifndef PORT_REGS_H
#define PORT_REGS_H

#include "Std_Types.h"

/*******************************************************************************
 *                              GPIO_Regs Definitions                          *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40002000
#define GPIO_PORTB_BASE_ADDRESS           0x40020400
#define GPIO_PORTC_BASE_ADDRESS           0x40020800
#define GPIO_PORTD_BASE_ADDRESS           0x40020C00
#define GPIO_PORTE_BASE_ADDRESS           0x40021000
#define GPIO_PORTF_BASE_ADDRESS           0x40021400
#define GPIO_PORTG_BASE_ADDRESS           0x40021800
#define GPIO_PORTH_BASE_ADDRESS           0x40021C00

/* GPIO Registers offset addresses */
#define PORT_MODER_OFFSET          0x00
#define PORT_OTYPER_OFFSET         0x04
#define PORT_PUPDR_OFFSET          0x0C
#define PORT_ODR_OFFSET            0x14
#define PORT_AFRL_OFFSET           0x20
#define PORT_AFRH_OFFSET           0x24

#endif /* PORT_REGS_H */