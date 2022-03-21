 /******************************************************************************
 *
 * Module: UART
 *
 * File Name: UART_IRQ.c
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - UART Driver.
 *
 * Author: Ehab Mohamed
 ******************************************************************************/
#include "UART.h"
#include "UART_Regs.h"


#if(UART0_INTERRUPT_ENABLE == STD_ON)
   volatile STATIC uint32* UARTRegs_Ptr = (volatile uint32*)UART0_BASE_ADDRESS;

   void UART0_Handler()
   {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)UARTRegs_Ptr + UART_ICR_REG_OFFSET), 4);
      UART1_RecDataBuffer = *(volatile uint32 *)((volatile uint8 *)UARTRegs_Ptr + UART_DATA_REG_OFFSET);
      
      if(UART0_CallBackPtr != NULL_PTR)UART0_CallBackPtr();
      
   }    // End UART0_Handler() function 

#endif

#if(UART1_INTERRUPT_ENABLE == STD_ON)
   volatile STATIC uint32* UARTRegs_Ptr = (volatile uint32*)UART1_BASE_ADDRESS;
   volatile void(*UART1_CallBackPtr)(void) = NULL_PTR;
   volatile uint8 UART1_RecDataBuffer;

   void UART1_Handler()
   {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)UARTRegs_Ptr + UART_ICR_REG_OFFSET), 4);
      UART1_RecDataBuffer = *(volatile uint32 *)((volatile uint8 *)UARTRegs_Ptr + UART_DATA_REG_OFFSET);
      
      if(UART1_CallBackPtr != NULL_PTR)UART1_CallBackPtr();
      
   }   /* End UART1_Handler() function */
#endif