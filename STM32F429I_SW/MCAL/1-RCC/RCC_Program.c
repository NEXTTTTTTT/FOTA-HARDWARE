/**********************************************************
* Author : Ehab Mohamed 
* Version: V1.0
* Date: 8 Nov 2021
***********************************************************/
#include "Platform_Types.h"
#include "Common_Macros.h"

#include "RCC_Interface.h"
#include "RCC_Private.h"
#include "RCC_Config.h"

void RCC_EnableClock(uint8 Bus, uint8 PerId)
{
    
    if(PerId <= 31)
    {
       
        switch(Bus)
        {
            case AHB1 :
                SET_BIT(RCC_AHB1ENR, PerId);
                break;
            
            case AHB2 :
                SET_BIT(RCC_AHB2ENR, PerId);
                break;

            case AHB3 :
                SET_BIT(RCC_AHB3ENR, PerId);
                break;

            case APB1:
                SET_BIT(RCC_APB1ENR, PerId);
                break;

            case APB2:
                SET_BIT(RCC_APB2ENR, PerId);
                break;
        }

    }
    else
    {
        /* Return Error */
    }

}   /* End RCC_EnableClock() */

void RCC_DisableClock(uint8 Bus, uint8 PerId)
{
    
    if(PerId <= 31)
    {
       
        switch(Bus)
        {
            case AHB :
                CLEAR_BIT(RCC_AHBENR, PerId);
                break;

            case APB1:
                CLEAR_BIT(RCC_APB1ENR, PerId);
                break;

            case APB2:
                CLEAR_BIT(RCC_APB2ENR, PerId);
                break;
        }

    }
    else
    {
        /* Return Error */
    }

}   /* End RCC_EnableClock() */

void RCC_InitSystemClock()
{
    
    #if(RCC_CLOCK_TYPE == RCC_HSI)
        RCC_CR = 0x00000081;   /* Enable HSI + Trimming=0 */
        RCC_CFGR = 0x00000000;

    #elif(RCC_CLOCK_TYPE == RCC_HSE_CRYSTAL)
        RCC_CR = 0x00010000;    /* Enable HSE with no bypass */
        RCC_CFGR = 0x00000001;
        
    #elif(RCC_CLOCK_TYPE == RCC_HSE_RC)
        RCC_CR = 0x00050000;    /* Enable HSE with bypass */
        RCC_CFGR = 0x00000001;

    #elif(RCC_CLOCK_TYPE == RCC_PLL)

        #if(RCC_PLL_INPUT == RCC_PLL_IN_HSI_DIV_2)

        #elif(RCC_PLL_INPUT == RCC_PLL_IN_HSE_DIV_2)

        #elif(RCC_PLL_INPUT == RCC_PLL_IN_HSE)

        #endif


        RCC_CFGR = (RCC_CFGR & 0xFFCCFFFF) | (PLL_MULL << 18);
        SET_BIT(RCC_CR, 24);
    
    #else
        #error("You chosed the wrong Clock Type")

    #endif

}