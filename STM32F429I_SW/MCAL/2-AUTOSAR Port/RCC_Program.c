/**********************************************************
* Author : Ehab Mohamed 
* Version: V1.0
* Date: 8 Nov 2021
***********************************************************/
#include "Platform_Types.h"
#include "Bit_Math.h"


#include "RCC_Interface.h"
#include "RCC_Private.h"
#include "RCC_Config.h"

void RCC_InitSystemClock()
{
	#if(SYS_CLK == HSI)
	        SET_BIT(RCC_CR, 0);    /* Enable HSI + Trimming=0 */

	        while(BIT_IS_CLEAR(RCC_CR, 1) );

	        RCC_CFGR &= 0xFFFFFFFC;

	#elif(SYS_CLK == HSE)
	        SET_BIT(RCC_CR, 16);     /* Enable HSE with no bypass */

	        while(BIT_IS_CLEAR(RCC_CR, 17) );

	        SET_BIT(RCC_CFGR, 0);

	#elif(SYS_CLK == PLL)
	        SystemInit();

	        /* Enable HSE To be used as the system clock before
	         * configuring the PLL

	        SET_BIT(RCC_CR, 16);      Enable HSE with no bypass

	       	 while(BIT_IS_CLEAR(RCC_CR, 17) );

	         SET_BIT(RCC_CFGR, 0);
	         CLEAR_BIT(RCC_CR, 24);	  Disable PLL

	         PLL Configuration
	        RCC_PLLCFGR = (RCC_PLLCFGR & 0xFF00FFC0) | 0x00402C04;
	        RCC_CFGR = (RCC_CFGR & 0xFFFF000F) | 0x00009400;
	        SET_BIT(RCC_CR, 24);

	        while(BIT_IS_CLEAR(RCC_CR, 25) );

	        RCC_CFGR = (RCC_CFGR & 0xFFFFFFF0) | 0x00000002;*/


    #else
        #error("You chose the wrong Clock Type")

    #endif

}

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


