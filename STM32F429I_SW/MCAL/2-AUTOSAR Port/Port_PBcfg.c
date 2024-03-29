 /******************************************************************************
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for STM32F249I 
 *              Microcontroller - Port AUTOSAR Driver
 *
 * Author: Ehab Mohamed
 ******************************************************************************/



/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#include "Port.h"
   
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_PBcfg.c does not match the expected version"
#endif

   /* Symbolic names for major(most used) configurations */
typedef enum
{
  PIN_DEFAULT_CONFIG, PIN_OUT_HIGH, PIN_IN_PULLUP_HIGH, PIN_IN_PULL_DOWN, PIN_OUT_LOW, PIN_IN_PULLUP_LOW, PA0_U0RX, PA1_U0TX
}MajorPinsConfig;

/* Array of structures contians most used  pin configurations*/
STATIC const Port_configChannel pinsConfigSettings[] = { {PORT_PIN_IN, STD_LOW, PIN_PUSH_PULL,OFF, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_OUT, STD_HIGH, PIN_PUSH_PULL,OFF, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_IN, STD_HIGH, PIN_PUSH_PULL,PULL_UP, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_IN, STD_HIGH, PIN_PUSH_PULL,PULL_DOWN, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_OUT, STD_LOW, PIN_PUSH_PULL,OFF, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_IN, STD_LOW, PIN_PUSH_PULL,PULL_UP, PIN_DIO_MODE, STD_OFF, STD_OFF},
                                                         {PORT_PIN_IN, STD_LOW, PIN_PUSH_PULL,OFF, 1, STD_OFF, STD_OFF},
                                                         {PORT_PIN_OUT, STD_LOW, PIN_PUSH_PULL,OFF, 1, STD_OFF, STD_OFF}
                                                       };

/* 
 * PB configuration structure used by the Port_Init() fucntion to initialze all
 *   pins.
 * It is an array of pointers to structures.Each index is set with the address of
 *  the configuration structure of the corresponding pin.
 */
const Port_ConfigType Port_Configuration = {  &pinsConfigSettings[PA0_U0RX],                 /* PA0 */
                                              &pinsConfigSettings[PA1_U0TX],                 /* PA1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA6 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA8 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA9 */            
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA10 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA11 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA12 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA13 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA14 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PA15 */

                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB6 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB8 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB9 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB10 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB11 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB12 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB13 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB14 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PB15 */

                                              NULL_PTR,                                         /* PC0 */
                                              NULL_PTR,                                         /* PC1 */
                                              NULL_PTR,                                          /* PC2 */
                                              NULL_PTR,                                          /* PC3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC6 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC8 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC9 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC10 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC11 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC12 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC13 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC14 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PC15 */
  
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD6 */
                                              NULL_PTR,                                              /* PD7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD8 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD9 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD10 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD11 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD12 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD13 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD14 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PD15 */

                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE4 */                                                    
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE6 */
                                              NULL_PTR,                                              /* PE7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE8 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE9 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE10 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE11 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE12 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE13 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE14 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PE15 */

                                              &pinsConfigSettings[PIN_IN_PULLUP_HIGH],                 /* PF0 */
                                              &pinsConfigSettings[PIN_OUT_LOW],                 /* PF1 */
                                              &pinsConfigSettings[PIN_OUT_LOW],                 /* PF2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PF3 */
                                              &pinsConfigSettings[PIN_IN_PULLUP_HIGH],                  /* PF4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PF5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PF6 */
                                              NULL_PTR,                                              /* PF7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF8 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF9 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF10 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF11 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF12 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF13 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF14 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PF15 */

                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG6 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PG7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PG8 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PG9 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PG10 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PG11 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PG12 */
                                              &pinsConfigSettings[PIN_OUT_LOW],      /* PG13 */
                                              &pinsConfigSettings[PIN_OUT_LOW],      /* PG14 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],      /* PG15 */

                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PH0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG]                 /* PH1 */
                                             };