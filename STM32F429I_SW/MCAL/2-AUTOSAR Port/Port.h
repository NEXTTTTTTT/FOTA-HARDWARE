 /******************************************************************************
 *
 * Module: Port Driver
 *
 * File Name: Port.h
 *
 * Description: Header file for STM32F429I Microcontroller - AUTOSAR Port Driver.
 *
 * Author: Ehab Mohamed
 ******************************************************************************/

#ifndef PORT_H
  #define PORT_H

  /* Id for the company in the AUTOSAR
  * for example Ehab Mohamed's ID = 2022 :) */
  #define PORT_VENDOR_ID    (2022U)

  /* PORT Module Id */
  #define PORT_MODULE_ID    (124U)

  /* PORT Instance Id */
  #define PORT_INSTANCE_ID  (0U)

  /*
  * Module Version 1.0.0
  */
  #define PORT_SW_MAJOR_VERSION           (1U)
  #define PORT_SW_MINOR_VERSION           (0U)
  #define PORT_SW_PATCH_VERSION           (0U)

  /*
  * AUTOSAR Version 4.0.3
  */
  #define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
  #define PORT_AR_RELEASE_MINOR_VERSION   (0U)
  #define PORT_AR_RELEASE_PATCH_VERSION   (3U)

  /* Standard AUTOSAR types */
  #include "Std_Types.h"
    
  /* AUTOSAR checking between Std Types and Port Modules */
  #if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
  ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
  ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
    #error "The AR version of Std_Types.h does not match the expected version"
  #endif
    
  /* Port Pre-Compile Configuration Header file */
  #include "Port_Cfg.h"
    
    /* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
  #if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
  ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
  ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
    #error "The AR version of Port_Cfg.h does not match the expected version"
  #endif

  /* Software Version checking between PORT_Cfg.h and Port.h files */
  #if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
  ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
  ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
    #error "The SW version of Port_Cfg.h does not match the expected version"
  #endif

  /* Non AUTOSAR files */
  #include "Bit_Math.h"
    
  /******************************************************************************
   *                      API Service Id Macros                                 *
   ******************************************************************************/
  /* Service ID for Port_Init */
  #define PORT_INIT_SID                         (uint8)0x00

  /* Service ID for Port_SetPinDirection */
  #define PORT_SET_PIN_DIRECTION_SID            (uint8)0x01

  /* Service ID for Port_RefreshPortDirection */
  #define PORT_REFRESH_PORT_DIRECTION_SID       (uint8)0x02

  /* Service ID for Port_GetVersionInfo */
  #define PORT_GET_VERSION_INFO_SID             (uint8)0x03

  /* Service ID for Port_SetPinMode */
  #define PORT_SET_PIN_MODE_SID                 (uint8)0x04
    
  /*
  * Macros for Port Status
  */
  #define PORT_INITIALIZED                TRUE
  #define PORT_NOT_INITIALIZED            FALSE
    
  /*******************************************************************************
   *                      DET Error Codes                                        *
   *******************************************************************************/
  /* DET code to report Invalid port pin ID requested */
  #define PORT_E_PARAM_PIN              (uint8)0x0A

  /* DET code to report Port Pin not configured as changeable */
  #define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B
    
  /* API Port_Init service called with wrong parameter */
  #define PORT_E_PARAM_CONFIG           (uint8)0x0C

  /* API Port_SetPinMode service called when mode is unchangeable */
  #define PORT_E_PARAM_INVALID_MODE     (uint8)0x0D
  #define PORT_E_MODE_UNCHANGEABLE      (uint8)0x0E

  /* API service called without module initialization */
  #define PORT_E_UNINIT                 (uint8)0x0F

  /*
  * The API service shall return immediately without any further action,
  * beside reporting this development error.
  */
  #define PORT_E_PARAM_POINTER          (uint8)0x10

  /*******************************************************************************
   *                              Module Data Types                              *
   *******************************************************************************/
  /* Description: Enum to hold PIN direction */
  typedef enum
  {
      PORT_PIN_IN, PORT_PIN_OUT
  }Port_PinDirectionType;

  /* Description: Enum to hold PIN output type */
  typedef enum
  {
      PIN_PUSH_PULL, PIN_OPEN_DRAIN
  }Port_PinOutputType;

  /* Description: Enum to hold internal resistor type for PIN */
  typedef enum
  {
      OFF, PULL_UP, PULL_DOWN
  }Port_InternalResistor;

  /* Description: Structure to configure each individual PIN:
  *   1. The direction of pin --> INPUT or OUTPUT
  *   2. The intial value of the output pin
  *   3. The internal resistor of the input pin --> Disable, Pull up or Pull down
  *   4. The mode of the pin --> GPIO, ADC, CAN,...
  *   5. The direction of the pin is changeable during runtime -> STD_ON,STD_OFF
  *   6. The mode of the pin is changeable during runtime -> STD_ON,STD_OFF
  */
  typedef struct
  {
    Port_PinDirectionType direction;
    uint8                 initial_value;
    Port_PinOutputType    output_type;
    Port_InternalResistor resistor;
    uint8                 pin_mode;
    boolean               pin_direction_changeable;
    boolean               pin_mode_changeable;
  }Port_configChannel;

  /* Description: Structure to configure all 113 pins in SMT32F429I*/
  typedef struct 
  {
    const Port_configChannel *MCU_Channels[MCU_TOTAL_PINS_NUM];   /* Array of pointers to structures instead of array of structures to save storage */
  }Port_ConfigType;

  /* Description: Enum for the symbolic names of  port pins in SMT32F429I*/
  typedef uint8 Port_PinType;
      
  /* Type definition for different port pin modes used by Port APIs. */
  typedef uint8 Port_PinModeType;

  /* Symbolic names for TM4C123GH6PM channels */
  typedef enum
  {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
    PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
    PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
    PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
    PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
    PH0, PH1
  }MCU_channelsNames;

  /*******************************************************************************
   *                       External Variables                                    *
   *******************************************************************************/

  /* Extern PB structures to be used by Port.c and other modules */
  extern const Port_ConfigType Port_Configuration;

  /*******************************************************************************
   *                      Function Prototypes                                    *
   *******************************************************************************/

  /*******************************************************************************
  * Service Name: Port_Init
  * Service ID[hex]: 0x00
  * Sync/Async: Synchronous
  * Reentrancy: Non Reentrant
  * Parameters (in): ConfigPtr - Pointer to post-build configuration data
  * Parameters (inout): None
  * Parameters (out): None
  * Return value: None
  * Description: Initializes the Port Driver module.
  ********************************************************************************/
  void Port_Init(const Port_ConfigType*);

  /*******************************************************************************
  * Service Name: Port_SetPinDirection
  * Service ID[hex]: 0x01
  * Sync/Async: Synchronous
  * Reentrancy: Reentrant
  * Parameters (in): Pin->Port Pin ID number, Direction->Port Pin Direction
  * Parameters (inout): None
  * Parameters (out): None
  * Return value: None
  * Description: Sets the port pin direction.
  ********************************************************************************/
  void Port_SetPinDirection(Port_PinType, Port_PinDirectionType);

  /*******************************************************************************
  * Service Name: Port_RefreshPortDirection
  * Service ID[hex]: 0x02
  * Sync/Async: Synchronous
  * Reentrancy: Non Reentrant
  * Parameters (in): None
  * Parameters (inout): None
  * Parameters (out): None
  * Return value: None
  * Description: Refreshes port direction.
  ********************************************************************************/
  void Port_RefreshPortDirection(void);

  /*******************************************************************************
  * Service Name: Port_GetVersionInfo
  * Service ID[hex]: 0x03
  * Sync/Async: Synchronous
  * Reentrancy: Non Reentrant
  * Parameters (in): None
  * Parameters (inout): None
  * Parameters (out): versioninfo
  * Return value: None
  * Description: Returns the version information of this module..
  ********************************************************************************/
  void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);

  /*******************************************************************************
  * Service Name: Port_SetPinMode
  * Service ID[hex]: 0x04
  * Sync/Async: Synchronous
  * Reentrancy: Reentrant
  * Parameters (in): Pin->Port Pin ID number, Mode->New Port Pin mode to be set on
                    port pin.
  * Parameters (inout): None
  * Parameters (out): None
  * Return value: None
  * Description: Sets the port pin mode.
  ********************************************************************************/
  void Port_SetPinMode(Port_PinType, Port_PinModeType);


#endif /* PORT_H */
