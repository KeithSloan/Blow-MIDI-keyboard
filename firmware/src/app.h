/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "driver/tmr/drv_tmr.h"
#include "system/ports/sys_ports.h"
#include "driver/usart/drv_usart.h"
#include "driver/usb/usbfs/drv_usbfs.h"
#include "driver/usb/usbfs/src/drv_usbfs_local.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/
/* SS1 is controlled by this GPIO */
#define SPI_SLAVE_1_CS_PORT_ID          PORT_CHANNEL_D

#define SPI_SLAVE_1_CS_PORT_PIN         PORTS_BIT_POS_4
#define APP_SPI_CS_SELECT(x,y)     \
                                        SYS_PORTS_PinClear(PORTS_ID_0,x,y)

#define APP_SPI_CS_DESELECT(x,y)   \
                                        SYS_PORTS_PinSet(PORTS_ID_0,x,y)    
    
void WriteSerialChar(char chr);

typedef enum
{
    /* Application's state machine's initial state. */
    SERIAL_STATE_INIT=0,
    SERIAL_TRANSMITING,
    SERIAL_IDLE,

} SERIAL_STATE;

typedef enum
{
    /* Application's state machine's initial state. */
    TIMER_STATE_INIT=0,
    TIMER_RUNNING,

} TIMER_STATE;

typedef enum
{
    /* Application's state machine's initial state. */
    USB_STATE_INIT=0,
    USB_WAITING_FOR_CONFIGURATION,
    USB_IDLE,
    USB_WRITE_COMMAND_IN_PROCESS,
    USB_WRITE_COMPLETE,        

} USB_STATE;

typedef enum
{
    /* Application's state machine's initial state. */
    SPI_STATE_INIT=0,
    SPI_START_CONVERSION,
    SPI_WAIT_DATA_READY,        
    SPI_READING,

} SPI_STATE;



#define Threashold 166
#define MaxValue 1568
#define Notes 8
#define Delta 90 // (MaxValue-Threashold)/Notes
#define Volume 124


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

 

typedef struct
{
    /* For Timer */
    TIMER_STATE             timerState;
    /* Heartbeat driver timer handle. */
    DRV_HANDLE              heartbeatTimer;
    /* Heartbeat timer timeout count. */
    unsigned int            heartbeatCount;
    /* Heartbeat LED toggle flag. */
    bool                    heartbeatToggle;
    /* For Serial */
    SERIAL_STATE            serialState;
    DRV_HANDLE              serialDevHandle;
    DRV_USART_BUFFER_HANDLE serialBufferHandle;
    char                    serialBuffer[80];
    char                    tx_byte;
    /* For SPI */
    SPI_STATE               spiState;
    DRV_HANDLE              spiDevHandle;
    DRV_SPI_BUFFER_HANDLE   spiBufferHandle;
    uint8_t                 spiBuffer[4];
    int                     maxVal;
    int                     note;
    /* For MIDI USB */
    USB_STATE               usbState;
    DRV_HANDLE              usbDevHandle;
    bool                    deviceIsConfigured;
    USB_DEVICE_TRANSFER_HANDLE readTransferHandle;
    bool                    epDataReadPending;
    bool                    epDataWritePending;
    bool                    epDataWriteEnabled;
    bool                    epDataReadEnabled;
    bool                    noteOn;
    int                     midiNote;
    int                     altSetting;
    int                     configurationValue;
    uint8_t                 endpointRx;
    uint8_t                 endpointTx;
} APP_DATA;




// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
#define APP_HEARTBEAT_TMR               DRV_TMR_INDEX_0
#define APP_HEARTBEAT_TMR_IS_PERIODIC   true
#define APP_HEARTBEAT_TMR_PERIOD        0x2000
#define APP_HEARTBEAT_COUNT_MAX         6
#define APP_HEARTBEAT_PORT              PORT_CHANNEL_E
#define APP_HEARTBEAT_PIN               PORTS_BIT_POS_6
/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );


#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

