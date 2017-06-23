/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

int MIDI_LOOKUP[9] = {0,52,53,55,57,59,60,62,64};

/* Receive data buffer */
uint8_t receivedDataBuffer[64] __attribute__((coherent, aligned(4)));;

/* Transmit data buffer */
uint8_t transmitDataBuffer[64] __attribute__((coherent, aligned(4)));;

//#define Write 1
#define WriteN 1        // Enable Write Endpoint
//#define Read 1

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// ****************************************************************************


/* TODO:  Add any necessary local functions.
*/
#define READ_CORE_TIMER()                 _CP0_GET_COUNT()          // Read the MIPS Core Timer
 
void BSP_DelayUs(uint16_t microseconds)
{
    uint32_t time;
    
    time = READ_CORE_TIMER(); // Read Core Timer    
    time += (SYS_CLK_FREQ / 2 / 1000000) * microseconds; // calc the Stop Time    
    while ((int32_t)(time - READ_CORE_TIMER()) > 0){};    
}

//void WriteSerialChar(char chr)
//{
    // make sure the transmit buffer is not full before trying to write byte 
//    if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART_TransferStatus(appData.serialDevHandle)) )
//        {
//        appData.tx_byte = chr;
//        DRV_USART_WriteByte(appData.serialDevHandle, appData.tx_byte);  // send byte
//        }
//}

void WriteSysEx()
{

uint8_t SysEx[22];
SysEx[0] = 0xF0;
SysEx[1] = 0x7E;
SysEx[2] = 0x7F;
SysEx[3] = 0x06;
SysEx[4] = 0x02;
SysEx[5] = 0x00;
SysEx[6] = 0x01;
SysEx[7] = 0x05;
SysEx[8] = 0x63;
SysEx[9] = 0x0E;
SysEx[10] = 0x2D;
SysEx[11] = 0x50;
SysEx[12] = 0x30;
SysEx[13] = 0x31;
SysEx[14] = 0x30;
SysEx[15] = 0x30;
SysEx[16] = 0xF7;
SysEx[17] = 0x00;
SysEx[18] = 0x00;
SysEx[19] = 0x00;
SysEx[20] = 0x00;
SysEx[21] = 0x00;
USB_DEVICE_TRANSFER_HANDLE dvcTransferHandle;
USB_DEVICE_RESULT result;
result = USB_DEVICE_EndpointWrite( appData.usbDevHandle,
                               &dvcTransferHandle,
                               appData.endpointTx,
                               SysEx, 20,
                               USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);
    if (result != USB_DEVICE_RESULT_OK)
       {
       BSP_LED_1On();
       }
    WriteSerialChar('S');
}

void ReadReceiveEP()
{
    USB_DEVICE_RESULT result;
    
    result = USB_DEVICE_EndpointRead(appData.usbDevHandle, &appData.readTransferHandle,
            //            appData.endpointRx, &receivedDataBuffer[0], sizeof(receivedDataBuffer) );
            appData.endpointRx, &receivedDataBuffer[0], 16 );
    if (result != USB_DEVICE_RESULT_OK)
        {
        BSP_LED_1On();
        }
    WriteSerialChar('R');
    appData.epDataReadPending = true;
}



void WriteMIDI(uint8_t Note, uint8_t Key,uint8_t Velocity)
{
    struct __attribute__((packed coherent, aligned(16)))
    {
        uint8_t CN_CIN;         //Cable Number(Jack) + CIN 
        uint8_t CIN_Channel;    //Language ID of this string.
        uint8_t key;
        uint8_t velocity;
    } databuffer;
    
    
    databuffer.CN_CIN = 0x10 | Note;        // Note 0x08 Note off 0x09 Note On
    databuffer.CIN_Channel = (Note << 4) | 0x01; // Channel 0 or 1?
    databuffer.key = Key;                   // 95, 96
    databuffer.velocity = Velocity;         // 23, 24  
    
    USB_DEVICE_TRANSFER_HANDLE dvcTransferHandle;
    USB_DEVICE_RESULT result;
 
// Has the USB device been plugged in and configured for write  
if (appData.epDataWriteEnabled == true )
   {
    WriteSerialChar('W');
    // Sending 32 bits of data from 'databuffer to the TxEndpoint
   result = USB_DEVICE_EndpointWrite( appData.usbDevHandle,
                               &dvcTransferHandle,
                               appData.endpointTx,
                               &databuffer, 4,
                               USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);
   if (result != USB_DEVICE_RESULT_OK)
      {
      WriteSerialChar('E');
      BSP_LED_1On();
      }
   appData.epDataWritePending = true;
   }   
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

// Note Stack

#define size 16
struct stack {
   int NoteType[size];
   int NoteValue[size];
   int top;
} st;

int getNoteType()
{
   int nt;
   nt = st.NoteType[st.top];
   return (nt);
}

int getNoteVal()
{
    int nv;
    nv = st.NoteValue[st.top];
    return(nv);
}

void Push(int type,int val)
{
   st.top++;
   st.NoteType[st.top] = type;
   st.NoteValue[st.top] = val;
}

void Pop()
{
    st.top--;
}

int stackEmpty()
{
   if (st.top == -1)
      return 1;
   else
      return 0;
}

int initStack()
{
    st.top = -1;
}

/* TODO:  Add any necessary callback functions.
*/
/* Timer Call back*/
static void APP_TimerCallback (  uintptr_t context, uint32_t  alarmCount )
    {
    appData.heartbeatCount++;
    if (appData.heartbeatCount >= APP_HEARTBEAT_COUNT_MAX)
        {
        appData.heartbeatCount  = 0;
        appData.heartbeatToggle = true;
        }
    }

void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
            DRV_USART_BUFFER_HANDLE bufferHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_USART_BUFFER_EVENT_COMPLETE:

                // This means the data was transferred.
                BSP_LED_2On();
                appData.serialState = SERIAL_IDLE;
                break;

            case DRV_USART_BUFFER_EVENT_ERROR:

                // Error handling here.
                BSP_LED_1On();
                break;

            default:
                BSP_LED_3On();
                break;
        }
    }



void QueueSerialBuffer(void)
{
    DRV_USART_BufferAddWrite(appData.serialDevHandle,&appData.serialBufferHandle,
            (char *)&appData.serialBuffer[0],40);
    appData.serialState = SERIAL_TRANSMITING;
         
}

void WriteSerialChar(char chr)
{
    BSP_LED_2Off();
    appData.serialBuffer[0] = chr;
    DRV_USART_BufferAddWrite(appData.serialDevHandle,&appData.serialBufferHandle,
            (char *)&appData.serialBuffer[0],1);
    appData.serialState = SERIAL_TRANSMITING;
}

void APP_SPIEventHandler(DRV_SPI_BUFFER_EVENT event,
        DRV_SPI_BUFFER_HANDLE bufferHandle, void * context )
{
    int val, note, midiNote;
    bool newNote;
    int i = (int) event;
    switch(event)
    {
        case DRV_SPI_BUFFER_EVENT_PROCESSING:
            
            APP_SPI_CS_SELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
            WriteSerialChar('a');
            break;
            
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
             // WriteSerialChar('b');
            if ( appData.spiBuffer[0] != 0xFF)
               {
               val = (appData.spiBuffer[0] << 8 ) | appData.spiBuffer[1];
               if (val > appData.maxVal)
                  {
                  appData.maxVal = val;
                  }
                
               //note = (val + Delta - Threashold)/Delta;
               //if ( note > sizeof(MIDI_LOOKUP) ) note = sizeof(MIDI_LOOKUP);
               //midiNote = MIDI_LOOKUP[note];
               // Set new note boolean
               //newNote = ((note > 0) && (appData.midiNote != midiNote));
               // Do we need to send NoteOff
               //if ( appData.midiNote != 0) // Previous NoteOn ?
               //   {
               //   if ( note == 0 || newNote )
               //      {
                     // Send NoteOff
                     //WriteMIDI(0x08,appData.midiNote,0);
                      //Push(0x08,appData.midiNote);
                     //sprintf(appData.serialBuffer,"Note OFF\n");
                     //QueueSerialBuffer();
                 //    }
                 // }
                // Do we need to send NoteOn
               //if (((note > 0 ) && (appData.midiNote == 0)) || newNote )
               //   {
                  // Send NoteOn
                  //WriteMIDI(0x09,midiNote,Volume);
                  //Push(0x09,midiNote);
                  //sprintf(appData.serialBuffer,"Note ON\n");
                  //QueueSerialBuffer();
                  //}
                // Do we need to send NoteOn
                if ( val > Threashold && appData.keybrdState == KEYBRD_NOKEY)
                   {
                   WriteSerialChar('c');
                   appData.midiNote = 60;
                   // Send NoteON
                   WriteMIDI(0x09,midiNote,Volume);
                   WriteSerialChar('g');
                   appData.keybrdState = KEYBRD_KEYPRESS;
                   sprintf(appData.serialBuffer," Read %2x %2x %2x Val %5d Max %5d %d %2d\n",
                         appData.spiBuffer[0],appData.spiBuffer[1],appData.spiBuffer[2],
                         val,appData.maxVal,note,appData.midiNote);
                   QueueSerialBuffer();
                   BSP_LED_3On();
                   }
                //Do we need to send NoteOff
                if ( val <= Threashold && appData.keybrdState == KEYBRD_KEYPRESS)
                   {
                    WriteSerialChar('d');
                   // Send NoteOff
                   WriteMIDI(0x08,midiNote,0);
                   WriteSerialChar('h');
                   appData.keybrdState = KEYBRD_NOKEY;
                   }
                }
            else
                {
                // Should not occur
                WriteSerialChar('1');
                BSP_LED_1On();
                }   
            
            // Should work by leaving CS high for continuous conversion but appears not
            //appData.spiState = SPI_WAIT_DATA_READY;
            APP_SPI_CS_DESELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
            //WriteSerialChar('e');
            appData.spiState = SPI_START_CONVERSION;
            break;
            
        default:
            WriteSerialChar('f');
            break;
    }
  //WriteSerialChar('g');  
}

/* USB Event handler is a call back for USB handling*/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    uint8_t * configurationValue;
    USB_SETUP_PACKET * setupPacket;
    USB_DEVICE_RESULT result;
    struct eventStruct{
                USB_DEVICE_TRANSFER_HANDLE transferHandle;
                size_t length;
                USB_DEVICE_RESULT status;
            };
            
    struct eventStruct *ptr;
    size_t len;
    USB_DEVICE_RESULT res;
    
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
            WriteSerialChar('X');
            BSP_LED_2On();
            appData.deviceIsConfigured = false;
            break;
            
        case USB_DEVICE_EVENT_DECONFIGURED:
            WriteSerialChar('D');
            appData.deviceIsConfigured = false;
            //appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            break;
            
        case USB_DEVICE_EVENT_CONFIGURED:
            //BSP_LED_2On();
            WriteSerialChar('C');
            appData.deviceIsConfigured = true;
            
            /* Save the other details for later use. */
            appData.configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData)->configurationValue;

            if (USB_DEVICE_EndpointIsEnabled(appData.usbDevHandle, appData.endpointRx) == false )
            {
                /* Enable Read Endpoint */
                result = USB_DEVICE_EndpointEnable(appData.usbDevHandle, 0, appData.endpointRx,
                        USB_TRANSFER_TYPE_BULK, 64);
                if (result != USB_DEVICE_RESULT_OK)
                {
                    WriteSerialChar('9');
                    BSP_LED_1On();
                }
                
                appData.epDataReadEnabled = true;
                WriteSerialChar('R');
            }
            // Prime Receive End Point
                result = USB_DEVICE_EndpointRead(appData.usbDevHandle, &appData.readTransferHandle,
            //            appData.endpointRx, &receivedDataBuffer[0], sizeof(receivedDataBuffer) );
                appData.endpointRx, &receivedDataBuffer[0], 64 );
                if (result != USB_DEVICE_RESULT_OK)
                {
                    WriteSerialChar('8');
                    BSP_LED_1On();
                }    
            appData.epDataReadPending = true;
#ifdef WriteN            
            if (USB_DEVICE_EndpointIsEnabled(appData.usbDevHandle, appData.endpointTx) == false )
            {
                /* Enable Write Endpoint */
            //    BSP_LED_3On();
                result = USB_DEVICE_EndpointEnable(appData.usbDevHandle, 0, appData.endpointTx,
                        USB_TRANSFER_TYPE_BULK, 64);
            if (result != USB_DEVICE_RESULT_OK)
                {
                WriteSerialChar('7');
                    BSP_LED_1On();
                }
                WriteSerialChar('T');
            }
            appData.epDataWriteEnabled = true;
            appData.usbState = USB_IDLE;
#endif                  
            break;
            
        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
            WriteSerialChar('S');
            /* This means we have received a setup packet */
            setupPacket = (USB_SETUP_PACKET *)eventData;
            if(setupPacket->bRequest == USB_REQUEST_SET_INTERFACE)
            {
                /* If we have got the SET_INTERFACE request, we just acknowledge
                 for now. This demo has only one alternate setting which is already
                 active. */
                USB_DEVICE_ControlStatus(appData.usbDevHandle,USB_DEVICE_CONTROL_STATUS_OK);
            }
            else if(setupPacket->bRequest == USB_REQUEST_GET_INTERFACE)
            {
                /* We have only one alternate setting and this setting 0. So
                 * we send this information to the host. */

                USB_DEVICE_ControlSend(appData.usbDevHandle, &appData.altSetting, 1);
            }
            else
            {
                /* We have received a request that we cannot handle. Stall it*/
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            }
            break;
            
        case USB_DEVICE_EVENT_SUSPENDED:
            BSP_LED_3On();
            WriteSerialChar('U');
            break;
            
        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach (appData.usbDevHandle);
            WriteSerialChar('P');
            break;
            
        case USB_DEVICE_EVENT_POWER_REMOVED:
            WriteSerialChar('E');
            /* VBUS is not available */
            USB_DEVICE_Detach(appData.usbDevHandle);
            break;
            
        case USB_DEVICE_EVENT_ENDPOINT_READ_COMPLETE:
           /* Endpoint read is complete */
            ptr = eventData;
            len = ptr -> length;
            res = ptr -> status;
            WriteSerialChar('J');
            if (receivedDataBuffer[0] != 0x00 )
            {
                WriteSerialChar('N');
                WriteSerialChar(receivedDataBuffer[2]);
                WriteSerialChar('0'+ptr->length);
                WriteSysEx();
            }
            appData.epDataReadPending = false;
            ReadReceiveEP();
            break;
            
        case USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE:
            /* Endpoint write is complete */
            //BSP_LED_3Toggle();
        
            ptr = eventData;
            len = ptr -> length;
            res = ptr -> status;
            WriteSerialChar('K');
            appData.usbState = USB_WRITE_COMPLETE;
            appData.epDataWritePending = false;
            break;
            
        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
            WriteSerialChar('L');
            break;
            
        case USB_DEVICE_EVENT_ERROR:
            WriteSerialChar('6');
            BSP_LED_1On();
        
        default:
          //  WriteSerialChar('Z');
            WriteSerialChar('A'+ event);
            break;
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{

    /* Place the App state machines in there initial state. */
    appData.serialState     = SERIAL_STATE_INIT;
    appData.spiState        = SPI_STATE_INIT;
    appData.timerState      = TIMER_STATE_INIT;
    appData.usbState        = USB_STATE_INIT;
    appData.keybrdState     = KEYBRD_STATE_INIT;
    appData.heartbeatTimer  = DRV_HANDLE_INVALID;
    appData.heartbeatCount  = 0;
    appData.heartbeatToggle = false;

    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceIsConfigured = false;
    appData.epDataReadPending  = false;
    appData.epDataWritePending = false;
    appData.epDataReadEnabled  = false;
    appData.epDataWriteEnabled = false;
    appData.noteOn = false;
    appData.midiNote = 0;
    appData.altSetting = 0;
    appData.configurationValue = 0;
    appData.endpointRx = 0x01;
    appData.endpointTx = 0x01 | USB_EP_DIRECTION_IN;
    
    /* Keep SS lines inactive */
    //APP_SPI_CS_DESELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
    //APP_SPI_CS_SELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
    
    initStack();
}


    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */



/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
  
    USB_DEVICE_RESULT result;
    
    /* Signal the application's heartbeat. */
    if (appData.heartbeatToggle == true)
    {
        //SYS_PORTS_PinToggle(PORTS_ID_0, APP_HEARTBEAT_PORT,
        //                    APP_HEARTBEAT_PIN);
        //BSP_LED_1Toggle();
        // make sure the transmit buffer is not full before trying to write byte 
        //if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART_TransferStatus(myUSARTHandle)) )
        //    {
        //    appData.tx_byte = 'A';
        //    DRV_USART_WriteByte(myUSARTHandle, appData.tx_byte);  // send modified byte
        //   BSP_DelayUs(50000);
        //        appData.state = APP_STATE_RX;       // change state to RX and wait for next received byte
        //    }
        appData.heartbeatToggle = false;
        
 //       if(DRV_SPI_BufferStatus(appData.spiBufferHandle) & DRV_SPI_BUFFER_EVENT_COMPLETE)
 //       {
 //           WriteSerialChar('b');
 //       }
#ifdef Write        
        if (appData.epDataWriteEnabled == true && appData.epDataWritePending == false )
        {
            if (appData.noteOn == false)
            {
        //        BSP_LED_3Toggle();
                WriteMIDI(0x09,72,124);
                appData.noteOn = true;
            }
            else
            {
        //        BSP_LED_2Toggle();
                WriteMIDI(0x08,72,0);
                appData.noteOn = false;
            }    
        }
#endif 
#ifdef Read        
        if (appData.epDataReadEnabled == true && appData.epDataReadPending == false )
            {
            result = USB_DEVICE_EndpointRead(appData.usbDevHandle, &appData.readTransferHandle,
                        appData.endpointRx, &receivedDataBuffer[0], sizeof(receivedDataBuffer) );
            if (result != USB_DEVICE_RESULT_OK)
                {
                BSP_LED_1On();
                }
            WriteSerialChar('R');
            appData.epDataReadPending = true;
        }
#endif        
    }
    
    // Timer state Machine
    switch (appData.timerState)
    {
        case TIMER_STATE_INIT:
            BSP_LED_1Off();
            appData.heartbeatTimer = DRV_TMR_Open( APP_HEARTBEAT_TMR,
                DRV_IO_INTENT_EXCLUSIVE);
            if ( DRV_HANDLE_INVALID != appData.heartbeatTimer )
            {
                DRV_TMR_AlarmRegister(appData.heartbeatTimer,
                APP_HEARTBEAT_TMR_PERIOD,
                APP_HEARTBEAT_TMR_IS_PERIODIC,
                (uintptr_t)&appData,
                APP_TimerCallback);
                DRV_TMR_Start(appData.heartbeatTimer);
                appData.timerState = TIMER_RUNNING;
            }
            
        case TIMER_RUNNING:
            break;

    }
    
    // Serial State Machine
    switch (appData.serialState)
        {
        case SERIAL_STATE_INIT:
            appData.serialDevHandle = DRV_USART_Open(DRV_USART_INDEX_0,DRV_IO_INTENT_WRITE|DRV_IO_INTENT_BLOCKING);
            if (appData.serialDevHandle != DRV_HANDLE_INVALID)
               {
               // Set Buffer handler (Null context)
               DRV_USART_BufferEventHandlerSet(appData.serialDevHandle,
                    APP_USARTBufferEventHandler, NULL); 
               }
            appData.serialState = SERIAL_IDLE;
            break;
        
        case SERIAL_TRANSMITING:
            break;                 
                
        case SERIAL_IDLE:
             break;
        }
    
    // USB state machine
    switch (appData.usbState)
        {
        case USB_STATE_INIT:
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);
            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                appData.usbState = USB_WAITING_FOR_CONFIGURATION;
            }
            break;
        
        case USB_WRITE_COMMAND_IN_PROCESS:
            // NoteOn or NoteOff command in process
            // Waiting for write complete in eventhandler
            break;
            
        case USB_WRITE_COMPLETE:        // Combine with USB IDLE ?????
            // If note on stack pop note request and action
            // If nothing on note stack go to USB Idle state
            
        case USB_IDLE:    
            break;
            
        case USB_WAITING_FOR_CONFIGURATION:
            break;
           
        }
    
    // SPI State machine tasks
    switch (appData.spiState)
        {
        case SPI_STATE_INIT:
            appData.spiDevHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,DRV_IO_INTENT_READ);
            if (appData.spiDevHandle !=(uintptr_t) NULL)
               {
               // Turn continuous conversion on
               APP_SPI_CS_DESELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
               appData.spiState = SPI_START_CONVERSION;   
               }
            else
               {
               BSP_LED_1On();
               }
            break;
            
        case SPI_START_CONVERSION:
            APP_SPI_CS_SELECT(SPI_SLAVE_1_CS_PORT_ID,SPI_SLAVE_1_CS_PORT_PIN);
            appData.spiState = SPI_WAIT_DATA_READY;
            break;
            
        case SPI_WAIT_DATA_READY:
            if (0 == PORTDbits.RD3) // If conversion complete read values
               {
               appData.spiBufferHandle = DRV_SPI_BufferAddRead(appData.spiDevHandle,
                       &(appData.spiBuffer[0]), 3, APP_SPIEventHandler, NULL );
               appData.spiState = SPI_READING;
               }
            break;
            
        case SPI_READING:
            break;
        }
    
// Keyboard State Machine
    switch (appData.keybrdState)
        {
        case KEYBRD_STATE_INIT:
            appData.keybrdState = KEYBRD_NOKEY;
            break;
            
        case KEYBRD_NOKEY:
            break;
        
        case KEYBRD_KEYPRESS:
            break;
        }
               
}



 

/*******************************************************************************
 End of File
 */
