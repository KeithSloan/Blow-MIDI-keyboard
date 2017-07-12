/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits, 
    and allocates any necessary global system resources, such as the 
    sysObj structure that contains the object handles to all the MPLAB Harmony 
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include "system_config.h"
#include "system_definitions.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config PWP =        OFF
#pragma config BWP =        OFF
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      PRIPLL
#pragma config FSOSCEN =    OFF
#pragma config IESO =       ON
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FPBDIV =     DIV_2
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     OFF
#pragma config FWDTWINSZ =  WINSZ_25
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_5
#pragma config FPLLMUL =    MUL_20
#pragma config FPLLODIV =   DIV_1
#pragma config UPLLIDIV =   DIV_5
#pragma config UPLLEN =     ON
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FSRSSEL =    PRIORITY_7
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON
#pragma config FVBUSONIO =  ON
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_SPI Initialization Data"> 
 /*** SPI Driver Initialization Data ***/
  /*** Index 0  ***/
 DRV_SPI_INIT drvSpi0InitData =
 {
    .spiId = DRV_SPI_SPI_ID_IDX0,
    .taskMode = DRV_SPI_TASK_MODE_IDX0,
    .spiMode = DRV_SPI_SPI_MODE_IDX0,
    .allowIdleRun = DRV_SPI_ALLOW_IDLE_RUN_IDX0,
    .spiProtocolType = DRV_SPI_SPI_PROTOCOL_TYPE_IDX0,
    .commWidth = DRV_SPI_COMM_WIDTH_IDX0,
    .baudClockSource = DRV_SPI_CLOCK_SOURCE_IDX0,
    .spiClk = DRV_SPI_SPI_CLOCK_IDX0,
    .baudRate = DRV_SPI_BAUD_RATE_IDX0,
    .bufferType = DRV_SPI_BUFFER_TYPE_IDX0,
    .clockMode = DRV_SPI_CLOCK_MODE_IDX0,
    .inputSamplePhase = DRV_SPI_INPUT_PHASE_IDX0,
    .txInterruptSource = DRV_SPI_TX_INT_SOURCE_IDX0,
    .rxInterruptSource = DRV_SPI_RX_INT_SOURCE_IDX0,
    .errInterruptSource = DRV_SPI_ERROR_INT_SOURCE_IDX0,
    .queueSize = DRV_SPI_QUEUE_SIZE_IDX0,
    .jobQueueReserveSize = DRV_SPI_RESERVED_JOB_IDX0,
 };
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_Timer Initialization Data">
/*** TMR Driver Initialization Data ***/

const DRV_TMR_INIT drvTmr0InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX0,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX0,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX0, 
    .prescale = DRV_TMR_PRESCALE_IDX0,
    .mode = DRV_TMR_OPERATION_MODE_16_BIT,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX0,
    .asyncWriteEnable = false,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USART Initialization Data">

const DRV_USART_INIT drvUsart0InitData =
{
    .moduleInit.value = DRV_USART_POWER_STATE_IDX0,
    .usartID = DRV_USART_PERIPHERAL_ID_IDX0, 
    .mode = DRV_USART_OPER_MODE_IDX0,
    .flags = DRV_USART_INIT_FLAGS_IDX0,
    .brgClock = DRV_USART_BRG_CLOCK_IDX0,
    .lineControl = DRV_USART_LINE_CNTRL_IDX0,
    .baud = DRV_USART_BAUD_RATE_IDX0,
    .handshake = DRV_USART_HANDSHAKE_MODE_IDX0,
    .interruptTransmit = DRV_USART_XMIT_INT_SRC_IDX0,
    .interruptReceive = DRV_USART_RCV_INT_SRC_IDX0,
    .interruptError = DRV_USART_ERR_INT_SRC_IDX0,
    .queueSizeTransmit = DRV_USART_XMIT_QUEUE_SIZE_IDX0,
    .queueSizeReceive = DRV_USART_RCV_QUEUE_SIZE_IDX0,
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
    .dmaInterruptTransmit = DRV_USART_XMIT_INT_SRC_IDX0,    
    .dmaChannelReceive = DMA_CHANNEL_NONE,
    .dmaInterruptReceive = DRV_USART_RCV_INT_SRC_IDX0,    
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">
/******************************************************
 * USB Driver Initialization
 ******************************************************/
/****************************************************
 * Endpoint Table needed by the Device Layer.
 ****************************************************/
uint8_t __attribute__((aligned(512))) endPointTable[DRV_USBFS_ENDPOINTS_NUMBER * 32];
const DRV_USBFS_INIT drvUSBInit =
{
    /* Assign the endpoint table */
    .endpointTable= endPointTable,

    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    .operationMode = DRV_USBFS_OPMODE_DEVICE,
    
    .operationSpeed = USB_SPEED_FULL,
    
    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USB_ID_1
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************
//<editor-fold defaultstate="collapsed" desc="SYS_DEVCON Initialization Data">
/*******************************************************************************
  Device Control System Service Initialization Data
*/

const SYS_DEVCON_INIT sysDevconInit =
{
    .moduleInit = {0},
};

// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="USB Stack Initialization Data">


/**************************************************
 * USB Device Function Driver Init Data
 **************************************************/
/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1] =
{
    /* Function 1 */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 0,       /* First interfaceNumber of this function */ 
        .speed = USB_SPEED_FULL,    /* Function Speed */ 
        .numberOfInterfaces = 1,    /* Number of interfaces */
        .funcDriverIndex = 0,  /* Index of Vendor Driver */
        .driver = NULL,            /* No Function Driver data */ 
        .funcDriverInit = NULL     /* No Function Driver Init data */
    },
};

/*******************************************
 * USB Device Layer Descriptors
 *******************************************/
/*******************************************
 *  USB Device Descriptor 
 *******************************************/
const USB_DEVICE_DESCRIPTOR deviceDescriptor =
{
    0x12,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,          // DEVICE descriptor type
    0x0200,                         // USB Spec Release Number in BCD format
    0x00,                           // Class Code
    0x00,                           // Subclass code
    0x00,                           // Protocol code
    USB_DEVICE_EP0_BUFFER_SIZE,     // Max packet size for EP0, see system_config.h
    0x0763,                         // Vendor ID
    0x202D,                         // Product ID
    0x0100,                         // Device release number in BCD format
    0x01,                           // Manufacturer string index
    0x02,                           // Product string index
    0x00,                           // Device serial number string index
    0x01                            // Number of possible configurations
};


/*******************************************
 *  USB Full Speed Configuration Descriptor
 *******************************************/
const uint8_t fullSpeedConfigurationDescriptor[]=
{
    /* Configuration Descriptor */

    0x09,                                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                       // Descriptor Type
    0x91,0,                                             //(32 Bytes)Size of the Config descriptor.e
    0x02,                                               // Number of interfaces in this cfg
    0x01,                                               // Index value of this configuration
    0x00,                                               // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED, // Attributes
    50,                                                 // Max power consumption (2X mA)
    /* Descriptor for Function 1 - Vendor     */ 
    
    /* Interface Descriptor */
/* Interface Descriptor */
    0x09,//sizeof(USB_INTF_DSC), // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE, // INTERFACE descriptor type
    0, // Interface Number
    0, // Alternate Setting Number
    0, // Number of endpoints in this intf
    1, // Class code AUDIO
    1, // Subclass code AUDIO_CONTROL
    0, // Protocol code
    0, // Interface string index

    /* MIDI Adapter Class-specific AC Interface Descriptor */
    0x09, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x01, //bDescriptorSubtype - HEADER
    0x00,0x01, //bcdADC
    0x09,0x00, //wTotalLength
    0x01, //bInCollection
    0x01, //baInterfaceNr(1)

    /* MIDI Adapter Standard MS Interface Descriptor */
    0x09, //bLength
    0x04, //bDescriptorType
    0x01, //bInterfaceNumber
    0x00, //bAlternateSetting
    0x02, //bNumEndpoints
    0x01, //bInterfaceClass
    0x03, //bInterfaceSubclass
    0x00, //bInterfaceProtocol
    0x00, //iInterface
    
    /* Endpoint Descriptor 1 */
    /* MIDI Adapter Class-specific MS Interface Descriptor */
    
    0x07, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x01, //bDescriptorSubtype - MS_HEADER
    0x00,0x01, //BcdADC
    0x52,0x00, //wTotalLength
    
    /* MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
    0x06, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x02, //bDescriptorSubtype - MIDI_IN_JACK
    0x01, //bJackType - EMBEDDED
    0x01, //bJackID
    0x06, //iJack
    
    /* MIDI Adapter MIDI IN Jack Descriptor (External) */
    0x06, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x02, //bDescriptorSubtype - MIDI_IN_JACK
    0x02, //bJackType - EXTERNAL
    0x02, //bJackID
    0x00, //iJack
    
    /* MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
    0x09, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x03, //bDescriptorSubtype - MIDI_OUT_JACK
    0x01, //bJackType - EMBEDDED
    0x03, //bJackID
    0x01, //bNrInputPins
    0x02, //BaSourceID(1)
    0x01, //BaSourcePin(1)
    0x03, //iJack
    
    /* MIDI Adapter MIDI OUT Jack Descriptor (External) */
    0x09, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x03, //bDescriptorSubtype - MIDI_OUT_JACK
    0x02, //bJackType - EXTERNAL
    0x04, //bJackID
    0x01, //bNrInputPins
    0x01, //BaSourceID(1)
    0x01, //BaSourcePin(1)
    0x00, //iJack
    
    /* MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
    0x06, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x02, //bDescriptorSubtype - MIDI_IN_JACK
    0x01, //bJackType - EMBEDDED
    0x05, //bJackID
    0x07, //iJack
    
    /* MIDI Adapter MIDI IN Jack Descriptor (External) */
    0x06, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x02, //bDescriptorSubtype - MIDI_IN_JACK
    0x02, //bJackType - EXTERNAL
    0x06, //bJackID
    0x00, //iJack
    
    /* MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
    0x09, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x03, //bDescriptorSubtype - MIDI_OUT_JACK
    0x01, //bJackType - EMBEDDED
    0x07, //bJackID
    0x01, //bNrInputPins
    0x06, //BaSourceID(1)
    0x01, //BaSourcePin(1)
    0x04, //iJack
    
    /* MIDI Adapter MIDI OUT Jack Descriptor (External) */
    0x09, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x03, //bDescriptorSubtype - MIDI_OUT_JACK
    0x02, //bJackType - EXTERNAL
    0x08, //bJackID
    0x01, //bNrInputPins
    0x05, //BaSourceID(1)
    0x01, //BaSourePin(1)
    0x00, //iJack
    
    /* MIDI Adapter MIDI IN Jack Descriptor (External) */
    0x06, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x02, //bDescriptorSubtype - MIDI_IN_JACK
    0x02, //bJackType - EXTERNAL
    0x0A, //bJackID
    0x00, //iJack
    
/* MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
    0x09, //bLength
    0x24, //bDescriptorType - CS_INTERFACE
    0x03, //bDescriptorSubtype - MIDI_OUT_JACK
    0x01, //bJackType - EMBEDDED
    0x0B, //bJackID
    0x01, //bNrInputPins
    0x0A, //BaSourceID(1)
    0x01, //BaSourcePin(1)
    0x05, //iJack

    0x07, //bLength
    0x05, //bDescriptorType - ENDPOINT
    0x02, //bEndpointAddress - OUT
 //   0x01,
    0x02,0x40, //wMaxPacketSize
    //0x00,0x04,
    0x00,  //bInterval
    0x00, //bRefresh
    
    /* MIDI Adapter Class-specific Bulk OUT Endpoint Descriptor */
    0x06, //bLength
    0x25, //bDescriptorType - CS_ENDPOINT
    0x01, //bDescriptorSubtype - MS_GENERAL
    0x02, //bNumEmbMIDIJack
    0x01, //
    0x05, //
    
    0x07, //bLength
    0x05, //bDescriptorType - ENDPOINT
    0x81, //bEndpointAddress - In
    0x02,0x40, //wMaxPacketSize
    //0x00,0x04,
    0x00,  //bInterval
    0x00, //bRefresh
    
    /* MIDI Adapter Class-specific Bulk OUT Endpoint Descriptor */
    0x07, //bLength
    0x25, //bDescriptorType - CS_ENDPOINT
    0x01, //bDescriptorSubtype - MS_GENERAL
    0x03, //bNumEmbMIDIJack
    0x03, //
    0x07, //
    0x0B
};

/**************************************
 *  String descriptors.
 *************************************/

 /*******************************************
 *  Language code string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex; //Index of the string descriptor
        uint16_t languageID ; // Language ID of this string.
        uint8_t bLength; // Size of this descriptor in bytes
        uint8_t bDscType; // STRING descriptor type 
        uint16_t string[1]; // String
    }
    sd000 =
    {
        0, // Index of this string is 0
        0, // This field is always blank for String Index 0
        sizeof(sd000)-sizeof(sd000.stringIndex)-sizeof(sd000.languageID),
        USB_DESCRIPTOR_STRING,
        {0x0409} // Language ID
    };
/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex; //Index of the string descriptor
        uint16_t languageID ; // Language ID of this string.
        uint8_t bLength; // Size of this descriptor in bytes
        uint8_t bDscType; // STRING descriptor type
        uint16_t string[7]; // String
    }
    sd001 =
    {
        1, // Index of this string descriptor is 1. 
        0x0409, // Language ID of this string descriptor is 0x0409 (English)
        sizeof(sd001)-sizeof(sd001.stringIndex)-sizeof(sd001.languageID),
        USB_DESCRIPTOR_STRING,
        {'C','u','r','l','S','y','s'}
    };

/*******************************************
 *  Product string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex; //Index of the string descriptor
        uint16_t languageID ; // Language ID of this string.
        uint8_t bLength; // Size of this descriptor in bytes
        uint8_t bDscType; // STRING descriptor type
        uint16_t string[8]; // String
    }
    sd002 =
    {
        2, // Index of this string descriptor is 2. 
        0x0409, // Language ID of this string descriptor is 0x0409 (English)
        sizeof(sd002)-sizeof(sd002.stringIndex)-sizeof(sd002.languageID),
        USB_DESCRIPTOR_STRING,
  {'K','K','K','S','y','n','t','h'}
    }; 

/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[3]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002
};


/*******************************************
 * Array of Full speed config descriptors
 *******************************************/
USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    fullSpeedConfigurationDescriptor
};

#ifdef OLD
/**************************************
 *  String descriptors.
 *************************************/

 /*******************************************
 *  Language code string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;   // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type 
        uint16_t string[1];     // String
    }
    sd000 =
    {
        0, // Index of this string is 0
        0, // This field is always blank for String Index 0
        sizeof(sd000)-sizeof(sd000.stringIndex)-sizeof(sd000.languageID),
        USB_DESCRIPTOR_STRING,
        {0x0409}                // Language ID
    };  
/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;    // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[0];    // String
    }
    sd001 =
    {
        1,      // Index of this string descriptor is 1. 
        0x0409, // Language ID of this string descriptor is 0x0409 (English)
        sizeof(sd001)-sizeof(sd001.stringIndex)-sizeof(sd001.languageID),
        USB_DESCRIPTOR_STRING,
    };

/*******************************************
 *  Product string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;   // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type 
        uint16_t string[0];    // String
    }
    sd002 =
    {
        2,       // Index of this string descriptor is 2. 
        0x0409,  // Language ID of this string descriptor is 0x0409 (English)
        sizeof(sd002)-sizeof(sd002.stringIndex)-sizeof(sd002.languageID),
        USB_DESCRIPTOR_STRING,
    }; 

/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[3]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002
};

#endif
/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    &deviceDescriptor,          /* Full speed descriptor */
    1,                          /* Total number of full speed configurations available */
    fullSpeedConfigDescSet,     /* Pointer to array of full speed configurations descriptors*/
    NULL, 
    0, 
    NULL, 
    3,                          // Total number of string descriptors available.
    stringDescriptors,          // Pointer to array of string descriptors.
    NULL, 
    NULL
};


/****************************************************
 * USB Device Layer Initialization Data
 ****************************************************/
const USB_DEVICE_INIT usbDevInitData =
{
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Number of function drivers registered to this instance of the
       USB device layer */
    .registeredFuncCount = 1,
    
    /* Function driver table registered to this instance of the USB device layer*/
    .registeredFunctions = (USB_DEVICE_FUNCTION_REGISTRATION_TABLE*)funcRegistrationTable,

    /* Pointer to USB Descriptor structure */
    .usbMasterDescriptor = (USB_DEVICE_MASTER_DESCRIPTOR*)&usbMasterDescriptor,

    /* USB Device Speed */
    .deviceSpeed = USB_SPEED_FULL,
    
    /* Index of the USB Driver to be used by this Device Layer Instance */
    .driverIndex = DRV_USBFS_INDEX_0,

    /* Pointer to the USB Driver Functions. */
    .usbDriverInterface = DRV_USBFS_DEVICE_INTERFACE,
    
    /* Specify queue size for vendor endpoint read */
    .queueSizeEndpointRead = 1,
    
    /* Specify queue size for vendor endpoint write */
    .queueSizeEndpointWrite= 1,
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Static Initialization Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&sysDevconInit);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_PORTS_Initialize();
    /* Board Support Package Initialization */
    BSP_Initialize();        

    /* Initialize Drivers */

    /*** SPI Driver Index 0 initialization***/

    SYS_INT_VectorPrioritySet(DRV_SPI_INT_VECTOR_IDX0, DRV_SPI_INT_PRIORITY_IDX0);
    SYS_INT_VectorSubprioritySet(DRV_SPI_INT_VECTOR_IDX0, DRV_SPI_INT_SUB_PRIORITY_IDX0);
    sysObj.spiObjectIdx0 = DRV_SPI_Initialize(DRV_SPI_INDEX_0, (const SYS_MODULE_INIT  * const)&drvSpi0InitData);

    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);

    SYS_INT_VectorPrioritySet(INT_VECTOR_T1, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0);
 
 
     sysObj.drvUsart0 = DRV_USART_Initialize(DRV_USART_INDEX_0, (SYS_MODULE_INIT *)&drvUsart0InitData);
    /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBFS_Initialize(DRV_USBFS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBInit);

    /* Initialize System Services */

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();
  
    /* Initialize Middleware */

    /* Initialize the USB device layer */
    sysObj.usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

