/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ws2812b_control.c

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

#include "ws2812b_control.h"

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

WS2812B_CONTROL_DATA ws2812b_controlData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void WS2812B_CONTROL_Initialize ( void )

  Remarks:
    See prototype in ws2812b_control.h.
 */

void WS2812B_CONTROL_Initialize(void)
{
    /* Place the App state machine in its initial state. */
    ws2812b_controlData.state = WS2812B_CONTROL_STATE_INIT;


    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void WS2812B_CONTROL_Tasks ( void )

  Remarks:
    See prototype in ws2812b_control.h.
 */

void WS2812B_CONTROL_Tasks(void)
{

    /* Check the application's current state. */
    switch (ws2812b_controlData.state)
    {
        /* Application's initial state. */
    case WS2812B_CONTROL_STATE_INIT:
    {
        int i;
        ws2812b_controlData.update_timer = SYS_TMR_DelayMS(100);
        ws2812b_controlData.state = WS2812B_CONTROL_STATE_SERVICE_TASKS;
        for (i = 0; i < NB_LEDS; i++)
        {
            ws2812b_controlData.led_colors[i].B = 255;
            ws2812b_controlData.led_colors[i].R = 0;
            ws2812b_controlData.led_colors[i].G = 0;
        }
        break;
    }

    case WS2812B_CONTROL_STATE_SERVICE_TASKS:
    {
        if (SYS_TMR_DelayStatusGet(ws2812b_controlData.update_timer) == true)
        {
            int i, j;

            ws2812b_controlData.update_timer = SYS_TMR_DelayMS(33);
            for(i = 0; i < NB_LEDS; i++)
            {
                for(j = 23; j >= 0; j--)
                {
                    LEDS_CONTROLOn();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    if(ws2812b_controlData.led_colors[i].color & (1 << j))
                    {
                        LEDS_CONTROLOn();
                    }
                    else
                    {
                        LEDS_CONTROLOff();
                    }
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    LEDS_CONTROLOff();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                    Nop();
                }
            }
        }
        break;
    }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
    default:
    {
        /* TODO: Handle error in application's state machine. */
        break;
    }
    }
}



/*******************************************************************************
 End of File
 */
