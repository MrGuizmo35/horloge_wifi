/*******************************************************************************
  SPI Driver Functions for Static Enhanced Buffer Driver Tasks Functions

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_static_ebm_tasks.c

  Summary:
    SPI driver tasks functions

  Description:
    The SPI device driver provides a simple interface to manage the SPI
    modules on Microchip microcontrollers. This file contains implemenation
    for the SPI driver.

  Remarks:
  This file is generated from framework/driver/spi/template/drv_spi_static_ebm_tasks.c.ftl
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#include "system_config.h"
#include "system_definitions.h"
#include "driver/spi/static/src/drv_spi_static_local.h"

int32_t DRV_SPI0_SlaveEBMSend8BitISR( struct DRV_SPI_OBJ * pDrvObj )
{
    register DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;
    if (currentJob == NULL)
    {
        return 0;
    }
    /* determine the maximum number of bytes that can be transmitted */
    uint8_t bufferBytes = PLIB_SPI_TX_8BIT_FIFO_SIZE(SPI_ID_1) - PLIB_SPI_FIFOCountGet(SPI_ID_1, SPI_FIFO_TYPE_TRANSMIT);
    size_t dataUnits = MIN(currentJob->dataLeftToTx, bufferBytes);
    currentJob->dataLeftToTx -= dataUnits;


    /* Check to see if we'll run out of data*/
    if (currentJob->dataLeftToTx < bufferBytes)
    {
        PLIB_SPI_FIFOInterruptModeSelect(SPI_ID_1, SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_COMPLETELY_EMPTY);
        SYS_INT_SourceDisable(INT_SOURCE_SPI_1_TRANSMIT);
        SYS_INT_SourceEnable(INT_SOURCE_SPI_1_RECEIVE);
    }

    /* Set the buffer location of where to start transmitting from*/
    uint8_t *bufferLoc = &(currentJob->txBuffer[currentJob->dataTxed]);
    size_t counter;
    for (counter = 0; counter < dataUnits; counter++)
    {
        /* Add the data to the buffer */
        PLIB_SPI_BufferWrite(SPI_ID_1, bufferLoc[counter]);
    }

    currentJob->dataTxed += dataUnits;

    SYS_INT_SourceStatusClear(INT_SOURCE_SPI_1_TRANSMIT);
    return 0;

}

int32_t DRV_SPI0_SlaveEBMReceive8BitISR( struct DRV_SPI_OBJ * pDrvObj )
{
    register DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;

    if (currentJob->dataLeftToRx + currentJob->dummyLeftToRx == 0)
    {
        return 0;
    }

    uint8_t bufferBytes = PLIB_SPI_FIFOCountGet(SPI_ID_1, SPI_FIFO_TYPE_RECEIVE);
    size_t dataUnits = MIN(currentJob->dataLeftToRx, bufferBytes);
    bufferBytes -= dataUnits;
    currentJob->dataLeftToRx -= dataUnits;
    size_t dummyUnits = MIN(currentJob->dummyLeftToRx, bufferBytes);
    currentJob->dummyLeftToRx -= dummyUnits;

    uint8_t *bufferLoc = &(currentJob->rxBuffer[currentJob->dataRxed]);
    size_t counter;
    for (counter = 0; counter < dataUnits; counter++)
    {
        bufferLoc[counter] = PLIB_SPI_BufferRead(SPI_ID_1);
    }

    for (counter = 0; counter < dummyUnits; counter++)
    {
        PLIB_SPI_BufferRead(SPI_ID_1);
    }
    currentJob->dataRxed += dataUnits;

    if (currentJob->dataLeftToRx + currentJob->dummyLeftToRx < PLIB_SPI_RX_8BIT_HW_MARK(SPI_ID_1))
    {
        PLIB_SPI_FIFOInterruptModeSelect(SPI_ID_1, SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_NOT_EMPTY);
    }
    if (currentJob->dataLeftToRx + currentJob->dummyLeftToRx == 0)
    {
        SYS_INT_SourceDisable(INT_SOURCE_SPI_1_RECEIVE);
        SYS_INT_SourceEnable(INT_SOURCE_SPI_1_TRANSMIT);
    }
    SYS_INT_SourceStatusClear(INT_SOURCE_SPI_1_RECEIVE);
    return 0;
}
