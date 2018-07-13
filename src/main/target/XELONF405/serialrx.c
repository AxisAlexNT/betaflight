/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "io/serial.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/targetcustomserial.h"





static rxRuntimeConfig_t *rxRuntimeConfigPtr;
static serialPort_t *serialPort;

#define SUPPORTED_CHANNEL_COUNT (4 + 10)
static uint32_t channelData[SUPPORTED_CHANNEL_COUNT];
static bool rcFrameComplete = false;



#ifndef UART_MYPORT_RX_BAUDRATE
    #define UART_MYPORT_RX_BAUDRATE (115200)   //Скорость
    #warning "!!!FOR MYPROTO MPORT: UART baudrate ISN'T set in target.h of selected platform, setting it to 115200!!!"
#endif



typedef enum
{
    none,
    recv_cmd,
    error
} rxProtoState;

typedef enum
{
    ch_set,
    get_len,
    data_byte,
    fin_even_byte,
    fin_odd_byte
} command_types;


static rxProtoState rxState;
static command_types current_cmd;


static uint8_t cmd = 0;
static uint8_t dat = 0;


// Receive ISR callback
static void dataReceive(uint16_t c, void *data) //Это -- чистый коллбэк, он используется при создании порта (см. ниже) и вызывается, когда поступают данные
{
    UNUSED(data);

    //Окей, НСНМ нам поступил байт c, что с ним делать:

    cmd = (c >> 8);
    dat = (c & 0b0000000011111111);

    switch(cmd)
    {
      case 0b00100010:  
        rxState = recv_cmd;
        current_cmd = ch_set;
        break;

      default:
        //We've recieved strange command
        rxState = error;
        break;
    }



}

static uint8_t frameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (!rcFrameComplete) {
        return RX_FRAME_PENDING;
    }

    // Set rcFrameComplete to false so we don't process this one twice
    rcFrameComplete = false;

    return RX_FRAME_COMPLETE;
}

static uint16_t readRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)   //Эту функцию вызывает программа управления, она должна вернуть значение на канале
{
    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }
    return channelData[chan];
}


bool targetCustomSerialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfigPtr = rxRuntimeConfig;

    if (rxConfig->serialrx_provider != SERIALRX_TARGET_CUSTOM)
    {
        return false;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    rxRuntimeConfig->channelCount = SUPPORTED_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = UART_MYPORT_RX_BAUDRATE; // 20000 -- Value taken from rx_spi.c (NRF24 is being used downstream)
    rxRuntimeConfig->rcReadRawFn = readRawRC;
    rxRuntimeConfig->rcFrameStatusFn = frameStatus;

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        dataReceive,    //Вот эту функцию будут вызывать при поступлении нового байта
        NULL,
        UART_MYPORT_RX_BAUDRATE,
        MODE_RX,
        SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO
        );


    rxState = (serialPort != NULL) ? start1 : stperr;

    return serialPort != NULL;
}
