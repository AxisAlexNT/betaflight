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
static serialPort_t *debugSerialPort;

#define SUPPORTED_CHANNEL_COUNT (4 + 10)
static uint32_t channelData[SUPPORTED_CHANNEL_COUNT] = { 1111, 1222, 1333, 1444, 1555, 1666, 1777, 1888};
static bool rcFrameComplete = true;

static uint32_t readbuffer[8] = { 0 };
static uint32_t ch_n, cnt, iter, tmp, cur_d, tm_ch = 0;

static uint32_t cnt_tst = 1000;

/*
#ifndef UART_MYPORT_RX_BAUDRATE
    #define UART_MYPORT_RX_BAUDRATE (BAUD_115200)   //Скорость
    #warning "!!!FOR MYPROTO MPORT: UART baudrate ISN'T set in target.h of selected platform, setting it to 115200!!!"
#endif
*/


typedef enum
{
    none,
    wait_for_start2,
    started,
    recv_cmd,
    error
} rxProtoState;

typedef enum
{
    start1            =   0b10111011,     //
    start2            =   0b11101110,     //
    ch_set            =   0b00100010,     // 00100010 <Channel Number> -- Indicates that we will be changing channel value
    get_len           =   0b00110011,     // 00110011 <N> -- Indicates that number of N bytes will be transfered
    data_byte_even_n  =   0b10001000,     // 10001000 <D> -- A part of big number is sent, this part should contain even number of 'one's (like 1010)
    data_byte_odd_n   =   0b10011001,     // 10011001 <D> -- A part of big number is sent, this part should contain odd number of 'one's (like 10101)
    data_byte         =   0b11001100,     // 11001100 <B> -- Just recieve a byte B
    fin_byte          =   0b11011101      // 11011101 XXXXXXXX -- Все байты числа переданы
} command_types;


static rxProtoState rxState;
static command_types current_cmd;


static uint8_t cmd = 0;
static uint8_t dat = 0;


// Receive ISR callback
static void dataReceive(uint16_t c, void *data) //Это -- чистый коллбэк, он используется при создании порта (см. ниже) и вызывается, когда поступают данные
{
    UNUSED(data);

    serialPrint(debugSerialPort, 'NEW DATA');

    serialWrite(debugSerialPort, c);


    //Окей, НСНМ нам поступил байт c, что с ним делать:

    cmd = (c >> 8);
    dat = (c & 0b0000000011111111);


    if ((rxState == none) || (rxState == error))
    {
      if (cmd == start1)
      {
        rxState = wait_for_start2;
      }
    }else if (rxState == wait_for_start2){
      if (cmd == start2)
      {
        rxState = started;
        rcFrameComplete = true;
      }else{
        rxState = error;
      }
    }else if (rxState == started){
      switch(cmd)
      {
        case ch_set:
          rxState = recv_cmd;
          current_cmd = ch_set;
          ch_n = dat;
          //rcFrameComplete = false;
          break;

        default:
          //We've recieved strange command
          rxState = error;
          break;
      }
    }else if (rxState == recv_cmd){
        switch (current_cmd) {
            case set_ch:

            switch (cmd)
            {
                case get_len:
                    iter = 0;
                    cnt = dat;
                    break;

                case data_byte_even_n:
                    if (iter < cnt)
                    {
                        cur_d = 0;
                        tmp = dat;

                        while (tmp > 0)
                        {
                            cur_d += tmp & 0b1;
                            tmp = (tmp >> 1);
                        }

                        if (cur_d & 0b1 == 0) //means that cur_d is even as the last bit is zero
                        {
                            tm_ch = (tm_ch << 8) | dat;
                        }else{
                            rxState = error;
                        }

                    }else{
                        rxState = error;
                    }

                break;


            case data_byte_odd_n:
                  if (iter < cnt)
                  {
                      cur_d = 0;
                      tmp = dat;

                      while (tmp > 0)
                      {
                          cur_d += tmp & 0b1;
                          tmp = (tmp >> 1);
                      }

                      if (cur_d & 0b1 == 1) //means that cur_d is even as the last bit is zero
                      {
                          tm_ch = (tm_ch << 8) | dat;
                      }else{
                          rxState = error;
                      }

                  }else{
                      rxState = error;
                  }

                  break;

            case fin_byte:
                channelData[ch_n] = tm_ch;
                tm_ch = 0;
                rcFrameComplete = true;

            default:
                break;
            }
          break;

        default:
          break;
      }
    }

}

static uint8_t frameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);
    // cnt_tst++;
    //
    // if (!rcFrameComplete) {
    //
    //   if (cnt_tst > 1999)
    //   {
    //     cnt_tst = 1000;
    //   }
    //     return RX_FRAME_PENDING;
    // }

    // Set rcFrameComplete to false so we don't process this one twice
    rcFrameComplete = false;

    return RX_FRAME_COMPLETE;
}

static uint16_t readRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)   //Эту функцию вызывает программа управления, она должна вернуть значение на канале
{
    if (chan >= rxRuntimeConfig->channelCount) {
        return 999;
    }

    /*
    cnt_tst++;

    if (cnt_tst > 1999)
      {
        cnt_tst = 1000;
      }

    if (cnt_tst % 150 == 0) channelData[3] = cnt_tst;
    //channelData[1] = 1234;
    //channelData[0] = 1524;
    */
    return channelData[chan];
}


static void tdcf(uint16_t c, void *data) {}


bool targetCustomSerialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfigPtr = rxRuntimeConfig;

    //if (rxConfig->serialrx_provider != SERIALRX_TARGET_CUSTOM)
    //{
    //    return false;
    //}

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
       return false;
    }

    rxRuntimeConfig->channelCount = 8;
    rxRuntimeConfig->rxRefreshRate = 200; // 20000 -- Value taken from rx_spi.c (NRF24 is being used downstream)
    rxRuntimeConfig->rcReadRawFn = readRawRC;
    rxRuntimeConfig->rcFrameStatusFn = frameStatus;

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        dataReceive,    //Вот эту функцию будут вызывать при поступлении нового байта
        NULL,
        BAUD_115200, //UART_MYPORT_RX_BAUDRATE,
        MODE_RX,
        SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO
        );


    rxState = (serialPort != NULL) ? none : error;

    //rxState = none;

    return serialPort != NULL;
    //return true;
}
