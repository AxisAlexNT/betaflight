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


//



static void routeIncommingPacket() //syslinkPacket_t* slp)
{
            // case CRTP_PORT_SETPOINT_GENERIC:
            //     // First byte of the packet is the type
            //     // Only support the CPPM Emulation type
            //     if (crtpPacket->data[0] == cppmEmuType) {
            //         crtpCommanderCPPMEmuPacket_t *crtpCppmPacket =
            //                 (crtpCommanderCPPMEmuPacket_t*)&crtpPacket->data[1];

            //         // Write RPYT channels in TAER order
            //         channelData[0] = crtpCppmPacket->channelThrust;
            //         channelData[1] = crtpCppmPacket->channelRoll;
            //         channelData[2] = crtpCppmPacket->channelPitch;
            //         channelData[3] = crtpCppmPacket->channelYaw;

            //         // Write the rest of the auxiliary channels
            //         uint8_t i;
            //         for (i = 0; i < crtpCppmPacket->hdr.numAuxChannels; i++) {
            //             channelData[i + 4] = crtpCppmPacket->channelAux[i];
            //         }
            //     }
            //     rcFrameComplete = true;
            //     break;
            // default:
            //     // Unsupported port - do nothing
            //     break;

}

// Receive ISR callback
static void dataReceive(uint16_t c, void *data) //Это -- чистый коллбэк, он используется при создании порта (см. ниже) и вызывается, когда поступают данные
{
    UNUSED(data);

    channelData[0] = c;

    
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
    rxRuntimeConfig->rxRefreshRate = 20000; // Value taken from rx_spi.c (NRF24 is being used downstream)
    rxRuntimeConfig->rcReadRawFn = readRawRC;
    rxRuntimeConfig->rcFrameStatusFn = frameStatus;

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        dataReceive,
        NULL,
        115200,
        MODE_RX,
        SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO
        );

    return serialPort != NULL;
}

