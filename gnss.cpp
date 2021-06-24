/* mbed Microcontroller Library
 * Copyright (c) 2017 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file gnss.cpp
 * This file defines a class that communicates with a u-blox GNSS chip.
 */

#include "mbed.h"
#include "ctype.h"
#include "gnss.h"
#include "mbed_thread.h"
#include <stdio.h>

#ifdef UBLOX_WEARABLE_FRAMEWORK
#include "SDCardModel.h"
#else
#define SEND_LOGGING_MESSAGE printf
#endif

GnssParser::GnssParser(void)
{
    // Create the enable pin but set everything to disabled
    _gnssEnable = NULL;

#ifdef TARGET_UBLOX_C030
    _gnssEnable = new DigitalInOut(GNSSEN, PIN_OUTPUT, PushPullNoPull, 0);
#else
    _gnssEnable = new DigitalInOut(GNSSEN, PIN_OUTPUT, PullNone, 1);
#endif
}

GnssParser::~GnssParser(void)
{
    if (_gnssEnable != NULL) {
        *_gnssEnable = 0;
        delete _gnssEnable;
    }
}

void GnssParser::powerOff(void)
{
    // Set the GNSS into backup mode using the command RMX-LPREQ
    struct {
        unsigned long dur;
        unsigned long flags;
    } msg = {0 /*endless*/,0 /*backup*/};
    sendUbx(0x02, 0x41, &msg, sizeof(msg));
}

void GnssParser::cutOffPower(void)
{
    // Disabling PA15 to cut off power supply
    if (_gnssEnable != NULL)
        *_gnssEnable = 0;
    thread_sleep_for(1);
}

void GnssParser::_powerOn(void)
{
    if (_gnssEnable != NULL) {
        *_gnssEnable = 1;
    }
    thread_sleep_for(1);
}

int GnssParser::_getMessage(Pipe<char>* pipe, char* buf, int len)
{
    int unkn = 0;
    int sz = pipe->size();
    int fr = pipe->free();
    if (len > sz)
        len = sz;
    while (len > 0)
    {
        // NMEA protocol
        pipe->set(unkn);
        int nmea = _parseNmea(pipe,len);
        if ((nmea != NOT_FOUND) && (unkn > 0))
            return UNKNOWN | pipe->get(buf,unkn);
        if (nmea == WAIT && fr)
            return WAIT;
        if (nmea > 0)
            return NMEA | pipe->get(buf,nmea);
        // UBX protocol

        pipe->set(unkn);
        int ubx = _parseUbx(pipe,len);
        if ((ubx != NOT_FOUND) && (unkn > 0))
            return UNKNOWN | pipe->get(buf,unkn);
        if (ubx == WAIT && fr)
            return WAIT;
        if (ubx > 0)
            return UBX | pipe->get(buf,ubx);

        // UNKNOWN
        unkn ++;
        len--;
    }
    if (unkn > 0)
        return UNKNOWN | pipe->get(buf,unkn);
    return WAIT;
}

int GnssParser::_parseNmea(Pipe<char>* pipe, int len)
{
    int o = 0;
    int c = 0;
    char ch;
    if (++o > len)                      return WAIT;
    if ('$' != pipe->next())            return NOT_FOUND;
    // This needs to be extended by crc checking
    for (;;)
    {
        if (++o > len)                  return WAIT;
        ch = pipe->next();
        if ('*' == ch)                  break; // crc delimiter
        if (!isprint(ch))               return NOT_FOUND;
        c ^= ch;
    }
    if (++o > len)                      return WAIT;
    ch = _toHex[(c >> 4) & 0xF]; // high nibble
    if (ch != pipe->next())             return NOT_FOUND;
    if (++o > len)                      return WAIT;
    ch = _toHex[(c >> 0) & 0xF]; // low nibble
    if (ch != pipe->next())             return NOT_FOUND;
    if (++o > len)                      return WAIT;
    if ('\r' != pipe->next())           return NOT_FOUND;
    if (++o > len)                      return WAIT;
    if ('\n' != pipe->next())           return NOT_FOUND;
    return o;
}

int GnssParser::_parseUbx(Pipe<char>* pipe, int l)
{
    int o = 0;
    if (++o > l)                return WAIT;
    if ('\xB5' != pipe->next()) return NOT_FOUND;
    if (++o > l)                return WAIT;
    if ('\x62' != pipe->next()) return NOT_FOUND;
    o += 4;
    if (o > l)                  return WAIT;
    int i,j,ca,cb;
    i = pipe->next();
    ca  = i;
    cb  = ca; // cls
    i = pipe->next();
    ca += i;
    cb += ca; // id
    i = pipe->next();
    ca += i;
    cb += ca; // len_lsb
    j = pipe->next();
    ca += j;
    cb += ca; // len_msb
    j = i + (j << 8);
    while (j--)
    {
        if (++o > l)            return WAIT;
        i = pipe->next();
        ca += i;
        cb += ca;
    }
    ca &= 0xFF;
    cb &= 0xFF;
    if (++o > l)                return WAIT;
    if (ca != pipe->next())     return NOT_FOUND;
    if (++o > l)                return WAIT;
    if (cb != pipe->next())     return NOT_FOUND;
    return o;
}

int GnssParser::send(const char* buf, int len)
{
    return _send(buf, len);
}

int GnssParser::sendNmea(const char* buf, int len)
{
    char head[1] = { '$' };
    char tail[5] = { '*', 0x00 /*crc_high*/, 0x00 /*crc_low*/, '\r', '\n' };
    int i;
    int crc = 0;
    for (i = 0; i < len; i ++)
        crc ^= *buf++;
    i  = _send(head, sizeof(head));
    i += _send(buf, len);
    tail[1] = _toHex[(crc > 4) & 0xF0];
    tail[2] = _toHex[(crc > 0) & 0x0F];
    i += _send(tail, sizeof(tail));
    return i;
}

int GnssParser::sendUbx(unsigned char cls, unsigned char id, const void* buf /*= NULL*/, int len /*= 0*/)
{
    char head[6] = { 0xB5, 0x62, cls, id, (char) len, (char) (len >> 8)};
    char crc[2];
    int i;
    int ca = 0;
    int cb = 0;
    for (i = 2; i < 6; i ++)
    {
        ca += head[i];
        cb += ca;
    }
    for (i = 0; i < len; i ++)
    {
        ca += ((char*)buf)[i];
        cb += ca;
    }
    i  = _send(head, sizeof(head));
    i += _send(buf, len);
    crc[0] = ca & 0xFF;
    crc[1] = cb & 0xFF;
    i += _send(crc,  sizeof(crc));
    return i;
}

const char* GnssParser::findNmeaItemPos(int ix, const char* start, const char* end)
{
    // Find the start
    for (; (start < end) && (ix > 0); start ++)
    {
        if (*start == ',')
            ix --;
    }
    // Found and check bounds
    if ((ix == 0) && (start < end) &&
            (*start != ',') && (*start != '*') && (*start != '\r') && (*start != '\n'))
        return start;
    else
        return NULL;
}

bool GnssParser::getNmeaItem(int ix, char* buf, int len, double& val)
{
    char* end = &buf[len];
    const char* pos = findNmeaItemPos(ix, buf, end);
    // Find the start
    if (!pos)
        return false;
    val = strtod(pos, &end);
    // Restore the last character
    return (end > pos);
}

bool GnssParser::getNmeaItem(int ix, char* buf, int len, int& val, int base /*=10*/)
{
    char* end = &buf[len];
    const char* pos = findNmeaItemPos(ix, buf, end);
    // Find the start
    if (!pos)
        return false;
    val = (int)strtol(pos, &end, base);
    return (end > pos);
}

bool GnssParser::getNmeaItem(int ix, char* buf, int len, char& val)
{
    const char* end = &buf[len];
    const char* pos = findNmeaItemPos(ix, buf, end);
    // Find the start
    if (!pos)
        return false;
    // Skip leading spaces
    while ((pos < end) && isspace(*pos))
        pos++;
    // Check bound
    if ((pos < end) &&
            (*pos != ',') && (*pos != '*') && (*pos != '\r') && (*pos != '\n'))
    {
        val = *pos;
        return true;
    }
    return false;
}

bool GnssParser::getNmeaAngle(int ix, char* buf, int len, double& val)
{
    char ch;
    if (getNmeaItem(ix,buf,len,val) && getNmeaItem(ix+1,buf,len,ch) &&
            ((ch == 'S') || (ch == 'N') || (ch == 'E') || (ch == 'W')))
    {
        val *= 0.01;
        int i = (int)val;
        val = (val - i) / 0.6 + i;
        if (ch == 'S' || ch == 'W')
            val = -val;
        return true;
    }
    return false;
}

int GnssParser::enable_ubx() {
    unsigned char ubx_cfg_prt[]= {
        // See https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
        0x01,                   // Port - 1
        0x00,                   // Reserved - 0
        0x00, 0x00,             // txReady - 0 thres, pin 0, pol 0, en 0
        0xC0, 0x08, 0x00, 0x00, // mode - 1 stop bit, no parity, 8 bit
        0x00, 0xC2, 0x01, 0x00, // baud - 115200
        0x23, 0x00,             // inProtoMask - Ubx, Nmea, Rtcm3
        0x03, 0x00,             // outProtoMask - Ubx, Nmea
        0x00, 0x00,             // flags
        0x00, 0x00              // reserved
    };
    int conf = RETRY;
    int length = 0;

    while(conf)
    {
        length = sendUbx(0x06, 0x00, ubx_cfg_prt, sizeof(ubx_cfg_prt));
        if(length >= (int)(sizeof(ubx_cfg_prt) + UBX_FRAME_SIZE))
        {
            thread_sleep_for(5000);
            break;
        }
        else
        {
            conf = conf - 1;
        }
    }
    return (conf == 0) ? 0 : 1;
}

eUBX_MESSAGE GnssParser::get_ubx_message(char *buff) {
    eUBX_MESSAGE return_value = UNKNOWN_UBX;

    if(buff[SYNC_CHAR_INDEX_1] == 0xB5 && buff[SYNC_CHAR_INDEX_2] == 0x62) {

        switch (buff[MSG_CLASS_INDEX]) {

        case NAV: {
            switch (buff[MSG_ID_INDEX]) {

            case 0x07: {
                return_value = UBX_NAV_PVT;
            }
            break;
            case 0x09: {
                return_value = UBX_NAV_ODO;
            }
            break;
            case 0x03: {
                return_value = UBX_NAV_STATUS;
            }
            break;
            case 0x35: {
                return_value = UBX_NAV_SAT;
            }
            break;
            default:
            {
                return_value = UNKNOWN_UBX;
            }
            break;
            }
        }
        break;
        case ACK: {
            switch (buff[MSG_ID_INDEX]) {
            case 0x00: {
                return_value = UBX_ACK_NAK;
            }
            break;
            case 0x01: {
                return_value = UBX_ACK_ACK;
            }
            break;
            default:
            {
                return_value = UNKNOWN_UBX;
            }
            break;
            }
        }
        break;
        case LOG: {
            switch (buff[MSG_ID_INDEX]) {
            case 0x11: {
                return_value = UBX_LOG_BATCH;
            }
            break;
            default:
            {
                return_value = UNKNOWN_UBX;
            }
            break;
            }
        }
        break;
        default:
        {
            return_value = UNKNOWN_UBX;
        }
        break;
        }
    }
    return return_value;
}

tUBX_ACK_ACK GnssParser::decode_ubx_cfg_ack_nak_msg(char *buf) {
    tUBX_ACK_ACK return_decoded_msg;
    uint8_t index = UBX_PAYLOAD_INDEX;

    return_decoded_msg.msg_class = buf[index++];
    return_decoded_msg.msg_id = buf[index];

    return return_decoded_msg;
}

tUBX_NAV_ODO GnssParser::decode_ubx_nav_odo_msg(char *buf) {
    tUBX_NAV_ODO return_decoded_msg;
    uint8_t index = UBX_PAYLOAD_INDEX;

    return_decoded_msg.version = buf[index++];
    index +=3; // 3 bytes are reserved

    return_decoded_msg.itow = buf[index++];
    return_decoded_msg.itow |= (buf[index++] << 8);
    return_decoded_msg.itow |= (buf[index++] << 16);
    return_decoded_msg.itow |= (buf[index++] << 24);

    return_decoded_msg.distance = buf[index++];
    return_decoded_msg.distance |= (buf[index++] << 8);
    return_decoded_msg.distance |= (buf[index++] << 16);
    return_decoded_msg.distance |= (buf[index++] << 24);

    return_decoded_msg.totalDistance = buf[index++];
    return_decoded_msg.totalDistance |= (buf[index++] << 8);
    return_decoded_msg.totalDistance |= (buf[index++] << 16);
    return_decoded_msg.totalDistance |= (buf[index++] << 24);

    return_decoded_msg.distanceSTD = buf[index++];
    return_decoded_msg.distanceSTD |= (buf[index++] << 8);
    return_decoded_msg.distanceSTD |= (buf[index++] << 16);
    return_decoded_msg.distanceSTD |= (buf[index++] << 24);

    return return_decoded_msg;
}

tUBX_NAV_PVT GnssParser::decode_ubx_nav_pvt_msg(char *buf) {
    tUBX_NAV_PVT return_decoded_msg;
    uint8_t index = UBX_PAYLOAD_INDEX;

    return_decoded_msg.itow = buf[index++];
    return_decoded_msg.itow |= (buf[index++] << 8);
    return_decoded_msg.itow |= (buf[index++] << 16);
    return_decoded_msg.itow |= (buf[index++] << 24);

    return_decoded_msg.year = buf[index++];
    return_decoded_msg.year |= (buf[index++] << 8);

    return_decoded_msg.month = buf[index++];

    return_decoded_msg.day = buf[index++];

    // Go to Fix type
    index = UBX_PAYLOAD_INDEX + 20;
    return_decoded_msg.fixType = buf[index++];
    return_decoded_msg.flag1 = buf[index];

    // Go to lon
    index = UBX_PAYLOAD_INDEX + 24;

    return_decoded_msg.lon = buf[index++];
    return_decoded_msg.lon |= (buf[index++] << 8);
    return_decoded_msg.lon |= (buf[index++] << 16);
    return_decoded_msg.lon |= (buf[index++] << 24);

    return_decoded_msg.lat = buf[index++];
    return_decoded_msg.lat |= (buf[index++] << 8);
    return_decoded_msg.lat |= (buf[index++] << 16);
    return_decoded_msg.lat |= (buf[index++] << 24);

    return_decoded_msg.height = buf[index++];
    return_decoded_msg.height |= (buf[index++] << 8);
    return_decoded_msg.height |= (buf[index++] << 16);
    return_decoded_msg.height |= (buf[index++] << 24);

    // Go to gSpeed
    index = UBX_PAYLOAD_INDEX + 60;
    return_decoded_msg.speed = buf[index++];
    return_decoded_msg.speed |= (buf[index++] << 8);
    return_decoded_msg.speed |= (buf[index++] << 16);
    return_decoded_msg.speed |= (buf[index++] << 24);

    return return_decoded_msg;
}

tUBX_LOG_BATCH GnssParser::decode_ubx_log_batch_msg(char *buf) {
    tUBX_LOG_BATCH return_decoded_msg;
    uint8_t index = UBX_PAYLOAD_INDEX;

    // move index itow
    index = UBX_PAYLOAD_INDEX + 4;

    return_decoded_msg.itow = buf[index++];
    return_decoded_msg.itow |= (buf[index++] << 8);
    return_decoded_msg.itow |= (buf[index++] << 16);
    return_decoded_msg.itow |= (buf[index++] << 24);

    // move index lon
    index = UBX_PAYLOAD_INDEX + 24;

    return_decoded_msg.lon = buf[index++];
    return_decoded_msg.lon |= (buf[index++] << 8);
    return_decoded_msg.lon |= (buf[index++] << 16);
    return_decoded_msg.lon |= (buf[index++] << 24);

    return_decoded_msg.lat = buf[index++];
    return_decoded_msg.lat |= (buf[index++] << 8);
    return_decoded_msg.lat |= (buf[index++] << 16);
    return_decoded_msg.lat |= (buf[index++] << 24);

    return_decoded_msg.height = buf[index++];
    return_decoded_msg.height |= (buf[index++] << 8);
    return_decoded_msg.height |= (buf[index++] << 16);
    return_decoded_msg.height |= (buf[index++] << 24);

    // move index to distance
    index = UBX_PAYLOAD_INDEX + 84;

    return_decoded_msg.distance = buf[index++];
    return_decoded_msg.distance |= (buf[index++] << 8);
    return_decoded_msg.distance |= (buf[index++] << 16);
    return_decoded_msg.distance |= (buf[index++] << 24);

    return_decoded_msg.totalDistance = buf[index++];
    return_decoded_msg.totalDistance |= (buf[index++] << 8);
    return_decoded_msg.totalDistance |= (buf[index++] << 16);
    return_decoded_msg.totalDistance |= (buf[index++] << 24);

    return_decoded_msg.distanceSTD = buf[index++];
    return_decoded_msg.distanceSTD |= (buf[index++] << 8);
    return_decoded_msg.distanceSTD |= (buf[index++] << 16);
    return_decoded_msg.distanceSTD |= (buf[index++] << 24);

    return return_decoded_msg;
}

tUBX_NAV_STATUS GnssParser::decode_ubx_nav_status_msg(char *buf) {

    tUBX_NAV_STATUS return_decoded_msg;
    uint8_t index = UBX_PAYLOAD_INDEX;

    return_decoded_msg.itow = buf[index++];
    return_decoded_msg.itow |= (buf[index++] << 8);
    return_decoded_msg.itow |= (buf[index++] << 16);
    return_decoded_msg.itow |= (buf[index++] << 24);

    // move index flag
    return_decoded_msg.fix = buf[index++];

    return_decoded_msg.flags = buf[index++];

    // move to ttff
    index+=2;

    return_decoded_msg.ttff = buf[index++];
    return_decoded_msg.ttff |= (buf[index++] << 8);
    return_decoded_msg.ttff |= (buf[index++] << 16);
    return_decoded_msg.ttff |= (buf[index++] << 24);

    return_decoded_msg.msss = buf[index++];
    return_decoded_msg.msss |= (buf[index++] << 8);
    return_decoded_msg.msss |= (buf[index++] << 16);
    return_decoded_msg.msss |= (buf[index++] << 24);

    return return_decoded_msg;
}


tUBX_NAV_SAT GnssParser::decode_ubx_nav_sat_msg(char *buf, int length) {
    tUBX_NAV_SAT return_decoded_msg;
    uint8_t index = UBX_PAYLOAD_INDEX;
    uint8_t numberSVs = buf[index + 5];

    if(length == (UBX_FRAME_SIZE + 8 + (12*numberSVs))) {
        return_decoded_msg.status = true;
    }
    else {
        return_decoded_msg.status = false;
    }

    return return_decoded_msg;
}

int GnssParser::ubx_request_batched_data(bool sendMonFirst) {
    unsigned char ubx_log_retrieve_batch[]= {0x00, 0x00, 0x00, 0x00};

    ubx_log_retrieve_batch[1] = (sendMonFirst == true) ? 0x01 : 0x00;

    int conf = RETRY;
    while(conf)
    {

        int length = sendUbx(0x21, 0x10, ubx_log_retrieve_batch, sizeof(ubx_log_retrieve_batch));
        if(length >= (int)(sizeof(ubx_log_retrieve_batch) + UBX_FRAME_SIZE))
        {
            thread_sleep_for(1000);
            break;
        }
        else
        {
            conf = conf - 1;
        }
    }
    if(conf == 0)
    {
        return 1;
    }

    return 0;
}

const char GnssParser::_toHex[] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };

// ----------------------------------------------------------------
// Serial Implementation
// ----------------------------------------------------------------

GnssSerial::GnssSerial(PinName tx /*= GNSSTXD  */, PinName rx /*= GNSSRXD */, int baudrate /*= GNSSBAUD */,
                       int rxSize /*= 256 */, int txSize /*= 128 */) :
    SerialPipe(tx, rx, baudrate, rxSize, txSize)
{
    baud(baudrate);
}

GnssSerial::~GnssSerial(void)
{
    powerOff();
}

bool GnssSerial::init(PinName pn)
{
    Timer timer;
    int size;

    // Unused (kept only for compatibility with the I2C version)
    (void)pn;

    // Power up and enable the module
    _powerOn();

    // Send a byte to wakup the device again
    putc(0xFF);
    // Wait until we get some bytes
    size = _pipeRx.size();

    timer.start();
    while ((timer.read_ms() < 1000) && (size == _pipeRx.size())) {
        // Nothing, just wait
    }
    timer.stop();

    thread_sleep_for(1000);

    enable_ubx();

    thread_sleep_for(1000);

    baud(115200);

    // Send a byte to wakup the device again
    putc(0xFF);

    // Wait until we get some bytes
    size = _pipeRx.size();

    timer.start();
    while ((timer.read_ms() < 1000) && (size == _pipeRx.size())) {
        // Nothing, just wait
    }

    return (size != _pipeRx.size());
}

int GnssSerial::getMessage(char* buf, int len)
{
    return _getMessage(&_pipeRx, buf, len);
}

int GnssSerial::_send(const void* buf, int len)
{
#ifdef UBLOX_WEARABLE_FRAMEWORK
    GET_SDCARD_INSTANCE->write(logging_file_name, (void *)buf, len);
#endif
    return put((const char*)buf, len, true /*=blocking*/);
}

// End Of File
