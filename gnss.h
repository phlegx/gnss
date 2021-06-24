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

#ifndef GNSS_H
#define GNSS_H

/**
 * @file gnss.h
 * This file defines a class that communicates with a u-blox GNSS chip.
 */

#include "mbed.h"
#include "pipe.h"
#include "serial_pipe.h"

#if defined (TARGET_UBLOX_C030) || defined (TARGET_UBLOX_C027)
# define GNSS_IF(onboard, shield) onboard
#else
# define GNSS_IF(onboard, shield) shield
#endif

#ifdef TARGET_UBLOX_C027
# define GNSSEN   GPSEN
# define GNSSTXD  GPSTXD
# define GNSSRXD  GPSRXD
# define GNSSBAUD GPSBAUD
#endif

#define UBX_FRAME_SIZE 8
#define RETRY 5
#define SYNC_CHAR_INDEX_1 0
#define SYNC_CHAR_INDEX_2 1
#define MSG_CLASS_INDEX 2
#define MSG_ID_INDEX 3
#define UBX_LENGTH_INDEX 4
#define UBX_PAYLOAD_INDEX 6

enum eUBX_MSG_CLASS {NAV = 0x01, ACK = 0x05, LOG = 0x21};

enum eUBX_MESSAGE  {UBX_LOG_BATCH, UBX_ACK_ACK, UBX_ACK_NAK, UBX_NAV_ODO, UBX_NAV_PVT, UBX_NAV_STATUS, UBX_NAV_SAT, UNKNOWN_UBX};

typedef struct UBX_ACK_ACK {
    uint8_t msg_class;
    uint8_t msg_id;

} tUBX_ACK_ACK;

typedef struct UBX_NAV_ODO {
    uint8_t version;
    uint8_t reserved[3];
    uint32_t itow;
    uint32_t distance;
    uint32_t totalDistance;
    uint32_t distanceSTD;
} tUBX_NAV_ODO;

typedef struct UBX_NAV_PVT {
    uint32_t itow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t fixType;
    uint8_t flag1; // gnssFixOK, diffSoln, psmState, headVehValid and carrSoln.
    int32_t lon; // scaling 1e-7
    int32_t lat; // scaling 1e-7
    int32_t height;
    int32_t speed;

} tUBX_NAV_PVT;

typedef struct UBX_LOG_BATCH {
    uint32_t itow;
    int32_t lon; // scaling 1e-7
    int32_t lat; // scaling 1e-7
    int32_t height;
    uint32_t distance;
    uint32_t totalDistance;
    uint32_t distanceSTD;

} tUBX_LOG_BATCH;

typedef struct UBX_CFG_BATCH {
    uint32_t version;
    uint8_t flags;
    uint32_t bufSize;
    uint32_t notifThrs;
    uint8_t pioId;
    uint8_t reserved1;

} tUBX_CFG_BATCH;

typedef struct UBX_NAV_STATUS {
    uint32_t itow;
    uint8_t fix;
    uint8_t flags;
    uint32_t ttff;
    uint32_t msss;

} tUBX_NAV_STATUS;

typedef struct UBX_NAV_SAT {
    bool status;

} tUBX_NAV_SAT;

/** Basic GNSS parser class.
*/
class GnssParser
{
public:
    /** Constructor.
     */
    GnssParser();
    /** Destructor.
     */
    virtual ~GnssParser(void);

    /** Power-on/wake-up the GNSS.
    */
    virtual bool init(PinName pn) = 0;

    enum {
        // getLine Responses
        WAIT      = -1, //!< wait for more incoming data (the start of a message was found, or no data available)
        NOT_FOUND =  0, //!< a parser concluded the the current offset of the pipe doe not contain a valid message

#define LENGTH(x)   (x & 0x00FFFF)  //!< extract/mask the length
#define PROTOCOL(x) (x & 0xFF0000)  //!< extract/mask the type

        UNKNOWN   = 0x000000,       //!< message type is unknown
        UBX       = 0x100000,       //!< message if of protocol NMEA
        NMEA      = 0x200000        //!< message if of protocol UBX
    };

    /** Get a line from the physical interface. This function
     * needs to be implemented in the inherited class.
     * @param buf the buffer to store it.
     * @param len size of the buffer.
     * @return type and length if something was found,
     *         WAIT if not enough data is available,
     *         NOT_FOUND if nothing was found
     */
    virtual int getMessage(char* buf, int len) = 0;

    /** Send a buffer.
     * @param buf the buffer to write.
     * @param len size of the buffer to write.
     * @return bytes written.
     */
    virtual int send(const char* buf, int len);

    /** send a NMEA message, this function just takes the
     * payload and calculates and adds checksum. ($ and *XX\r\n will be added).
     * @param buf the message payload to write.
     * @param len size of the message payload to write.
     * @return total bytes written.
     */
    virtual int sendNmea(const char* buf, int len);

    /** Send a UBX message, this function just takes the
     * payload and calculates and adds checksum.
     * @param cls the UBX class id.
     * @param id the UBX message id.
     * @param buf the message payload to write.
     * @param len size of the message payload to write.
     * @return total bytes written.
     */
    virtual int sendUbx(unsigned char cls, unsigned char id,
                        const void* buf = NULL, int len = 0);

    /** Power off the GNSS, it can be again woken up by an
     * edge on the serial port on the external interrupt pin.
    */
    void powerOff(void);

    /** Cuts off the power supply of GNSS by disabling gnssEnable pin
    * 	Backup supply is provided, can turn it on again by enabling PA15
    */
    void cutOffPower(void);

    /** get the first character of a NMEA field.
     * @param ix the index of the field to find.
     * @param start the start of the buffer.
     * @param end the end of the buffer.
     * @return the pointer to the first character of the field.
     */
    static const char* findNmeaItemPos(int ix, const char* start, const char* end);

    /** Extract a double value from a buffer containing a NMEA message.
     * @param ix the index of the field to extract.
     * @param buf the NMEA message.
     * @param len the size of the NMEA message.
     * @param val the extracted value.
     * @return true if successful, false otherwise.
     */
    static bool getNmeaItem(int ix, char* buf, int len, double& val);

    /** Extract a interger value from a buffer containing a NMEA message.
     * @param ix the index of the field to extract.
     * @param buf the NMEA message.
     * @param len the size of the NMEA message.
     * @param val the extracted value.
     * @param base the numeric base to be used (e.g. 8, 10 or 16).
     * @return true if successful, false otherwise.
     */
    static bool getNmeaItem(int ix, char* buf, int len, int& val, int base/*=10*/);

    /** Extract a char value from a buffer containing a NMEA message.
     * @param ix the index of the field to extract.
     * @param buf the NMEA message.
     * @param len the size of the NMEA message.
     * @param val the extracted value.
     * @return true if successful, false otherwise.
     */
    static bool getNmeaItem(int ix, char* buf, int len, char& val);

    /** Extract a latitude/longitude value from a buffer containing a NMEA message.
     * @param ix the index of the field to extract (will extract ix and ix + 1),
     * @param buf the NMEA message,
     * @param len the size of the NMEA message,
     * @param val the extracted latitude or longitude,
     * @return true if successful, false otherwise.
     */
    static bool getNmeaAngle(int ix, char* buf, int len, double& val);

    /** Enable UBX messages.
     * @param none
     * @return 1 if successful, false otherwise.
     */
    int enable_ubx();

    /** GET Message type of receiver UBX message
     * @param buff the UXB message
     * @return eUBX_MESSAGE
     */
    eUBX_MESSAGE get_ubx_message(char *);

    /** Method to parse contents of UBX ACK-ACK/NAK and return messageid amd class for which ACK is received
     * @param buff the UXB message
     * @return tUBX_ACK_ACK
     */
    tUBX_ACK_ACK decode_ubx_cfg_ack_nak_msg(char *);

    /** Method to parse contents of UBX_NAV_ODO and return decoded msg
     * @param buff the UXB message
     * @return tUBX_NAV_ODO
     */
    tUBX_NAV_ODO decode_ubx_nav_odo_msg(char *);

    /** Method to parse contents of UBX_NAV_PVT and return decoded msg
     * @param buff the UXB message
     * @return tUBX_NAV_PVT
     */
    tUBX_NAV_PVT decode_ubx_nav_pvt_msg(char *);

    /** Method to parse contents of UBX_LOG_BATCH and return decoded msg
     * @param buff the UXB message
     * @return tUBX_LOG_BATCH
     */
    tUBX_LOG_BATCH decode_ubx_log_batch_msg(char *);

    /** Method to parse contents of UBX_NAV_STATUS and return decoded msg
     * @param buff the UXB message
     * @return tUBX_NAV_STATUS
     */
    tUBX_NAV_STATUS decode_ubx_nav_status_msg(char *);

    /** Method to parse contents of UBX_NAV_SAT and return decoded msg
     * @param buff the UXB message, int length
     * @return tUBX_NAV_SAT
     */
    tUBX_NAV_SAT decode_ubx_nav_sat_msg(char *, int);

    /** Method to send UBX LOG-RETRIEVEBATCH msg. This message is used to request batched data.
     * @param bool
     * @return int
     */
    int ubx_request_batched_data(bool sendMonFirst = false);

protected:
    /** Power on the GNSS module.
    */
    void _powerOn(void);

    /** Get a line from the physical interface.
     * @param pipe the receiveing pipe to parse messages .
     * @param buf the buffer to store it.
     * @param len size of the buffer.
     * @return type and length if something was found,
     *         WAIT if not enough data is available,
     *         NOT_FOUND if nothing was found.
     */
    static int _getMessage(Pipe<char>* pipe, char* buf, int len);

    /** Check if the current offset of the pipe contains a NMEA message.
     * @param pipe the receiveing pipe to parse messages.
     * @param len numer of bytes to parse at maximum.
     * @return length if something was found (including the NMEA frame),
     *         WAIT if not enough data is available,
     *         NOT_FOUND if nothing was found.
     */
    static int _parseNmea(Pipe<char>* pipe, int len);

    /** Check if the current offset of the pipe contains a UBX message.
     * @param pipe the receiveing pipe to parse messages.
     * @param len numer of bytes to parse at maximum.
     * @return length if something was found (including the UBX frame),
     *         WAIT if not enough data is available,
     *         NOT_FOUND if nothing was found.
     */
    static int _parseUbx(Pipe<char>* pipe, int len);

    /** Write bytes to the physical interface. This function
     * needs to be implemented by the inherited class.
     * @param buf the buffer to write.
     * @param len size of the buffer to write.
     * @return bytes written.
     */
    virtual int _send(const void* buf, int len) = 0;

    static const char _toHex[16]; //!< num to hex conversion
    DigitalInOut *_gnssEnable;    //!< IO pin that enables GNSS
};

/** GNSS class which uses a serial port as physical interface.
 */
class GnssSerial : public SerialPipe, public GnssParser
{
public:
    /** Constructor.
     * @param tx is the serial ports transmit pin (GNSS to CPU).
     * @param rx is the serial ports receive pin (CPU to GNSS).
     * @param baudrate the baudrate of the GNSS use 9600.
     * @param rxSize the size of the serial rx buffer.
     * @param txSize the size of the serial tx buffer.
     */
    GnssSerial(PinName tx    GNSS_IF( = GNSSTXD, = D8 /* = D8 */), // resistor on shield not populated
               PinName rx    GNSS_IF( = GNSSRXD, = D9 /* = D9 */), // resistor on shield not populated
               int baudrate  GNSS_IF( = GNSSBAUD, = 9600 ),
               int rxSize    = 512,
               int txSize    = 512 );

    /** Destructor.
     */
    virtual ~GnssSerial(void);

    /** Initialise the GNSS device.
     * @param pn  NOT USED.
     * @param baudrate
     * @return true if successful, otherwise false.
     */
    virtual bool init(PinName pn = NC);

    /** Get a line from the physical interface.
     * @param buf the buffer to store it.
     * @param len size of the buffer.
     * @return type and length if something was found,
     *         WAIT if not enough data is available,
     *         NOT_FOUND if nothing was found.
     */
    virtual int getMessage(char* buf, int len);

protected:
    /** Write bytes to the physical interface.
     * @param buf the buffer to write.
     * @param len size of the buffer to write.
     * @return bytes written.
     */
    virtual int _send(const void* buf, int len);
};

/** GNSS class which uses a i2c as physical interface.
*/
class GnssI2C : public I2C, public GnssParser
{
public:
    /** Constructor.
     * @param sda is the I2C SDA pin (between CPU and GNSS).
     * @param scl is the I2C SCL pin (CPU to GNSS).
     * @param adr the I2C address of the GNSS set to (66<<1).
     * @param rxSize the size of the serial rx buffer.
     */
    GnssI2C(PinName sda          GNSS_IF( = NC, = /* D16 TODO */ NC ),
            PinName scl          GNSS_IF( = NC, = /* D17 TODO */ NC ),
            unsigned char i2cAdr GNSS_IF( = (66<<1), = (66<<1) ),
            int rxSize           = 256 );
    /** Destructor
     */
    virtual ~GnssI2C(void);

    /** Helper function to probe the i2c device.
     * @param pn  the power-on pin for the chip.
     * @return true if successfully detected the GNSS chip.
     */
    virtual bool init(PinName pn = GNSS_IF( NC, NC /* D7 resistor R67 on shield not mounted */));

    /** Get a line from the physical interface.
     * @param buf the buffer to store it.
     * @param len size of the buffer.
     * @return type and length if something was found,
     *         WAIT if not enough data is available,
     *         NOT_FOUND if nothing was found.
     */
    virtual int getMessage(char* buf, int len);

    /** Send a buffer.
     *  @param buf the buffer to write.
     *  @param len size of the buffer to write.
     *  @return bytes written.
     */
    virtual int send(const char* buf, int len);

    /** Send an NMEA message, this function just takes the
     * payload and calculates and adds checksum ($ and *XX\r\n will be added).
     * @param buf the message payload to write.
     * @param len size of the message payload to write.
     * @return total bytes written.
     */
    virtual int sendNmea(const char* buf, int len);

    /** Send a UBX message, this function just takes the
     * payload and calculates and adds checksum.
     * @param cls the UBX class id.
     * @param id the UBX message id.
     * @param buf the message payload to write.
     * @param len size of the message payload to write.
     * @return total bytes written.
     */
    virtual int sendUbx(unsigned char cls, unsigned char id,
                        const void* buf = NULL, int len = 0);

protected:
    /** Check if the port is writeable (like SerialPipe)
     * @return true if writeable
     */
    bool writeable(void) {
        return true;
    }

    /** Write a character (like SerialPipe).
     * @param c  the character to write.
     * @return true if succesffully written .
     */
    bool putc(int c) {
        char ch = c;
        return send(&ch, 1);
    }

    /** Write bytes to the physical interface.
     * @param buf the buffer to write.
     * @param len size of the buffer to write.
     * @return bytes written.
     */
    virtual int _send(const void* buf, int len);

    /** Read bytes from the physical interface.
     * @param buf the buffer to read into.
     * @param len size of the read buffer .
     * @return bytes read.
     */
    int _get(char* buf, int len);

    Pipe<char> _pipe;           //!< the rx pipe.
    unsigned char _i2cAdr;      //!< the i2c address.
    static const char REGLEN;   //!< the length i2c register address.
    static const char REGSTREAM;//!< the stream i2c register address.
};

#endif

// End Of File
