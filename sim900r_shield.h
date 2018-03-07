/*
 * sim900r_shield.h
 * A library based on copy of library for SeeedStudio seeeduino GPRS shield 
 *
 * Copyright (c) 2015 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : lawliet zou
 * Create Time: April 2015
 * Change Log :
 *
 * The MIT License (MIT)
  *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __SIM900R_SHIELD_H__
#define __SIM900R_SHIELD_H__

#include "sim900r.h"

enum Protocol {
    CLOSED = 0,
    TCP    = 1,
    UDP    = 2,
};
const char OK[]   = "OK\r\n";

class GPRS
{
public:
    GPRS(Stream& serial, uint8_t pkPin = 2, uint8_t stPin = 3);

    static GPRS* getInstance() {
        return inst;
    };

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                               POWER                                     ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

    void  powerOn(void);
    void  powerOff(void);
    void  powerSwitch(void);
    bool  isPowerOn(void);


///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                              INITIALIZATION                             ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

    uint8_t isReady(void);
    uint8_t init(void);
    uint8_t init(char* ipv4Buf);
    uint8_t initialSetting(void);


///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                               TOOLS                                     ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

  char* getImei(char* imei);
  char* getDateTime(char* buffer);             // Получить время с часов модуля
  uint8_t GPRS::readBalance(const char* moneyRequestBuf,
                                  char* moneyBalanceBuf, 
                                  int   bufLen,
                                  int&  moneyBalanceInt);


///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                                SMS                                      ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

    bool sendSMS(char *message, char *phone);
    bool readSMS(char *message, char *phone);
    bool deleteSMS(int index);



//////////////////////////////////////////////////////
/// GPRS
//////////////////////////////////////////////////////
   /**  Connect the GPRS module to the network.
     *  @return true if connected, false otherwise
     */

//    bool join(const __FlashStringHelper *apn = 0, const __FlashStringHelper *userName = 0, const __FlashStringHelper *passWord = 0);
     bool join(char* apn = 0, char* = 0, char* = 0, int timeout = 2 * DEFAULT_TIMEOUT);

    /** Disconnect the GPRS module from the network
     *  @returns
     */
    void disconnect(void);

    /** Open a tcp/udp connection with the specified host on the specified port
     *  @param socket an endpoint of an inter-process communication flow of GPRS module,for SIM900 module, it is in [0,6]
     *  @param ptl protocol for socket, TCP/UDP can be choosen
     *  @param host host (can be either an ip address or a name. If a name is provided, a dns request will be established)
     *  @param port port
     *  @param timeout wait seconds till connected
     *  @returns true if successful
     */
    bool connect(Protocol ptl, const char * host, int port, int timeout = 2 * DEFAULT_TIMEOUT);
    bool connect(Protocol ptl, const __FlashStringHelper *host, const __FlashStringHelper *port, int timeout = 2 * DEFAULT_TIMEOUT);

    /** Check if a tcp link is active
     *  @returns true if successful
     */
    bool is_connected(void);

    /** Close a tcp connection
     *  @returns true if successful
     */
    bool close(void);

    /** check if GPRS module is readable or not
     *  @returns true if readable
     */
    int readable(void);

    /** wait a few time to check if GPRS module is readable or not
     *  @param socket socket
     *  @param wait_time time of waiting
     */
    int wait_readable(int wait_time);

    /** wait a few time to check if GPRS module is writeable or not
     *  @param socket socket
     *  @param wait_time time of waiting
     */
    int wait_writeable(int req_size);

    int send(const char * str);
    /** send data to socket
     *  @param socket socket
     *  @param str string to be sent
     *  @param len string length
     *  @returns return bytes that actually been send
     */
    int send(const char * str, int len);

    /** read data from socket
     *  @param socket socket
     *  @param buf buffer that will store the data read from socket
     *  @param len string length need to read from socket
     *  @returns bytes that actually read
     */
    int recv(char* buf, int len);

    /** convert the host to ip
     *  @param host host ip string, ex. 10.11.12.13
     *  @param ip long int ip address, ex. 0x11223344
     *  @returns true if successful
     */
    //NOT USED bool gethostbyname(const char* host, uint32_t* ip);

    char* getIPAddress();
    unsigned long getIPnumber();

private:
    uint8_t _stPin = 3;
    uint8_t _pkPin = 2;

    uint32_t str_to_ip(const char* str);
//    SoftwareSerial gprsSerial;
    Stream* stream;
    static GPRS* inst;
    uint32_t _ip;
    char ip_string[16]; //XXX.YYY.ZZZ.WWW + \0
};
#endif
