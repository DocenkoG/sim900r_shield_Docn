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
  unsigned char GPRS::syncNtp (const char* ntpServer);
  void read_buffer(char* buffer,int count);


///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                                SMS                                      ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

    bool sendSMS(char *message, char *phone);
    bool readSMS(char *message, char *phone);
    bool deleteSMS(int index);


///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                                GPRS                                     ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

unsigned char GPRS::getGprsStatus(char* ipv4Buf);
unsigned char GPRS::joinGprs(char* ipv4Buf);
int GPRS::httpGet(char* url, int&  dataLen);
//void GPRS::disconnectGprs()
//bool GPRS::connect(Protocol ptl, const char * host, int port, int timeout)
//bool GPRS::connect(Protocol ptl,const __FlashStringHelper *host, const __FlashStringHelper *port, int timeout)
//bool GPRS::close()
//int GPRS::readable(void)
//int GPRS::wait_readable(int wait_time)
//int GPRS::wait_writeable(int req_size)
//int GPRS::send(const char * str, int len)
//int GPRS::send(const char * str)


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
