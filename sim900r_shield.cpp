  /*
 * sim900r_shield.cpp
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

#include <stdio.h>
//#include <HardwareSerial.h>
#include "sim900r_shield.h"

GPRS* GPRS::inst;

GPRS::GPRS( Stream& serial, uint8_t pkPin, uint8_t stPin)
{
  _stPin = stPin;
  _pkPin = pkPin;

  inst = this;

  stream = &serial;
  sim900_init(stream);
  //SERIAL_PORT_HARDWARE.begin(baudRate);
  //(HardwareSerial&)serialPort.begin(baudRate);
  //(HardwareSerial*)serialPort.begin(baudRate);
  //(Stream*)serialPort.begin(baudRate);
  //serialPort.begin(baudRate);
  //*serialPort.begin(baudRate);
  //(Serial*)stream->begin(baudRate);
}


///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                               POWER                                     ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

void GPRS::powerOn(void) {         
  if(!digitalRead(_stPin)) {      // Если питание не подано,
    powerSwitch();                // то выполним стандартную последовательность сигналов 
  }      
}



void GPRS::powerOff(void) {       // Желательно корректно отключать питание, для сохранения переменных
  unsigned long t1 = millis();    // в памяти и для разрегистрации в сети мобильного оператора.
  sim900_send_cmd("AT+CPOWD=1\r\n"); // Даем команду на выключение
  while (millis()-t1 < 9000UL) {     // и в течение ХХХХ милисекунд джем
    if(!digitalRead(_stPin)) {       // произошло ли выключение.
      return;                        // Выходим, если отключилось быстрее.
    }
  }
  powerSwitch();                     // Иначе передёрним питание 
}



void GPRS::powerSwitch(void) {       // The same sequence is used for switching on and to power off
  pinMode(_pkPin, OUTPUT);
  digitalWrite(_pkPin, LOW);
  delay(1000);
  digitalWrite(_pkPin, HIGH);
  delay(2000);
  digitalWrite(_pkPin, LOW);
  delay(3000);
}



bool GPRS::isPowerOn(void)
{
  return digitalRead(_stPin);
}




///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                              INITIALIZATION                             ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

uint8_t GPRS::isReady(void){
  long t1 = millis();                 // фиксируем текущий момент
  while (millis()-t1 < 140L*1000L) {  // и в течение ХХХХ милисекунд джем готовности модуля
    sim900_flush_serial();
    if (sim900_check_with_cmd("AT+CPAS\r\n","+CPAS: 0",CMD)) { // периодически запрашивая состояние 
      /*
      0 – готов к работе
      2 – неизвестно
      3 – входящий звонок
      4 – голосовое соединение
      */
      return 0;                       // если достигнута готовность
    }
    delay(5000);                      // если не достигнута - пауза и повторный запрос
  }
  return 2;
}



uint8_t GPRS::init(void)
{
  uint8_t rc;
  if(0 == (rc = initialSetting())) {
    delay(5000);      // Пауза для завершения всех инициализационных процессов
  }
  return rc;
}


uint8_t GPRS::init(char* ipv4Buf) 
{
  uint8_t rc;
  if(0 == (rc = initialSetting())) {
/*
    if(1 == joinGprs(ipv4Buf)) {
      rc = 0;
    } else {
      rc = 11;      // нет GPRS, ip-адрес не получен
    }
*/
  }
  return rc;
}




uint8_t GPRS::initialSetting(void)
{
  uint8_t rc=0;
  if( 0 != isReady() )                                           rc += 1;
                     // а есть ли вообще модуль и способен ли он отвечать?
  if(!sim900_check_with_cmd("ATE0\r\n",OK,CMD))                  rc += 2;
                     // отключили эхо-ответ, тем самым уменьшив объем буферов
                     // для анализа ответа от модуля.
  if(!sim900_check_with_cmd("AT+CFUN=1\r\n",OK,CMD))             rc += 4;
  if (!sim900_check_with_cmd("AT+CNMI?\r\n", "+CNMI: 2,0,2,1,1",CMD)) {
    delay(100);
    sim900_flush_serial();
    if (!sim900_check_with_cmd("AT+CNMI=2,0,2,1,1\r\n",OK,CMD))  rc += 8;
                     // Установили режим, при котором SMS не лезут сразу в 
                     // serial, а сохраняются в памяти. Это удобно для последующего
                     // чтения и разбора SMS
  }
  if (!sim900_check_with_cmd("AT+CMGF?\r\n", "+CMGF: 1",CMD)) {
    if (!sim900_check_with_cmd("AT+CMGF=1\r\n",OK,CMD))          rc += 16;
  }
  if (!sim900_check_with_cmd("AT+CLIP=1\r\n",OK,CMD))            rc += 32;
  if (!sim900_check_with_cmd("AT+CSCS=\"GSM\"\r\n",OK,CMD))      rc += 64;
  return rc;
}



///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                               TOOLS                                     ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

char* GPRS::getImei(char* imei) 
{
  //                   --> CRLF       =  2
  //999999999999999    --> 15 + CRLF  = 17
  //                   --> CRLF       =  2
  //OK                 --> 2 + CRLF   =  4
  char tbuf[20];
  sim900_clean_buffer(tbuf,sizeof(tbuf));
  char *p, *s;
  int   i = 0;
  sim900_flush_serial();
  sim900_send_cmd("AT+GSN\r\n");
  sim900_read_buffer(tbuf, sizeof(tbuf)-1, DEFAULT_TIMEOUT);
  if(NULL != ( s = strstr(tbuf,"\r\n")+2 )) {
    if(NULL != ( p = strstr(s,"\r\n"))){
      p = s ;
      while((*p != '\r')&&(i < 15)) {
          imei[i++] = *(p++);
      }
    }
  }
  imei[i] = '\0';
  sim900_wait_for_resp(OK, CMD);
  return imei;
}



char* GPRS::getDateTime(char* buffer)
{
  //AT+CCLK?                       --> 8 + CRLF = 10
  //+CCLK: "14/11/13,21:14:41+04"  --> 29+ CRLF = 31
  //                               --> CRLF     =  2
  //OK                             -->          =  4

  byte i = 0;
  char gprsBuffer[40];
  char *p,*s;
  sim900_flush_serial();
  sim900_send_cmd("AT+CCLK?\r\n");
  sim900_clean_buffer(gprsBuffer,40);
  sim900_read_buffer(gprsBuffer,32,DEFAULT_TIMEOUT);
  if(NULL != ( s = strstr(gprsBuffer,"+CCLK:"))) {
    s = strstr((char *)(s),"\"");
    s = s + 1;  //We are in the first date-time character 
    p = strstr((char *)(s),"\"")-6; //p is last character minutes
    if (NULL != s) {
      i = 0;
      while (s < p) {
        buffer[i++] = *(s++);
      }
    }
  }  
  buffer[i] = '\0';            
  sim900_wait_for_resp(OK, CMD);
  return buffer;
}



uint8_t GPRS::readBalance(const char* moneyRequestBuf,
                                char* moneyBalanceBuf, 
                                int   bufLen,
                                int&  moneyBalanceInt)
{
  //AT+CUSD=1,"#100#"                                    = 19
  //                                                     =  2
  //                                                     =  2
  //OK                                          --> CRLF =  4
  //                                            --> CRLF =  2  
  //+CUSD: 0,"Balance:45,05r,Limit:0,01r ",64            = 41
  //                                                итого= 70
  byte i = 0;
  char gprsBuffer[70];
  char *p, *s;
  unsigned char rc;
  sim900_flush_serial();
  sim900_send_cmd("AT+CUSD=1,\"");
  sim900_send_cmd(moneyRequestBuf);
  sim900_check_with_cmd("\"\r\n",OK,DATA);
  sim900_clean_buffer(gprsBuffer,sizeof(gprsBuffer));
  sim900_read_buffer(gprsBuffer,sizeof(gprsBuffer)-1);
  //Serial.print(gprsBuffer);

  if(NULL != ( s = strstr(gprsBuffer,"CUSD:"))) {
    s = strstr((char *)(s),"\"");
    s = s + 1;                                     // We are in the first character 
    p = strstr((char *)(s),"r") + 1;
    if (NULL != s) {
      i = 0;
      while ((s < p) && (i < (bufLen -1))) {
        moneyBalanceBuf[i++] = *(s++);
      }
      moneyBalanceBuf[i] = '\0';            
    }     // Ответ получен. Теперь попробуем из него вытащить цифры баланса
    
    if(NULL != (s = strstr(moneyBalanceBuf,"Balance:"))) {       // Для МТС
      s = s + 8;
    } else {
      if(NULL != (s = strstr(moneyBalanceBuf,"balans"))) {       // Для Beeline
        s = s + 6;
      }
    }
    if(NULL != s) {
      p = strstr((char *)(s),"r");
      if (NULL != s) {
        i = 0;
        while (s < p) {
          gprsBuffer[i++] = *(s++);
        }
        gprsBuffer[i] = '\0';            
      }
      moneyBalanceInt = atoi(gprsBuffer);
      rc = 0;     // результат получен
    } else {
      rc = 2;     // в ответе нет баланса
    } 
  } else {
    rc = 4;       // не получен предсказуемый ответ
    i = 0;
    int n = min(sizeof(gprsBuffer), bufLen);
    while (i < n) {
      moneyBalanceBuf[i] = gprsBuffer[i];
      i += 1;
    }
    moneyBalanceBuf[i] = '\0';            
  }
  //sim900_wait_for_resp(OK, CMD);
  sim900_flush_serial();
  return rc;
}
  
///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                                SMS                                      ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

bool GPRS::sendSMS(char *message, char *phone)
{
  // Set message mode to ASCII
  if(!sim900_check_with_cmd("AT+CMGF=1\r\n", "OK\r\n", CMD)) {
    return false;
  }

  delay(500);
  sim900_send_cmd("AT+CMGS=\"");
  sim900_send_cmd(phone);

  if(!sim900_check_with_cmd("\"\r\n",">",CMD)) {
      return false;
  }

  delay(1000);
  sim900_send_cmd(message);
  delay(500);
  sim900_send_End_Mark();
  return true;
}



bool GPRS::readSMS(char *message, char *phone)
{
/* Чтение sms если существует и, и сразу его удаление. Иначе возвращает false

AT+CMGL="ALL"

+CMGL: 1,"REC READ","+79251857315","","18/03/05,19:45:33+12"
Test z sms k   For demon

OK
*/
    int i = 0, messageIndex;
    char gprsBuffer[111];
    char num[4];
    sim900_clean_buffer(gprsBuffer,sizeof(gprsBuffer));
    sim900_clean_buffer(num,sizeof(num));
    char *p, *s;
    sim900_send_cmd("AT+CMGL=\"ALL\"\r\n");
    sim900_read_buffer(gprsBuffer,sizeof(gprsBuffer)-1);
    //Serial.print("\n<");
    //Serial.print(gprsBuffer);
    //Serial.println(">");

    sim900_flush_serial();
    if(NULL != ( s = strstr(gprsBuffer,"+CMGL:"))) {
        p = s+6;
        while(*p != ',') {
            num[i++] = *(p++);
        }
        //
        s = strstr(p+2, ",");
        p = s+2;
        i = 0;
        while(*p != '\"') {
            phone[i++] = *(p++);
        }
        phone[i] = '\0';
        //
        s = strstr(p+2, "\r");
        p = s+2;
        i = 0;
        while((i<64)&&(*p != '\r')) {
            message[i++] = *(p++);
        }
        message[i] = '\0';
        //
        deleteSMS( atoi(num));
        return true;
    }
    message[0] = '\0';
    phone[0] = '\0';
    return false;
}



bool GPRS::deleteSMS(int index)
{
     char num[4];
     sim900_send_cmd("AT+CMGD=");
     itoa(index, num, 10);
     sim900_send_cmd(num);
     return sim900_check_with_cmd("\r\n",OK,CMD);
}







//Here is where we ask for APN configuration, with F() so we can save MEMORY
//bool GPRS::join(const __FlashStringHelper *apn, const __FlashStringHelper *userName, const __FlashStringHelper *passWord)
    bool GPRS::join(char* apn, char* userName, char* passWord, int timeout)
    {
     byte i;
     char *p, *s;
     char ipAddr[32];
/*    if(!sim900_check_with_cmd("AT+CIPSHUT\r\n","SHUT OK\r\n", CMD)) {
      Serial.write("Error = 1\r\n");
    return false;
    }
    delay(1000);
*/
    sim900_send_cmd("AT+CIPSHUT\r\n");
    delay(500);
    //Select multiple connection
    //sim900_check_with_cmd("AT+CIPMUX=1\r\n","OK",DEFAULT_TIMEOUT,CMD);

    //set APN. OLD VERSION
    //snprintf(cmd,sizeof(cmd),"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n",_apn,_userName,_passWord);
    //sim900_check_with_cmd(cmd, "OK\r\n", DEFAULT_TIMEOUT,CMD);
    sim900_send_cmd("AT+CSTT=\"");
    sim900_send_cmd(apn);
    sim900_send_cmd("\",\"");
    sim900_send_cmd(userName);
    sim900_send_cmd("\",\"");
    sim900_send_cmd(passWord);
    sim900_send_cmd("\"\r\n");
    delay(500);
    //Brings up wireless connection

    sim900_send_cmd("AT+CIICR\r\n");
    delay(4000);
    sim900_wait_for_resp("OK\r\n", CMD);
    delay(500);
//    sim900_check_with_cmd("AT+CIICR\r\n","OK\r\n", CMD);


    //Get local IP address
    sim900_send_cmd("AT+CIFSR\r\n");
    delay(500);
    sim900_clean_buffer(ipAddr,32);
    sim900_read_buffer(ipAddr,32,DEFAULT_TIMEOUT);

	//Response:
	//AT+CIFSR\r\n       -->  8 + 2
	//\r\n				 -->  0 + 2
	//10.160.57.120\r\n  --> 15 + 2 (max)   : TOTAL: 29
	//Response error:
	//AT+CIFSR\r\n
	//\r\n
	//ERROR\r\n
    if (NULL != strstr(ipAddr,"ERROR")) {
      Serial.write("Error = 2\r\n");
      return false;
    }
    s = ipAddr + 12;
    p = strstr((char *)(s),"\r\n"); //p is last character \r\n
    if (NULL != s) {
      i = 0;
      while (s < p) {
        ip_string[i++] = *(s++);
      }
      ip_string[i] = '\0';
    }
    _ip = str_to_ip(ip_string);
    if(_ip != 0) {

      return true;
    }
    Serial.write("Error = 3\r\n");
    return false;
  }

  void GPRS::disconnect()
  {
    sim900_send_cmd("AT+CIPSHUT\r\n");
  }

  bool GPRS::connect(Protocol ptl,const char * host, int port, int timeout)
  {
    //char cmd[64];
   char num[4];
   char resp[96];

    //sim900_clean_buffer(cmd,64);
   if(ptl == TCP) {
    sim900_send_cmd("AT+CIPSTART=\"TCP\",\"");
    sim900_send_cmd(host);
    sim900_send_cmd("\",");
    itoa(port, num, 10);
    sim900_send_cmd(num);
    sim900_send_cmd("\r\n");
//        sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n",host, port);
  } else if(ptl == UDP) {
    sim900_send_cmd("AT+CIPSTART=\"UDP\",\"");
    sim900_send_cmd(host);
    sim900_send_cmd("\",");
    itoa(port, num, 10);
    sim900_send_cmd(num);
    sim900_send_cmd("\r\n");

	//        sprintf(cmd, "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n",host, port);
  } else {
    return false;
  }

  delay(2000);
    //sim900_send_cmd(cmd);
  sim900_read_buffer(resp,96,timeout);

//Serial.print("Connect resp: "); Serial.println(resp);
    if(NULL != strstr(resp,"CONNECT")) { //ALREADY CONNECT or CONNECT OK
      return true;
    }
    return false;
  }

//Overload with F() macro to SAVE memory
  bool GPRS::connect(Protocol ptl,const __FlashStringHelper *host, const __FlashStringHelper *port, int timeout)
  {
    //char cmd[64];
    char resp[96];

    //sim900_clean_buffer(cmd,64);
    if(ptl == TCP) {
        sim900_send_cmd(F("AT+CIPSTART=\"TCP\",\""));   //%s\",%d\r\n",host, port);
} else if(ptl == UDP) {
        sim900_send_cmd(F("AT+CIPSTART=\"UDP\",\""));   //%s\",%d\r\n",host, port);
} else {
  return false;
}
sim900_send_cmd(host);
sim900_send_cmd(F("\","));
sim900_send_cmd(port);
sim900_send_cmd(F("\r\n"));
//Serial.print("Connect: "); Serial.println(cmd);
sim900_read_buffer(resp, 96, timeout);
//Serial.print("Connect resp: "); Serial.println(resp);
    if(NULL != strstr(resp,"CONNECT")) { //ALREADY CONNECT or CONNECT OK
      return true;
    }
    return false;
  }

  bool GPRS::is_connected(void)
  {
    char resp[96];
    sim900_send_cmd("AT+CIPSTATUS\r\n");
    sim900_read_buffer(resp,sizeof(resp),DEFAULT_TIMEOUT);
    if(NULL != strstr(resp,"CONNECTED")) {
        //+CIPSTATUS: 1,0,"TCP","216.52.233.120","80","CONNECTED"
      return true;
    } else {
        //+CIPSTATUS: 1,0,"TCP","216.52.233.120","80","CLOSED"
        //+CIPSTATUS: 0,,"","","","INITIAL"
      return false;
    }
  }

  bool GPRS::close()
  {
    // if not connected, return
    if (!is_connected()) {
      return true;
    }
    return sim900_check_with_cmd("AT+CIPCLOSE\r\n", "CLOSE OK\r\n", CMD);
  }

  int GPRS::readable(void)
  {
    return sim900_check_readable();
  }

  int GPRS::wait_readable(int wait_time)
  {
    return sim900_wait_readable(wait_time);
  }

  int GPRS::wait_writeable(int req_size)
  {
    return req_size+1;
  }

  int GPRS::send(const char * str, int len)
  {
    //char cmd[32];
   char num[4];
   if(len > 0){
        //snprintf(cmd,sizeof(cmd),"AT+CIPSEND=%d\r\n",len);
		//sprintf(cmd,"AT+CIPSEND=%d\r\n",len);
    sim900_send_cmd("AT+CIPSEND=");
    itoa(len, num, 10);
    sim900_send_cmd(num);
    if(!sim900_check_with_cmd("\r\n",">",CMD)) {
        //if(!sim900_check_with_cmd(cmd,">",CMD)) {
      return 0;
    }
        /*if(0 != sim900_check_with_cmd(str,"SEND OK\r\n", DEFAULT_TIMEOUT * 10 ,DATA)) {
            return 0;
        }*/
            delay(500);
            sim900_send_cmd(str);
            delay(500);
            sim900_send_End_Mark();
            if(!sim900_wait_for_resp("SEND OK\r\n", DATA, DEFAULT_TIMEOUT * 10, DEFAULT_INTERCHAR_TIMEOUT * 10)) {
              return 0;
            }
          }
          return len;
        }

        int GPRS::send(const char * str) {
    //char cmd[32];
          int len=strlen(str);
          char num[4];
          if(len > 0){
        //snprintf(cmd,sizeof(cmd),"AT+CIPSEND=%d\r\n",len);
    //sprintf(cmd,"AT+CIPSEND=%d\r\n",len);
            sim900_send_cmd("AT+CIPSEND=");
            itoa(len, num, 10);
            sim900_send_cmd(num);
            if(!sim900_check_with_cmd("\r\n",">",CMD)) {
        //if(!sim900_check_with_cmd(cmd,">",CMD)) {
              return 0;
            }
        /*if(0 != sim900_check_with_cmd(str,"SEND OK\r\n", DEFAULT_TIMEOUT * 10 ,DATA)) {
            return 0;
        }*/
            delay(500);
            sim900_send_cmd(str);
            delay(500);
            sim900_send_End_Mark();
            if(!sim900_wait_for_resp("SEND OK\r\n", DATA, DEFAULT_TIMEOUT * 10, DEFAULT_INTERCHAR_TIMEOUT * 10)) {
              return 0;
            }
          }
          return len;
        }

        int GPRS::recv(char* buf, int len)
        {
          sim900_clean_buffer(buf,len);
    sim900_read_buffer(buf,len);   //Ya he llamado a la funcion con la longitud del buffer - 1 y luego le estoy añadiendo el 0
    return strlen(buf);
  }

uint32_t GPRS::str_to_ip(const char* str)
{
  uint32_t ip = 0;
  char* p = (char*)str;
  for(int i = 0; i < 4; i++) {
    ip |= atoi(p);
    p = strchr(p, '.');
    if (p == NULL) {
      break;
    }
    ip <<= 8;
    p++;
  }

  return ip;
}

char* GPRS::getIPAddress()
{
  //I have already a buffer with ip_string: snprintf(ip_string, sizeof(ip_string), "%d.%d.%d.%d", (_ip>>24)&0xff,(_ip>>16)&0xff,(_ip>>8)&0xff,_ip&0xff);
  return ip_string;
}

unsigned long GPRS::getIPnumber()
{
  return _ip;
}

void setST(uint8_t stPin)
{

}
void setPK(uint8_t pkPin)
{

}
