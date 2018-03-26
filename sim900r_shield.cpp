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



unsigned char GPRS::syncNtp (const char* ntpServer) //Синхронизфция времени в модеме с NTP сервером
{ /*
  AT+CNTP="pool.ntp.org",3,1,0
  AT+CNTP

  Коды завершения:
  1 - удачная синхронизация
  11 - нет GPRS
  12 - не установились параметры для синнхронизации
  61 - ошибка сетевого соединения
  62 - ошибка DNS
  63 - ошибка соединения
  64 - превышено время ожидания
  65 - ошибка сервера
  66 - операция недоступна
  */
   
  char *p,*s;
  char tmpBuf[56];
  unsigned char rc;
  sim900_clean_buffer(tmpBuf,sizeof(tmpBuf));
  if (1 == (rc = joinGprs(tmpBuf))) {       // Есть GPRS подключение или удалось подключиться
    sim900_send_cmd("AT+CNTP=\"");          // устанавливаем параметры синхронизации
    sim900_send_cmd(ntpServer);
    sim900_send_cmd("\",3,1,0\r\n");
    if (sim900_wait_for_resp(OK,CMD)) {     // Параметры установились успешно
      sim900_check_with_cmd("AT+CNTP\r\n"   ,OK,DATA);   // собственно синхронизация.  
      sim900_wait_for_resp ("+CNTP: ",  DATA,19,15000);  // ждем ответ об успешности 
      delay(10);
      sim900_clean_buffer(tmpBuf,sizeof(tmpBuf));
      sim900_read_buffer(tmpBuf,4);                      // читаем код завершения
      rc = atoi(tmpBuf);

    } else {                          // Не установились параметры синхронизации NTP
      rc = 12;
    }
  } else {
    rc = 11;                          // Если не удалось подключиться по GPRS
  }
  return rc;
}



void GPRS::read_buffer(char* buffer, int count) {
    sim900_read_buffer(      buffer,     count);
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



///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
///                                GPRS                                     ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

unsigned char GPRS::getGprsStatus(char* ipv4Buf)   
{
  /*
  AT+SAPBR=2,1                       2
                                     2
  +SAPBR: 1,3,"xxx.xxx.xxx.xxx"     31
                                     
  OK

  Коды возврата во втором параметре
  0 - соединение устанавливается
  1 - соединение установлено
  2 - соединение закрывается
  3 - нет соединения
  */                                   

  char *p, *s;
  unsigned char  rc;
  char  tmpBuf[35];
  int   i;
  sim900_flush_serial();
  sim900_send_cmd("AT+SAPBR=2,1\r\n");               // Запрос о состоянии GPRS соединения
  sim900_clean_buffer(tmpBuf,sizeof(tmpBuf));
  sim900_read_buffer(tmpBuf,sizeof(tmpBuf)-1,DEFAULT_TIMEOUT); // Считываем ответ
  //Serial.println("<");
  //Serial.println(tmpBuf);
  //Serial.println(">");

  if(NULL != ( s = strstr(tmpBuf,"+SAPBR: "))) {     // находим нужное место
    s += 8;                                          // 
    if(NULL != ( s = strstr((char *)(s),","))) {     // после первой запятой
      rc = *(s+1) - '0';                // для конвертации в цифру отнять код нуля
      s = strstr((char *)(s),"\"")+1;   // Теперь ищем ip-адрес в кавычках
      p = strstr((char *)(s),"\"");     // Закрывающая кавычка
      if ((NULL != p) && (NULL != s)) {
        i = 0;
        while (s < p) {
          ipv4Buf[i++] = *(s++);
        }
        ipv4Buf[i] = '\0';            
      } else {
        rc = 9;                 // не распознан ip-адрес
      }
    } else {
      rc = 10;                  // не распознан код состояния
    }
  } else {
    rc = 10;                    // не распознан код состояния
  }
  delay(10);               // Перед очисткой буфера надо дождаться завер-     
  sim900_flush_serial();   // шения вывода данных.
  return rc;
}



unsigned char GPRS::joinGprs(char* ipv4Buf)
{
  char *p,*s;
  long t1, t2;
  unsigned char  rc;
  rc = getGprsStatus(ipv4Buf);                // Запрос о состоянии GPRS соединения
  if (rc != 1)   {                            // Если не установлено, то установим
    sim900_check_with_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n",OK,CMD);
    sim900_check_with_cmd("AT+SAPBR=1,1\r\n" ,OK,CMD);
    rc = getGprsStatus(ipv4Buf);
  }
  return rc;
}



int GPRS::httpGet(char* url, int&  dataLen)
{
  char  tmpBuf[111];
  char *p, *s;
  int   result;
  sim900_flush_serial();
  sim900_clean_buffer(tmpBuf, sizeof(tmpBuf));
  sim900_check_with_cmd("AT+HTTPINIT\r\n",OK,CMD,33);
  sim900_check_with_cmd("AT+HTTPPARA=\"CID\",1\r\n",OK,CMD);
  sim900_send_cmd("AT+HTTPPARA=\"URL\",\"");
  sim900_send_cmd(url);
  sim900_check_with_cmd("\"\r\n",OK,CMD);
  sim900_check_with_cmd("AT+HTTPACTION=0\r\n",OK,CMD);
  if(!sim900_wait_for_resp ("+HTTPACTION:0," ,DATA))               return 7;
  //   
  sim900_read_buffer(tmpBuf,sizeof(tmpBuf)-1,DEFAULT_TIMEOUT);
  sim900_flush_serial();
  
  if(NULL != ( s = strstr(tmpBuf,","))) {
    tmpBuf[ s-tmpBuf ] = '\0';
    result = atoi( tmpBuf );
  } else {
    result = 1;                             // Статус Код не распознан
  } 
  //
  if(NULL != ( p = strstr( (char *)(s+1),"\r\n"))) {
    tmpBuf[ p-tmpBuf ] = '\0';
    dataLen = atoi( (char *)(s+1) );
  } else {
    result = 2;                             // dataLen не распознан
  } 
  return result;
}






void setST(uint8_t stPin)
{

}
void setPK(uint8_t pkPin)
{

}
