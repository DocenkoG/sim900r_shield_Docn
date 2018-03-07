/*
-------------------
*/

#include <sim900r_shield.h>
#include "private.h"
//#include <OneWire.h>             // Нужна, если нет DallasTemperature.h
//#include <Wire.h>
#include <troyka-imu.h>            // для барометра
//#include <DallasTemperature.h>
//#include <Time.h>      

#define PIN_PK         8           // Контакт включения GPRS модуля
#define PIN_ST         9           // контакт состояния GPRG модуля
#define ONE_WIRE_USB1 12           // Номер линии, к которой подключены датчики температуры (белый USB)
#define ONE_WIRE_USB2  7           // Номер линии, к которой подключены датчики температуры (белый USB)
#define DATA_USB1     13           // пока не использую (зеленый USB)
#define DATA_USB2      6           // пока не использую (зеленый USB)
#define BAUDRATE_USB  115200       // частота обмена Arduino <-> компьютер
#define BAUDRATE_SIM  19200        // частота обмена Arduino <-> sim900r

//#define MESSAGE_LEN  160
#define TROIKA_TEMP_ERROR     2    // Погрешность из-за нагрева прибора
#define TEMPERATURE_PRECISION 9    // Точноть 9 бит

#define ALL_INCOMING_SMS_ENABLED (1)  // 1 - разрешен приём команд со всех входящих/ 0 -только с номера phoneAdmin


const char phoneOwner[]= PHONE_OWNER;      // Телефон владельца устройства
const char phoneAdmin[]= PHONE_ADMIN;      // Телефон разработчика
const char pinCode[]   = "0000";           // ПИН-код! Поменяйте на свой!!! Иначе СИМ карта заблокируется!
const char apn[]       = "internet.mts.ru";// Для Билайн: "home.beeline.ru", МТС: "internet.mts.ru" megafon: "internet"
const char lgn[]       = "mts";            // Логин=Пароль MTS: "mts" megafon: "gdata"
const char balanceReq[]= "#100#";          // USSD номер проверки балланса
const char ntpService[]= "pool.ntp.org";   // Сервер NTP-синхронизации
const char broken[]    = "broken";         // 

char imei[16];                           // Буфер для считывания IMEI
char txtSMS[64];                         // Буфер для чтения текста СМС
char SenderID[16];                       // Буфер номер отправителя СМС
char currentPressure[7];                 // Данные о текущем давлении
char currentBarometrTemp[5];             // Данные о текущей температуре
char previosBarometrTemp[5];             // Данные о предшествующей температуре
int  MoneyBalanceInt;                    // Переменная в которой хранится текущий денежный баланс
char MoneyBalanceBuf[32];                // Строка ответа о текущем балансе
int  MoneyBalanceTreshold      = 30;     // Нижний порог отправки СМС о снижении баланса
unsigned char forceModemReInit =  0;     // Флаг принудительной переинициализации модема в случае обнаружения проблем

          //  Интервалы выполнения задач в минутах от 1 до 65535 (45 дней)
unsigned int periodConnectionChk=  2;    // Проверки GPRS соединения
unsigned int periodSensorChk    =  5;    // Снятия показаний датчиков
unsigned int periodDataSend     = 15;    // Отправки данных
unsigned int periodSmsStatus   =1440;    // Отправки sms
unsigned int periodNTPSync     =1440;    // NTP синхронизации
unsigned int periodBalanceChk   =360;    // Проверки баланса
unsigned int periodSysTimeSync  = 15;    // Синхронизации часов Ардуино с часами модема
unsigned int periodPowerOff     = 60;    // 

          //  Интервалы выполнения задач в секундах  от 1 до 65535 (18 час)
unsigned int periodUnreadSMSChk = 15;    // Принудительной проверки непрочитаных СМС
 
          //  Счетчик времени до выполнения задачи
unsigned int minutesBeforeConnectionChk; // проверки GPRS соединения
unsigned int minutesBeforeSensorChk;     // проверки датчиков
unsigned int minutesBeforeDataSend;      // отправки данных
unsigned int minutesBeforeSmsStatus;     // отправки sms
unsigned int minutesBeforeNTPSync;       // NTP синхронизации
unsigned int minutesBeforeBalanceChk;    // проверки баланса
unsigned int minutesBeforeSysTimeSync;   // синхронизации часов Ардуино с часами модема
unsigned int minutesBeforePowerOff;      // отключение модеиа для перепрошивки ардуино
unsigned int secondsBeforeUnreadSMSChk;  // принудительной проверки непрочитаных СМС

unsigned long currentMillisM;            // Маркеры для отсчёта минут
unsigned long currentMillisS;            // и секунд
unsigned long previousMillisM; 
unsigned long previousMillisS;        
unsigned long minutesCounter;         // Счётчик минут
unsigned long secondsCounter;         // Счётчик секунд, 4294967296 milis = 49 дней 

                                      // создаём объекты
Barometer           barometer;  
GPRS   gprsModul(Serial1, PIN_PK, PIN_ST);

//OneWire             oneWire(ONE_WIRE_USB1);
//DallasTemperature   sensors(&oneWire);
//DeviceAddress sensor_1, sensor_2, sensor_3, sensor_4; // Используем 4 температурных датчика



void setup() 
{
    unsigned long millis_1 = millis();
    Serial.begin(BAUDRATE_USB);
    while (millis() - millis_1 < 20L*1000L) {   // ждем в пределах 20 сек
        if (Serial) {                           // пока откроется монитор последовательного порта
            break;                 
        }
    }
    Serial.print("Serial OK!");
    Serial1.begin(BAUDRATE_SIM);
    //
    gprsModul.powerOn();
    int rc = gprsModul.init(); 
    Serial.print(" Init rc=");
    Serial.println(rc);
    sim900_clean_buffer(imei,sizeof(imei));
    gprsModul.getImei(imei);
    
    minutesCounter = 0;                         // Сброс минут
    secondsCounter = 0;                         // и секунд
    previousMillisS=previousMillisM=millis();   // Запоминание системного времени
   
    // Ставим начальные значения для запуска всех событий при старте
    minutesBeforeConnectionChk =  0;       
    minutesBeforeSensorChk     =  0;       
    minutesBeforeDataSend      =  0;       
    minutesBeforeSmsStatus     =  0;       
    minutesBeforeNTPSync       =  2;       
    secondsBeforeUnreadSMSChk  =  1;       
    minutesBeforeBalanceChk    =  0;       
    minutesBeforeSysTimeSync   =  0; 
    minutesBeforePowerOff      = 60;      
}



void loop() 
{
    unsigned long dMillis;
    currentMillisM = currentMillisS = millis();    // Считывем текущее системное время
    dMillis = currentMillisS - previousMillisS;
    if( dMillis >= 1000L) {                          // Прошло больше секунды. 
        unsigned int dSeconds  = (unsigned int) (dMillis / 1000L); 
        previousMillisS = currentMillisS - (dMillis % 1000L);
        //
        secondsCounter += dSeconds;
        // Декремент счётчиков секунд
        secondsBeforeUnreadSMSChk -= min(dSeconds,secondsBeforeUnreadSMSChk);
    }
  
    dMillis = currentMillisM - previousMillisM;   
    if(dMillis >= 60000) {                       // Прошло больше минуты
        unsigned int deltaMin = (unsigned int) (dMillis / 60000);   // Определяем сколько минут прошло
        minutesCounter    += deltaMin;            // Увеличиваем счетчик минут
        previousMillisM = currentMillisM - (dMillis % 60000);
        // Декремент счётчиков минут
        minutesBeforeConnectionChk -= min(deltaMin,minutesBeforeConnectionChk);
        minutesBeforeSensorChk     -= min(deltaMin,minutesBeforeSensorChk);
        minutesBeforeDataSend      -= min(deltaMin,minutesBeforeDataSend);
        minutesBeforeSmsStatus     -= min(deltaMin,minutesBeforeSmsStatus);
        minutesBeforeNTPSync       -= min(deltaMin,minutesBeforeNTPSync);
        minutesBeforeBalanceChk    -= min(deltaMin,minutesBeforeBalanceChk);
        minutesBeforeSysTimeSync   -= min(deltaMin,minutesBeforeSysTimeSync);
        minutesBeforePowerOff      -= min(deltaMin,minutesBeforePowerOff);
    }
  

    if( forceModemReInit ) {        // Потребовалась переинициализация модема
        forceModemReInit = 0;
        rebootModem();
    }


    if( secondsBeforeUnreadSMSChk <= 0) {
        secondsBeforeUnreadSMSChk = periodUnreadSMSChk;
        Serial.print("*");
        if (gprsModul.readSMS(txtSMS, SenderID)) commandProcessorSms(txtSMS, SenderID);
    }


    if( minutesBeforeSensorChk <= 0) {
        Serial.begin(BAUDRATE_USB);
        while (millis() - currentMillisS < 2L*1000L) { // ждем в пределах 2 сек
            if (Serial) {                              // пока откроется монитор последовательного порта
                break;                 
            }
        }
        minutesBeforeSensorChk = periodSensorChk;
        readBarometerPressure(currentPressure);
        readBarometerTemperature(currentBarometrTemp);
        char tmpBuf[20];
        Serial.println();
        Serial.print(minutesCounter);
        Serial.print(" min.  ");
        Serial.print(gprsModul.getDateTime(tmpBuf));
        Serial.print("; ");   
        Serial.print(currentPressure);   
        Serial.print("; ");   
        Serial.print(currentBarometrTemp);   
        Serial.print(";");
        Serial.println();   
    }


    if( minutesBeforeBalanceChk <= 0) {
        minutesBeforeBalanceChk = periodBalanceChk;
        gprsModul.readBalance(balanceReq, MoneyBalanceBuf, sizeof(MoneyBalanceBuf), MoneyBalanceInt);
        Serial.print( MoneyBalanceBuf);
        Serial.print(";  int=");
        Serial.println(MoneyBalanceInt);
    }

/*  
  if( minutesBeforeNTPSync <= 0) {
    minutesBeforeNTPSync = periodNTPSync;
    debugOutput("NTPSync");
    signed char rc = gprsModul.syncNtp(ntpService);
    Serial.print(" rc=");
    Serial.println(rc);   
  }
*/  
    if( minutesBeforePowerOff <= 0) {
        minutesBeforePowerOff = periodPowerOff;
        gprsModul.powerOff();
        forceModemReInit = 1;
        return;
    }

/*
  if (DataSendTimerM >= DataSendPeriodM) {
    // sendData....();
    DataSendTimerM = 0L;
  }
  //
  if (smsStatusTimerM >= smsStatusPeriodM) {
    sendStatusSMS();
    smsStatusTimerM = 0L;
  }
*/
}



char* readBarometerPressure(char* buffer) 
{
    barometer.begin();
    byte bb = barometer.readReg(LPS331_CTRL_REG1);
    if (bb != 0b11100000) {
        strcpy(buffer, broken);
        return buffer;
    }
    float pressure = barometer.readPressureMillibars()/1.33322;
    itoa(pressure, buffer, 10);
    buffer = strcat(buffer, "mm");
    return buffer;
}



char* readBarometerTemperature(char* buffer) 
{
    barometer.begin();
    byte bb = barometer.readReg(LPS331_CTRL_REG1);
    if (bb != 0b11100000) {
        strcpy(buffer, broken);
        return buffer;
    }
    float temperature = barometer.readTemperatureC() - TROIKA_TEMP_ERROR;
    itoa(temperature, buffer, 10);
    return buffer;
}


/*
char* readDalasTemperature(char* buffer) 
{
  // опрос всех датчиков на шине (Dalas Temperature)
  float ftemp;
  if (sensors.getAddress(sensor_1, 0) ) {
    sensors.setResolution(sensor_1, TEMPERATURE_PRECISION);
    sensors.requestTemperaturesByAddress(sensor_1);
    ftemp = sensors.getTempC(sensor_1);
    itoa(ftemp, buffer, 10);
  } else {
    strcpy(buffer, broken);
  }
  return buffer;
}
*/


/*
void sendStatusSMS(void) {
  unsigned int TrySMS = 8; // Количество попыток отправки СМС
  signed char rc;
  char   buf[20];
  char   textMsg[162] = "Hi from Daemon #5!\r\n";
  strcat(textMsg, gprsModul.getDateTime(buf));
  strcat(textMsg, "Barometr=");
  strcat(textMsg, readBarometerPressure(buf));
  strcat(textMsg, ",temp_1=");
  strcat(textMsg, readBarometerTemperature(buf));
  //strcat(textMsg, ", temp_2=");
  //strcat(textMsg, readDalasTemperature(buf));
//  rc = gprsModul.readBalance(MoneyBalanceBuf, sizeof(MoneyBalanceBuf), MoneyBalanceInt);
  strcat(textMsg, ",");
  if (rc>0) {
    strcat(textMsg, "balans=");
    strcat(textMsg, itoa(MoneyBalanceInt, buf, 10));
    strcat(textMsg, "r");
  } else {
    strcat(textMsg, MoneyBalanceBuf);
  }
  Serial.println(textMsg);
//  gprsModul.sendSMS(phoneAdmin, textMsg);
  delay(2000);               // паузы 500 недостаточно, 3000 - достаточно
  //gprsModul.sendSMS(phoneAdmin, textMsg);
  //delay(2000); 
}
*/



uint8_t commandProcessorSms(char* txtSMS, char* SenderID) {
    Serial.print("\nFrom: ");
    Serial.print(SenderID);
    Serial.print(";  sms: ");
    Serial.println(txtSMS);
    return 0;
}



void rebootModem(void) {
    unsigned char rc;
    gprsModul.powerOff();
    gprsModul.powerOn();
    rc = gprsModul.init(); 
    Serial.print("\nInit rc=");
    Serial.println(rc);
}