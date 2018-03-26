#include <sim900r_shield.h>

#define PIN_PK         8              // Контакт включения GPRS модуля
#define PIN_ST         9              // контакт состояния GPRG модуля
#define BAUDRATE_USB  115200       // частота обмена Arduino <-> компьютер
#define BAUDRATE_SIM  19200        // частота обмена Arduino <-> sim900r

char    ipv4Buf[16]; 

GPRS    gprsModul(Serial1, PIN_PK, PIN_ST);



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
}



void loop() {
  unsigned char rc;
  char tmpBuf[511];
  char urlBuf[]="http://diribus.net/loger.php?ddddddd=fff";
  int  dataLen;
  //
  Serial.println();
  sim900_clean_buffer(tmpBuf, sizeof(tmpBuf));
  rc = gprsModul.joinGprs(ipv4Buf);
  Serial.print ("joinGprs rc=");
  Serial.print (rc);
  Serial.print (";  ");
  Serial.print (ipv4Buf);
  rc = gprsModul.httpGet( urlBuf, dataLen );
  sim900_read_buffer (tmpBuf, dataLen);
  Serial.print (";  httpGet rc=");
  Serial.print (rc);
  Serial.print (";  Размер страницы =");
  Serial.print (dataLen);
  Serial.println();
  delay(15*1000);
}
