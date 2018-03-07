#include <sim900r_shield.h>

#define PIN_PK         8           // Контакт включения GPRS модуля
#define PIN_ST         9           // контакт состояния GPRG модуля
#define BAUDRATE_USB  115200       // частота обмена Arduino <-> компьютер
#define BAUDRATE_SIM  19200        // частота обмена Arduino <-> sim900r

GPRS   gprsModul(Serial1, PIN_PK, PIN_ST);   // создаём объект 

void setup() 
{
  Serial.begin(BAUDRATE_USB);
  while (!Serial) {
    // ждем, пока откроется монитор последовательного порта
  } 
  Serial1.begin(BAUDRATE_SIM);
  Serial.println("Serial OK!");
  Serial.print  ("Power... ");
  gprsModul.powerOn();
  Serial.print  (" On");
  //
  Serial.print  (";   init... ");
  char rc = gprsModul.init();
  Serial.print(" rc=");
  Serial.print(int(rc));  
}



void loop() {
  unsigned char rc;
  char tmpBuf[64];
  //
  Serial.print(";   imei=");
  sim900_clean_buffer(tmpBuf, sizeof(tmpBuf));
  gprsModul.getImei(tmpBuf);
  Serial.print(tmpBuf);
  Serial.println(";");
  //
  delay(15*1000L);
}
