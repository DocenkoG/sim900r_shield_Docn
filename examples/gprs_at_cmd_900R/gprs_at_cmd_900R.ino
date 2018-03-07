/***************************************************************** 
 *  Скетч для исследования AT команд и поведения GPRS-shield
 *  Предназначен для Arduino Leonardo (Iskra Neo), имеющих
 *  аппаратный serial1 порт (RX, TX) на контактах 0 и 1,
 *  Благодаря этому не требуется подключения никаких библиотек.
 *  Отлаживался с GPRS модулем от Амперки.
 *  Для включения / выключения GPRS-модуля введите символ "^"
 *****************************************************************/
 
#define PIN_PK  8
#define PIN_ST  9
#define BAUDRATE_USB  115200
#define BAUDRATE_SER1  19200

void setup()
{
  Serial.begin(BAUDRATE_USB);
  Serial1.begin(BAUDRATE_SER1);
 
  while (!Serial) { }   // ждём, пока не откроется монитор последовательного порта 
  Serial.println("Serial OK");
  powerSwitch();
  delay(10*1000);       // пауза для загрузки модема и регистрации его в сети
}
 
void loop()
{
  if(Serial1.available()){
    Serial.write(Serial1.read());
  }
  if(Serial.available()){   
    char c = Serial.read();
    Serial1.write(c);                 // записываем их в GPRS Shield
    if (c == '^')  {                  // -- Включить/выключить питание GSM модуля
      Serial.print("--- ");
      powerSwitch();
    }
  }  
}
 

void powerSwitch(void) {     // The same sequence is used for switching on and to power off
  pinMode(PIN_PK, OUTPUT);
  digitalWrite(PIN_PK, LOW);
  delay(1000);
  digitalWrite(PIN_PK, HIGH);
  delay(2000);
  digitalWrite(PIN_PK, LOW);
  delay(3000);
}

