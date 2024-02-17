/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/
//#include <Adafruit_TinyUSB.h> // for Serial
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoBLE.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10


// Data for CO2 sensor
byte ReadCO2[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte SelfCalOn[9]  = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
byte SelfCalOff[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
byte retval[9];
uint16_t uartco2;

BLEAdvertisingData advData;
//uint8_t cnt;

const uint8_t manufactData[4] = {0x01, 0x02, 0x03, 0x04};

// 0x01 + 4byte identifier
// 0x10 (temp) + 2 byte data
// 0x11 (hum) + 2 byte data
// 0x14 (press) + 2 byte data
// 0x17 (CO2) + 2 byte data

byte serviceData[17] = {
  // fix value
  0x01,
  // 4 byte UUID
  0x00,0x00,0x00,0x00,
  // Temp
  0x10,0x00,0x00,
  // Humitity
  0x11,0x00,0x00,
  // Pressure
  0x14,0x00,0x00,
  // CO2
  0x17,0x00,0x00
};

String address;

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

float temp,humidity,pressure;

void setup() {

  // Enbaled LED output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  Serial.begin(9600);
  while(!Serial){
      // Blink LED as Blue.
      digitalWrite(LED_BLUE, LOW);
      delay(500);
      digitalWrite(LED_BLUE, HIGH);
      delay(500);
  }
  Serial.println(F("BME280 test"));

  if (! bme.begin(0x76, &Wire)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1){
        // Blink LED as Red.
        digitalWrite(LED_RED, LOW);
        delay(500);
        digitalWrite(LED_RED, HIGH);
        delay(500);
      }
  }

  Serial.println("-- Default Test --");
  Serial.println("normal mode, 16x oversampling for all, filter off,");
  Serial.println("0.5ms standby period");
  delayTime = 1000;


  // For more details on the following scenarious, see chapter
  // 3.5 "Recommended modes of operation" in the datasheet


  // weather monitoring
  Serial.println("-- Weather Station Scenario --");
  Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  Serial.println("filter off");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

  // suggested rate is 1/60Hz (1m)
  delayTime = 10000; // in milliseconds

  Serial.println();

  delay(2000);

  if (!BLE.begin()) {
    Serial.println("failed to initialize BLE!");
    while (1){
        // Blink LED as Red.
        digitalWrite(LED_RED, LOW);
        delay(500);
        digitalWrite(LED_RED, HIGH);
        delay(500);
    }
  }

  //Initialize CO2 sensore
  Serial1.begin(9600);
  Serial1.write(SelfCalOn,sizeof SelfCalOn);

  address = BLE.address();
  Serial.print("Local BLE address is: ");
  Serial.println(address);

  //スキャン応答ペイロードのデバイス名称==> 31バイトを消費するので、利用しない
  //advData.setLocalName("Xiao");

  // Set parameters for advertising packet
  //advData.setFlags(0x09); //アドバタイジングパケットをモード初期値0x06
  advData.setManufacturerData(0xFFFF, manufactData, sizeof(manufactData));

  //Serial.println(strtol(address.substring(7,2), 0, 16));
  // address xx:xx:xx:xx:xx:xx
  // set last 4 byte as UUID
  byte u;
  char buf[3];
  address.substring(6,8).toCharArray(buf, sizeof(buf));
  sscanf(buf, "%x", &u);
  serviceData[1]=u;
  address.substring(9,11).toCharArray(buf, sizeof(buf));
  sscanf(buf, "%x", &u);
  serviceData[2]=u;
  address.substring(12,14).toCharArray(buf, sizeof(buf));
  sscanf(buf, "%x", &u);
  serviceData[3]=u;
  address.substring(15,17).toCharArray(buf, sizeof(buf));
  sscanf(buf, "%x", &u);
  serviceData[4]=u;

  // 0xfcbe = musen connect UUID
  advData.setAdvertisedServiceData(0xfcbe, serviceData, sizeof(serviceData));
  BLE.setAdvertisingData(advData);
  BLE.advertise();
  Serial.println("advertising ...");

  //Configure WDT.
  NRF_WDT->CONFIG         = 0x01;     // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV            = 9830401;  // Timeout set to 300 seconds, timeout[s] = (CRV-1)/32768
  NRF_WDT->RREN           = 0x01;     // Enable the RR[0] reload register
  NRF_WDT->TASKS_START    = 1;        // Start WDT

}

void loop() {

  // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement(); // has no effect in normal mode

  //printValues();

  Serial.print("Temperature = ");
  temp=bme.readTemperature();
  Serial.print(temp);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  pressure=bme.readPressure()/ 100.0F;
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  humidity=bme.readHumidity();
  Serial.print(humidity);
  Serial.println(" %");

  //UARTでCO2データ取得
  Serial1.write(ReadCO2,sizeof ReadCO2);
  Serial1.readBytes((char *)retval, sizeof retval);
  uartco2 = retval[2]*256 + retval[3];

  //PCに送信
  Serial.print("CO2 = ");
  Serial.print(uartco2);
  Serial.println();

  delay(delayTime);

  // Convert data to musen connect format.
  int var;
  var = temp*100;
  serviceData[6] = var >>8;
  serviceData[7] = var & 0xFF;
  var = humidity*100;
  serviceData[9] = var >>8;
  serviceData[10] = var & 0xFF;
  var = pressure*10;
  serviceData[12] = var >>8;
  serviceData[13] = var & 0xFF;
  serviceData[15] = uartco2 >>8;
  serviceData[16] = uartco2 & 0xFF;

  advData.setAdvertisedServiceData(0xfcbe, serviceData, sizeof(serviceData));
  BLE.setAdvertisingData(advData);
  BLE.advertise();

  // Blink LED as Green.
  digitalWrite(LED_GREEN, LOW);
  delay(100);
  digitalWrite(LED_GREEN, HIGH);

  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;

}