// Emulate the THGR2228N temperature and humidity sensor using a HTU21

// rf433 lib: https://github.com/tazounet/OregonV2Sender
// sensor lib: https://github.com/tazounet/SparkFun_HTU21D_Breakout_Arduino_Library
// power down lib: https://github.com/tazounet/Low-Power
// Arduino mini pro@1Mhz: https://github.com/tazounet/Arduino-LowPower

// serial debug
#define DEBUG 0

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "OregonV2Sender.h"
#include "SparkFunHTU21D.h"
#include "LowPower.h"

const uint8_t txPin = 4; // digital pin connected to the RF transmitter
const uint8_t ledPin = 13; // digital pin for LED

HTU21D htu;
OregonV2Sender sender;

typedef struct {
   uint8_t sensorId;
   uint8_t channel;
} sensorConf;

void setup()
{
  sensorConf cfg;
  uint16_t crcRead;
  uint16_t crcComputed;

#if DEBUG
  Serial.begin(9600);
  Serial.println("Setup");
#endif

  // read config from eeprom
  EEPROM.get(0, cfg);

  // read CRC
  EEPROM.get(sizeof(sensorConf), crcRead);

  // check CRC
  crcComputed = crc16((char*)&cfg, sizeof(sensorConf));

#if DEBUG
    Serial.print("CRC read=");
    Serial.print(crcRead);
    Serial.print(", CRC computed=");
    Serial.println(crcComputed);
#endif

  if (crcComputed != crcRead)
  {
    randomSeed(analogRead(0));
    cfg.sensorId = random (0x0, 0xFF);
    cfg.channel = 1;

    // write config to eeprom
    EEPROM.put(0, cfg);

    // write crc to eeprom
    crcComputed = crc16((char*)&cfg, sizeof(sensorConf));
    EEPROM.put(sizeof(sensorConf), crcComputed);

#if DEBUG
    Serial.print("Write to EEPROM, Sensor Id=");
    Serial.print(cfg.sensorId);
    Serial.print(", channel=");
    Serial.println(cfg.channel);
#endif
  }
  else
  {
#if DEBUG
    Serial.print("Read from EEPROM, Sensor Id=");
    Serial.print(cfg.sensorId);
    Serial.print(", channel=");
    Serial.println(cfg.channel);
#endif
  }

  sender.setup(txPin, cfg.channel, cfg.sensorId, true);
  pinMode(ledPin, OUTPUT);
  htu.begin();
}

void loop()
{
  float humd = htu.readHumidity();
  float temp = htu.readTemperature();
  long volt = readVcc();

#if DEBUG
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print("C, Hum: ");
  Serial.print(humd);
  Serial.print("%, Batt: ");
  Serial.print(volt / 1000.0);
  Serial.println("V");
#endif

  // send the data
  // 2*0.9V is low battery
  
  sender.send((byte)humd, temp, (volt > 1800));

  blinkLed();

  // delay for 32 seconds
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

void blinkLed()
{
  digitalWrite(ledPin, HIGH);
  delay(30);   
  digitalWrite(ledPin, LOW);
}

long readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

unsigned short crc16(char *msg, size_t size) {
  int i = 0;
  unsigned short crc = 0xffffU;

  while(size--)
  {
    crc ^= (unsigned short) *msg++ << 8;

    for(i = 0; i < 8; ++i)
      crc = crc << 1 ^ (crc & 0x8000U ? 0x1021U : 0U);
  }
  return(crc & 0xffffU);
}


