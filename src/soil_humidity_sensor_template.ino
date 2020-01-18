/*******************************************************************************
   This sketch is derived from the lmic example sketch for OTAA LoRaWAN nodes. 
   The example sketch was created by Thomas Telkamp, Matthijs Kooijman and
   Terry Moore. 

   The example sketch was extended and modified to run on the LoRaWAN Energy 
   Harvester node (https://github.com/ablexOnGithub/energyharvester) and
   use a capacitive soil humidity sensor.

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in
   arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.

 *******************************************************************************/
#include <LowPower.h>
#include <avr/power.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define SDA_PORT PORTC
#define SDA_PIN 4 // = A4
#define SCL_PORT PORTC
#define SCL_PIN 5 // = A5

#define I2C_HARDWARE 1
#define I2C_TIMEOUT 5000
#define I2C_SLOWMODE 1

#define PIN_VCC A0

#include <SoftWire.h>
SoftWire Wire = SoftWire();

// show debug statements; comment next line to disable debug statements
//#define DEBUG

// show development messages on console.
//#define DEVEL

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


static uint8_t mydata[24];
static osjob_t sendjob;

int onPinI2C1 = 4; //PD4
int onPinI2C2 = 5; //PD5
int onPinADC = 6; //PD6

int adcVal = 0;
int sensorAddress = 0x20;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 100; // approx. 46 ticks is a minute.
bool next = false;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN, 
  .rst = 9,
  .dio = {2, 7, 8},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
#ifdef DEBUG
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
#endif
      // Schedule next transmission
      next = true;
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
#ifdef DEBUG
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
#endif
      break;
  }
}

void writeI2CRegister8bit(int addr, int value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

unsigned int readI2CRegister16bit(int addr, int reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(addr, 2);
  unsigned int t = Wire.read() << 8;
  t = t | Wire.read();
  return t;
}

unsigned int readI2CRegister8bit(int addr, int reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(addr, 1);
  unsigned int t = Wire.read();
  return t;
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
#ifdef DEBUG
    Serial.println(F("OP_TXRXPEND, not sending"));
#endif
  } else {
	  // Prepare upstream data transmission at the next possible time.
	  switchSensorPowerOn();
	  switchI2COn();
	  getMeasurements();
	  switchSensorPowerOff();
	  switchI2COff();
	  LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
#ifdef DEBUG
      Serial.println(F("Packet queued"));
#endif
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void getMeasurements() {
  int lumi, tempr, capa;
  int vcc;
  
  vcc = (int)(getVcc()*100.0);
  writeI2CRegister8bit(sensorAddress, 6); //reset Humidity sensor
  delay(5);
  writeI2CRegister8bit(sensorAddress, 0x03); //request light measurement
  delay(200);
  lumi = readI2CRegister16bit(sensorAddress, 0x04); //read light register
#ifdef DEVEL
  Serial.print(F("\nLight: "));
  Serial.println(lumi); //light measurement  
#endif
  tempr = readI2CRegister16bit(sensorAddress, 0x05); //temperature register
#ifdef DEVEL
  Serial.print(F("Temperature: "));
  Serial.println(tempr / 10.0); //temperature register
#endif
  capa = readI2CRegister16bit(sensorAddress, 0x00); //read capacitance register
#ifdef DEVEL
  Serial.print(F("Soil Moisture Capacitance: "));
  Serial.println(capa); // capacitance register
  Serial.print(F("Vcc level (0.01V/digit):"));
  Serial.println(vcc);
#endif
  writeI2CRegister8bit(sensorAddress, 0x08); //go to sleep Humidity sensor
  mydata[0] = 0x01; // data channel 1
  mydata[1] = 0x67; // data type temperature
  mydata[2] = (tempr >> 8) & 0xFF;
  mydata[3] = tempr & 0xFF;
  mydata[4] = 0x02;
  mydata[5] = 0x02; // data type analog input 2 bytes
  mydata[6] = (capa >> 8) & 0xFF;
  mydata[7] = capa & 0xFF;
  mydata[8] = 0x03;
  mydata[9] = 0x65; // data type illumination
  mydata[10] = (lumi >> 8) & 0xFF;
  mydata[11] = lumi & 0xFF;
  mydata[12] = 0x04;
  mydata[13] = 0x02; // data type analog input 2 bytes
  mydata[14] = (vcc >> 8) & 0xFF;
  mydata[15] = vcc & 0xFF;
}

float getVcc() {
  int sensorValue;
  switchAnalogOn();
  sensorValue = analogRead(PIN_VCC);
  switchAnalogOff();
  return (((sensorValue + 0.5)*3.85) / 1024.0);  //1.1V * 3.5 = 3.85
}

void switchSensorPowerOff() {
  digitalWrite(onPinI2C1, LOW);  
  delay(10); // Set delay of 10ms before switching to Input
  pinMode(onPinI2C1,INPUT);
}

void switchSensorPowerOn() {
  pinMode(onPinI2C1,OUTPUT);
  digitalWrite(onPinI2C1, HIGH);  
}

void switchAnalogOn() {
  pinMode(PD6,OUTPUT);
  digitalWrite(PD6,HIGH);
}

void switchAnalogOff() {
  pinMode(PD6,OUTPUT);
  digitalWrite(PD6,LOW);
}

void switchI2COn() {
  pinMode(A4,OUTPUT);
  pinMode(A5,OUTPUT);
  power_twi_enable();
}

void switchI2COff() {
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  power_twi_disable();
}

void setup() {
  Serial.begin(38400);
  Serial.println(F("Starting"));

  // Set reference voltage to internal 1.1V
  analogReference(INTERNAL);

  // Measure reference Vcc against internal reference voltage 1.1V
  Serial.print(F("Current Vcc level:"));
  Serial.println(getVcc());

  switchSensorPowerOn();
  switchI2COn();
  Wire.begin();
  Serial.println(F("Starting humidity sensor"));
  writeI2CRegister8bit(0x09, 6); //reset Humidity sensor
  delay(1000);
  Serial.print(F("I2C Soil Moisture Sensor Address: "));
  Serial.println(sensorAddress, HEX);
  Serial.print(F("Sensor Firmware version: "));
  Serial.println(readI2CRegister8bit(sensorAddress, 0x07), HEX);
  Serial.println();
  writeI2CRegister8bit(sensorAddress, 0x08); //go to sleep Humidity sensor
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void goToSleep() {
  extern volatile unsigned long timer0_overflow_count;
  int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
//#ifdef DEBUG
  Serial.print(F("Enter sleeping for "));
  Serial.print(sleepcycles);
  Serial.println(F(" cycles of 8 seconds"));
//#endif
  Serial.flush(); // give the serial print chance to complete
  power_usart0_disable();
  power_spi_disable();
  for (int i = 0; i < sleepcycles; i++) {
    // Enter power down state for 8 s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    // LMIC uses micros() to keep track of the duty cycle, so
    // hack timer0_overflow for a rude adjustment:
    cli();
    timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
    sei();
  }
  power_spi_enable();
  power_usart0_enable();
#ifdef DEBUG
  Serial.println(F("Sleep complete"));
#endif
}

void loop() {
  if (next == false) {
    os_runloop_once();
  } else {
    next = false;
    // Start job
    do_send(&sendjob);
    goToSleep();
  }
}
