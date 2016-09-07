/* 
 FrSky Telemetry for Taranis via S.Port
 
 The UART TX-Pin is connected to Pin5 of the JR-Connector. FrSky uses inverted serial levels 
 and thus a inverter is needed in between. Example circuit can be found here:
 https://github.com/openLRSng/openLRSng/wiki/Telemetry-guide
 
 Almost all of the code was originally derived from https://github.com/openLRSng/
 */

#ifdef FRSKY_TELEMETRY

#include <AltSoftSerial.h>

#define SMARTPORT_INTERVAL 12000
#define SMARTPORT_BAUDRATE 57600

AltSoftSerial frskySerial(AltSoftSerial::TX, true);

uint32_t frskyLast = 0;
uint8_t frskySchedule = 0;
static uint32_t telemetry_ahead = 100;

float   telemetry_voltage = 0.f;
uint8_t telemetry_rssi    = 0.f;

void frskyInit()
{
  frskyLast = micros();
  frskySerial.begin(SMARTPORT_BAUDRATE);
}

void smartportSend(uint8_t *p)
{
  uint16_t crc = 0;
  frskySerial.write(0x7e);
  for (int i = 0; i < 9; i++) {
    if (i == 8) {
      p[i] = 0xff - crc;
    }
    if ((p[i] == 0x7e) || (p[i] == 0x7d)) {
      frskySerial.write(0x7d);
      frskySerial.write(0x20 ^ p[i]);
    } 
    else {
      frskySerial.write(p[i]);
    }
    if (i>0) {
      crc += p[i]; //0-1FF
      crc += crc >> 8; //0-100
      crc &= 0x00ff;
      crc += crc >> 8; //0-0FF
      crc &= 0x00ff;
    }
  }
}

void smartportIdle()
{
  frskySerial.write(0x7e);
}

void smartportSendFrame()
{
  uint8_t buf[9] = {0};
  
  uint8_t * bytes;
  uint32_t temp;
  uint32_t vtx_freq = 100;
  uint32_t accData[3] = {100,100,100};
  uint8_t telemetryVoltage = 100;

  //Serial.println(telemetry_voltage, 2);

  frskySchedule = (frskySchedule + 1) % 36;
  buf[0] = 0x98;
  buf[1] = 0x10;
  switch (frskySchedule) {
  case 0: // SWR (fake value = 0)
    buf[2] = 0x05;
    buf[3] = 0xf1;
    buf[4] = 0;
    //frskySerial.sendData(0xF105,0);
    break;
  case 1: // RSSI (fake value = 100)
    buf[2] = 0x01;
    buf[3] = 0xf1;
    buf[4] = telemetry_rssi ? telemetry_rssi : 1;
    //frskySerial.sendData(0xF101,100);
    break;
  case 2: //BATT  - rxBatt = ((uint8_t)data) * (3.3 / 255.0) * 4.0;
    buf[2] = 0x04;
    buf[3] = 0xf1;
    buf[4] = telemetry_voltage/4.f * 255.f/3.3f;
    //frskySerial.sendData(0xF104,100);
    break;
  case 3: //FCS VOLTAGE
    buf[2] = 0x10;
    buf[3] = 0x02;
    temp =  (uint32_t)(telemetry_voltage * 100);
    bytes = (uint8_t *) &temp;
    buf[4] = bytes[0];
    buf[5] = bytes[1];
    buf[6] = bytes[2];
    buf[7] = bytes[3];
    break;
  case 4: //RPM (use this as vTX frequency)
    buf[2] = 0x00;
    buf[3] = 0x05;
    temp = vtx_freq*2;
    bytes = (uint8_t *) &temp;
    buf[4] = bytes[0];
    buf[5] = bytes[1];
    buf[6] = bytes[2];
    buf[7] = bytes[3];
    //frskySerial.sendData(0x0500,temp);
    break;
  case 5: // ACC-ROLL
    buf[2] = 0x00;
    buf[3] = 0x07;
    #define ACC_1G 512
    temp = ((float)accData[0]/(ACC_1G*8))*1000;
    //bytes = (uint8_t *) &accData[0]; 
    bytes = (uint8_t *) &temp;
    buf[4] = bytes[0];
    buf[5] = bytes[1];
    buf[6] = bytes[2];
    buf[7] = bytes[3];
    //frskySerial.sendData(0x0700,temp);
    break;
  case 6: // ACC-PITCH
    buf[2] = 0x10;
    buf[3] = 0x07;
    bytes = (uint8_t *) &accData[1];
    buf[4] = bytes[0];
    buf[5] = bytes[1];
    buf[6] = bytes[2];
    buf[7] = bytes[3];
    //frskySerial.sendData(0x0710,accData[1]);
    break;
  case 7: // ACC-YAW
    buf[2] = 0x20;
    buf[3] = 0x07;
    bytes = (uint8_t *) &accData[2];
    buf[4] = bytes[0];
    buf[5] = bytes[1];
    buf[6] = bytes[2];
    buf[7] = bytes[3];
    //frskySerial.sendData(0x0720,accData[2]);
    break;
  default:
    smartportIdle();
    return;
  }
  smartportSend(buf);
}

void frskyUpdate()
{
  uint32_t now = micros();
  if ((now - frskyLast) > SMARTPORT_INTERVAL) {
    telemetry_ahead++;
    smartportSendFrame();
    frskyLast = now;
  }
}
#endif
