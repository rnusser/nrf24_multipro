/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#define BAYANG_BIND_COUNT       1000
#ifdef TELEMETRY_ENABLED
#define BAYANG_PACKET_PERIOD    3600
#else
#define BAYANG_PACKET_PERIOD    2000
#endif
#define BAYANG_PACKET_SIZE      15
#define BAYANG_RF_NUM_CHANNELS  4
#define BAYANG_RF_BIND_CHANNEL  0
#define BAYANG_ADDRESS_LENGTH   5

#ifndef FRSKY_TELEMETRY
float   telemetry_voltage;
uint8_t telemetry_rssi;
uint8_t  telemetry_datamode;
float    telemetry_data[3]; // pids
uint8_t  telemetry_dataitem;
uint16_t telemetry_uptime;
uint16_t telemetry_flighttime;
uint8_t  telemetry_flightmode;
#endif

#ifdef TELEMETRY_ENABLED
extern float    telemetry_voltage;
extern uint8_t  telemetry_rssi;
extern uint8_t  telemetry_datamode;
extern float    telemetry_data[3]; // pids
extern uint8_t  telemetry_dataitem;
extern uint16_t telemetry_uptime;
extern uint16_t telemetry_flighttime;
extern uint8_t  telemetry_flightmode;

uint8_t telemetry_rssi_count;
uint8_t telemetry_rssi_next;
#endif

static uint8_t Bayang_rf_chan;
static uint8_t Bayang_rf_channels[BAYANG_RF_NUM_CHANNELS] = {0,};
static uint8_t Bayang_rx_tx_addr[BAYANG_ADDRESS_LENGTH];

enum{
    // flags going to packet[2]
    BAYANG_FLAG_RTH      = 0x01,
    BAYANG_FLAG_HEADLESS = 0x02,
#ifdef TELEMETRY_ENABLED
  BAYANG_FLAG_TELEMETRY = 0x04,
#endif
    BAYANG_FLAG_FLIP     = 0x08,
    BAYANG_FLAG_VIDEO    = 0x10,
    BAYANG_FLAG_SNAPSHOT = 0x20,
};

enum{
    // flags going to packet[3]
    BAYANG_FLAG_INVERT   = 0x80,
#ifdef TELEMETRY_ENABLED
  BAYANG_FLAG_FLIGHT_MODE0 = 0x01,
  BAYANG_FLAG_FLIGHT_MODE1 = 0x02,
  BAYANG_FLAG_DATA_SELECT0 = 0x04,
  BAYANG_FLAG_DATA_SELECT1 = 0x08,
  BAYANG_FLAG_DATA_SELECT2 = 0x10,
  BAYANG_FLAG_DATA_ADJUST0 = 0x20,
  BAYANG_FLAG_DATA_ADJUST1 = 0x40,
#endif
};

uint32_t process_Bayang()
{
#ifdef TELEMETRY_ENABLED
  uint32_t timeout = micros(); // no delay
  Bayang_process();
#else
    uint32_t timeout = micros() + BAYANG_PACKET_PERIOD;
    Bayang_send_packet(0);
#endif
    return timeout;
}

void Bayang_init()
{
    uint8_t i;
  const u8 bind_address[] = {0, 0, 0, 0, 0};

  Bayang_rx_tx_addr[0] = transmitterID[0];
  Bayang_rx_tx_addr[1] = transmitterID[1];
  Bayang_rx_tx_addr[2] = transmitterID[2];
  Bayang_rx_tx_addr[3] = transmitterID[3];
  Bayang_rx_tx_addr[4] = transmitterID[3]; // transmitter id only has 4 so dupl the last id

  Bayang_rf_channels[0] = 0x00;
  for (i = 1; i < BAYANG_RF_NUM_CHANNELS; i++) {
    Bayang_rf_channels[i] = transmitterID[i] % 0x42;
  }
  NRF24L01_Initialize();
  NRF24L01_SetTxRxMode(TX_EN);
  XN297_SetTXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
  XN297_SetRXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
  NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, BAYANG_PACKET_SIZE); // rx pipe 0
  NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
  NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
  NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
  NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
  NRF24L01_SetPower(RF_POWER);
  NRF24L01_Activate(0x73);                         // Activate feature register
  NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
  NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
  NRF24L01_Activate(0x73);
  delay(150);
}

void Bayang_bind()
{
    uint16_t counter = BAYANG_BIND_COUNT;
  while (counter) {
        Bayang_send_packet(1);
        delayMicroseconds(BAYANG_PACKET_PERIOD);
        digitalWrite(ledPin, counter-- & 0x10);
    }
    XN297_SetTXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
  XN297_SetRXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    digitalWrite(ledPin, HIGH);
}

#define DYNTRIM(chval) ((u8)((chval >> 2) & 0xfc))

#ifdef DEBUG
static uint32_t tx_time = 0;
#endif

uint8_t Bayang_recv_packet()
{
  uint8_t received = 0;
  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
  {
    int sum = 0;
    uint16_t roll, pitch, yaw, throttle;
    uint32_t read_time = micros();
    XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);
    read_time = micros() - read_time;

    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();

    if (packet[0] == 0xA9)
    {
      //Serial.print("data packet");
      for (int i = 0; i < 14; i++)
      {
        sum += packet[i];
      }
      if ( (sum & 0xFF) == packet[14] )
      {
#ifdef DEBUG
        static uint32_t packet_received = 0;
        static uint32_t min_tx_time = 0xFFFFFFFF;
        static uint32_t max_tx_time = 0;
        static uint32_t avg_tx_time = 0;
        ++packet_received;
        static uint32_t last_micros = 0;
        uint32_t time_now = micros();


        if (min_tx_time > time_now - tx_time)
        {
          min_tx_time = time_now - tx_time;
        }
        if (max_tx_time < time_now - tx_time)
        {
          max_tx_time = time_now - tx_time;
        }

        avg_tx_time += time_now - tx_time;
#endif // DEBUG

        typedef union {
          uint16_t v;
          uint8_t  bytes[2];
        } val;
        val v;

        v.bytes[0] = packet[1];
        v.bytes[1] = packet[2];
        telemetry_voltage = v.v / 100.f;

        v.bytes[0] = packet[3];
        v.bytes[1] = packet[4];
        telemetry_uptime = v.v;

        v.bytes[0] = packet[5];
        v.bytes[1] = packet[6];
        telemetry_flighttime = v.v;

        telemetry_flightmode = packet[7] & 0x3; // 0 = level, 1 = acro,
        telemetry_datamode   = (packet[7] >> 2) & 0xF;  // (0=acro yaw, 1=acro roll/acro pitch, 2=level roll/pitch)
        telemetry_dataitem   = (packet[7] >> 6) & 0x3;  // (0=acro yaw, 1=acro roll/acro pitch, 2=level roll/pitch)

        v.bytes[0] = packet[8];
        v.bytes[1] = packet[9];
        telemetry_data[0] = v.v/1000.f;

        v.bytes[0] = packet[10];
        v.bytes[1] = packet[11];
        telemetry_data[1] = v.v/1000.f;

        v.bytes[0] = packet[12];
        v.bytes[1] = packet[13];
        telemetry_data[2] = v.v/1000.f;

#ifdef DEBUG
        if (time_now - last_micros > 1000000) // 1 second
        {
          last_micros = time_now;
          avg_tx_time /= packet_received;

          Serial.print("TELEMETRY [ ");
          Serial.print(packet_received, DEC);

          Serial.print(" pps, ");
          Serial.print(min_tx_time, DEC);
          Serial.print("us -> ");
          Serial.print(max_tx_time, DEC);
          Serial.print("us, avg");
          Serial.print(avg_tx_time, DEC);
          Serial.print("us, : ");
          Serial.print(read_time, DEC);
          Serial.print("us]: ");
          Serial.print(telemetry_voltage, 3);
          Serial.print("v, uptime");
          Serial.print(telemetry_uptime, DEC);
          Serial.print("s, ftime");
          Serial.print(telemetry_flighttime, DEC);
          Serial.print("s, flight mode");
          Serial.print(telemetry_flightmode, DEC);
          Serial.print(", data mode:");
          Serial.print(telemetry_datamode, DEC);
          Serial.print(", P:");
          Serial.print(telemetry_data[0], 10);
          Serial.print(", I:");
          Serial.print(telemetry_data[1], 10);
          Serial.print(", D:");
          Serial.print(telemetry_data[2], 10);
          Serial.println("");

          min_tx_time = 0xFFFFFFFF;
          max_tx_time = 0;
          packet_received = 0;
          avg_tx_time = 0;
        }
#endif // DEBUG
      }
    }
    else
    {
#ifdef DEBUG
      /*
      Serial.print("Other packet");
      for (int i = 0; i < 14; i++)
      {
        Serial.print(packet[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      */
#endif //DEBUG
    }
    received = 1;
  }
  return received;
}

void Bayang_send_packet(u8 bind)
{
  union {
      u16 value;
      struct {
          u8 lsb;
          u8 msb;
      } bytes;
  } chanval;

  if (bind) {
      packet[0] = 0xa4;
      memcpy(&packet[1], Bayang_rx_tx_addr, 5);
      memcpy(&packet[6], Bayang_rf_channels, 4);
      packet[10] = transmitterID[0];
      packet[11] = transmitterID[1];
  }
  else 
  {
    packet[0] = 0xa5;
    packet[1] = 0xfa;   // normal mode is 0xf7, expert 0xfa
    packet[2] = GET_FLAG(AUX2, BAYANG_FLAG_FLIP)
#ifndef TELEMETRY_ENABLED
                | GET_FLAG(AUX5, BAYANG_FLAG_HEADLESS)
                | GET_FLAG(AUX6, BAYANG_FLAG_RTH)
#else // repurpose AUX5 and AUX6
                | BAYANG_FLAG_TELEMETRY
#endif
                | GET_FLAG(AUX3, BAYANG_FLAG_SNAPSHOT)
                | GET_FLAG(AUX4, BAYANG_FLAG_VIDEO);
#ifndef TELEMETRY_ENABLED
    packet[3] = GET_FLAG(AUX1, BAYANG_FLAG_INVERT);
#else // use aux2 instead
    packet[3] = GET_FLAG_INV(AUX2, BAYANG_FLAG_INVERT);
#endif

#ifdef TELEMETRY_ENABLED
    static uint8_t dataselect = 2;
    uint8_t dataselect_old = dataselect;
    uint16_t partitions[4] ={1200,1400,1600,1800}; // 5 options (previous data set, data 1, data 2, data 3, next data set)
    for (uint8_t i = 0; i < 4; ++i)
    {
      int16_t hysteresis = 0;
      if (dataselect_old*2 == (i*2+1) - 1)
      {
        hysteresis = 25;
      }
      else if (dataselect_old*2 == (i*2+1) + 1)
      {
        hysteresis = -25;
      }

      if (ppm[AUX5] <= partitions[i] + hysteresis)
      {
        dataselect = i;
        break;
      }
      else
      {
        dataselect = i+1;
      }
    }


    // data adjust 1333  1666 - aux 6
    static uint8_t dataadjust = 1;
    uint8_t dataadjust_old = dataadjust;
    partitions[0] = 1333; // three options (decreaes, do nothing, increase)
    partitions[1] = 1666;
    for (uint8_t i = 0; i < 2; ++i)
    {
      int16_t hysteresis = 0;
      if (dataadjust_old*2 == (i*2+1) - 1)
      {
        hysteresis = 25;
      }
      else if (dataadjust_old*2 == (i*2+1) + 1)
      {
        hysteresis = -25;
      }

      if (ppm[AUX6] <= partitions[i] + hysteresis)
      {
        dataadjust = i;
        break;
      }
      else
      {
        dataadjust = i+1;
      }
    }

    // flight mode 1250 1500 1750 - aux 1
    static uint8_t flightmode = 1;
    uint8_t flightmode_old = flightmode;
    partitions[0] = 1250;  // 4 flight modes
    partitions[1] = 1500;
    partitions[2] = 1750;
    for (uint8_t i = 0; i < 3; ++i)
    {
      int16_t hysteresis = 0;
      if (flightmode_old*2 == (i*2+1) - 1)
      {
        hysteresis = 25;
      }
      else if (flightmode_old*2 == (i*2+1) + 1)
      {
        hysteresis = -25;
      }

      if (ppm[AUX1] <= partitions[i] + hysteresis)
      {
        flightmode = i;
        break;
      }
      else
      {
        flightmode = i+1;
      }
    }

    packet[3] |= (flightmode & 0x3);
    packet[3] |= (dataselect & 0x7) << 2;
    packet[3] |= (dataadjust & 0x3) << 5;
#endif

    chanval.value = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0, 0x3ff);   // aileron
    packet[4] = chanval.bytes.msb + DYNTRIM(chanval.value);
    packet[5] = chanval.bytes.lsb;
    chanval.value = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0, 0x3ff);   // elevator
    packet[6] = chanval.bytes.msb + DYNTRIM(chanval.value);
    packet[7] = chanval.bytes.lsb;
    chanval.value = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 0x3ff);   // throttle
    packet[8] = chanval.bytes.msb + 0x7c;
    packet[9] = chanval.bytes.lsb;
    chanval.value = map(ppm[RUDDER], PPM_MIN, PPM_MAX, 0, 0x3ff);   // rudder
    packet[10] = chanval.bytes.msb + DYNTRIM(chanval.value);
    packet[11] = chanval.bytes.lsb;
  }

  packet[12] = transmitterID[2];
  packet[13] = 0x0a;
  packet[14] = 0;
  for(uint8_t i=0; i<BAYANG_PACKET_SIZE-1; i++) {
      packet[14] += packet[i];
  }
  
  XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? BAYANG_RF_BIND_CHANNEL : Bayang_rf_channels[Bayang_rf_chan++]);
  Bayang_rf_chan %= sizeof(Bayang_rf_channels);
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
  NRF24L01_FlushTx();
  XN297_WritePayload(packet, BAYANG_PACKET_SIZE);
}

typedef enum
{
  BAYANG_STATE_IDLE,
  BAYANG_STATE_TRANSMITTING,
  BAYANG_STATE_RECEIEVING,
} BayangState;

BayangState Bayang_state = BAYANG_STATE_IDLE;
uint32_t Bayang_next_send = 0;

void Bayang_process()
{
  uint32_t time_micros = micros();

  if (BAYANG_STATE_TRANSMITTING == Bayang_state)
  {
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_TX_DS))
    {
      // send finished, switch to rx to receive telemetry

      //NRF24L01_SetTxRxMode(RX_EN);
      XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
      NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
      NRF24L01_FlushTx();
      NRF24L01_FlushRx();

      telemetry_rssi_count++;

      Bayang_state = BAYANG_STATE_RECEIEVING;
    }
  }
  else if (BAYANG_STATE_RECEIEVING == Bayang_state)
  {
    // 250us is about the time it takes to read a packet over spi
    if (time_micros > (Bayang_next_send-250))
    {
      // didnt receive telemetry in time
      Bayang_state = BAYANG_STATE_IDLE;
    }
    else if (Bayang_recv_packet())
    {
      telemetry_rssi_next++;
      // received telemetry packet
    }
    else
    {
      return;
    }

    if (100 == telemetry_rssi_count)
    {
      telemetry_rssi = telemetry_rssi_next;
      telemetry_rssi_next = 0;
      telemetry_rssi_count = 0;
    }

    Bayang_state = BAYANG_STATE_IDLE;
    //NRF24L01_SetTxRxMode(TX_EN);
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_FlushRx();
  }
  else if (time_micros > Bayang_next_send)
  {
#ifdef DEBUG
    tx_time = micros();
#endif
    Bayang_send_packet(0);
    Bayang_state = BAYANG_STATE_TRANSMITTING;
    Bayang_next_send = time_micros + BAYANG_PACKET_PERIOD;
  }
}
