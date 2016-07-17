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

/*
 * Author: Mike Morrison (bikemikem on rcgroups)
 * Use this with the serial monitor. Commands can be sent to the
 * radio to configure the transceiver (channel, address, payload,
 * bitrate, crc, ptx, prx, etc).
 */

#ifdef ENABLE_PROMISC

static uint8_t    promisc_chan          = 0;
static uint8_t    promisc_addr[5]       = {0,0x55,0,0,0};
static uint8_t    promisc_addr_len      = 2;
static uint8_t    promisc_payload[32]   = {0};
static uint8_t    promisc_payload_len   = 32;
static uint16_t   promisc_tx_period     = 2000;
static uint8_t    promisc_bitrate       = NRF24L01_BR_1M;
static uint16_t   promisc_scan_ms       = 400;
static bool       promisc_scan          = true;
static bool       promisc_radio_off     = true;
static bool       promisc_rx_mode       = true;
static uint8_t    promisc_crc_len       = 0;
static uint8_t    promisc_crc_flags     = 0;
static bool       promisc_dpl           = false;
static bool       promisc_parse_esb     = false; // parse enhanced shockburst packet


uint32_t process_Promisc()
{
    uint32_t timeout = micros() + (promisc_rx_mode ? 1500 : promisc_tx_period);
    if (promisc_rx_mode)
    {
      Promisc_recv_packet();
    }
    else
    {
      Promisc_send_packet();
    }

    return timeout;
}


const uint8_t ssv7241_reg_init[] = 
  {
  0x1F,0x1,0x00,
  0x1B,0x4,0x10,0xE1,0xD3,0x3D,
  0x19,0x4,0x06,0xAA,0xA2,0xDB,
  0x1A,0x4,0x27,0x61,0x01,0xF8,
  0x1F,0x1,0x01,
  0x18,0x4,0xBF,0x94,0x00,0xDF,
  0x19,0x4,0x77,0x48,0x9A,0xE8,
  0x1B,0x4,0x76,0x87,0xCA,0x01,
  0x1F,0x1,0x02,
  0x1B,0x4,0xA0,0x00,0x18,0xA0,
  0x1F,0x1,0x04,
  0x18,0x4,0x01,0x00,0xF0,0x00,
  0x1F,0x1,0x05,
  0x18,0x4,0x84,0x03,0x2A,0x03,
  0x19,0x4,0x90,0xBF,0x00,0x00,
  0x1A,0x4,0xA0,0x0F,0x00,0x00,
  0x1F,0x1,0x00,
  };


void ssv_init_reg()
{
    for (int i = 0; i < sizeof(ssv7241_reg_init); )
    {
      uint8_t reg = ssv7241_reg_init[i++];
      uint8_t sz  = ssv7241_reg_init[i++];
      if (1 == sz)
      {
        NRF24L01_WriteReg(reg, ssv7241_reg_init[i]);
      }
      else
      {
        NRF24L01_WriteRegisterMulti(reg, ssv7241_reg_init+i, sz);
      }
      i += sz;
    }
}


void Promisc_init()
{ 
    NRF24L01_Initialize();
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, promisc_payload_len); // rx pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, promisc_addr_len-2);   // address size
    NRF24L01_SetTxRxMode(RX_EN);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, promisc_addr, promisc_addr_len);
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);
    NRF24L01_Activate(0x73);

    delay(150);

    promisc_crc_flags = 0;
    if (1 == promisc_crc_len)
    {
      promisc_crc_flags = _BV(NRF24L01_00_EN_CRC);
    }
    else if (2 == promisc_crc_len)
    {
      promisc_crc_flags = _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO);
    }
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, promisc_crc_flags | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x00);

    NRF24L01_SetTxRxMode(TXRX_OFF);
}

void Promisc_bind()
{
}

static void Promisc_print_help()
{
  Serial.println("NRF24L01+ PROMISCUOUS MODE");
  Serial.println("Options:");
  Serial.println("r  - toggle between rx mode and radio off");
  Serial.println("t  - toggle between tx mode and radio off");
  Serial.println("s  - toggle scanning mode");
  Serial.println("n  - next channel");
  Serial.println("p  - prev channel");
  Serial.println("c  - channel (0-125)");
  Serial.println("aN - address between 3 and 5 hex bytes");
  Serial.println("bN - bitrate 0 = 1mbps, 1=2mbps, 3=256kbps");
  Serial.println("dN - tx: payload data where N is a set of hex bytes");
  Serial.println("   - rx: payload data where N is length of packet");
  Serial.println("qN - crc size (0 = off)");
  Serial.println("v  - DPL dynamic (variable) payload length");
  Serial.println("x  - toggle parse enhanced shockburst packet when not in enhanced mode(crc must be set 0)");
  Serial.println("z  - SSV init reg");

}

static void Promisc_print_info()
{
    Serial.print(promisc_radio_off? "OFF " :" ON ");
    Serial.print(promisc_rx_mode? "RX " :"TX ");
    Serial.print("CHANNEL: ");
    Serial.print(promisc_chan, DEC);
    Serial.print(", ADDR: ");
    for (int i = 0; i < promisc_addr_len; ++i)
    {
      Serial.print(promisc_addr[i], HEX);
      Serial.print(" ");
    }

    if (0 == promisc_bitrate)
    {
      Serial.print(", 1mbps");
    }
    else if (1 == promisc_bitrate)
    {
      Serial.print(", 2mbps");
    }
    else if (2 == promisc_bitrate)
    {
      Serial.print(", 250kbps");
    }
    Serial.print(", len: ");
    Serial.print(promisc_payload_len, DEC);
    Serial.print(", crc len: ");
    Serial.print(promisc_crc_len, DEC);
    Serial.print(promisc_dpl ? ", dynamic payload" : ", static payload");
    Serial.println("");

    Serial.print("Payload: ");
    if (!promisc_parse_esb)
    {
      for (int i = 0; i < promisc_payload_len; ++i)
      {
        Serial.print(promisc_payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println("");
    }
}

static void Promisc_check_for_user_input()
{
  if (Serial.available() > 0)
  {
    Serial.println("Received some input");
    
    // format:  c[0-125]p[0-1]
    // example: c22p0 = channel 22, preamble 0x00,0x55
    // example: c22p1 = channel 22, preamble 0x00,0xAA

    int state = 0; // cmd type c or p chan/preamble, s = scan, b = bitrate, n = next chan, p = prev chan, 
    // state 1 = chan
    // state 2 = addr_id
    // state 3 = bitrate 0 = 1mbps, 1 = 2mbps, 2 = 250kbps
    // state 4 = payload data
    // state 5 = crc size
    uint8_t chan        = 0;
    bool gotchan = false;
    uint8_t addr_id = 0;

    uint8_t bitrate     = 0;
    bool gotbitrate = false;
    bool gotaddr = false;
    bool gotpayload  = false;
    int i = 0;

    uint8_t val = Serial.read();
    while (1)
    {
      if (0 == state)
      {
        if ('c' == val)
        {
          gotchan = true;
          state = 1;
          i = 0;
        }
        else if ('a' == val)
        {
          state = 2;
          gotaddr = true;
        }
        else if ('s' == val)
        {
          promisc_scan = !promisc_scan;
        }
        else if ('h' == val)
        {
          Promisc_print_help();
          break;
        }
        else if ('b' == val)
        {
          state = 3;
        }
        else if ('d' == val)
        {
          promisc_payload_len = 0;
          state = 4;
          gotpayload = true;
        }
        else if ('n' == val)
        {
          ++promisc_chan;
          promisc_chan %= 126;
          NRF24L01_WriteReg(NRF24L01_05_RF_CH, promisc_chan);
          NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
          NRF24L01_FlushRx();
          NRF24L01_FlushTx();
          
          promisc_scan = false;
        }
        else if ('v' == val)
        {
          promisc_dpl = !promisc_dpl;
          if (promisc_dpl)
          {
            NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x01);      // Enable dynamic payload P0
            NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x01);
            NRF24L01_WriteReg(NRF24L01_1D_FEATURE, _BV(NRF2401_1D_EN_DPL));
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
            NRF24L01_FlushRx();
            NRF24L01_FlushTx();
          }
          else
          {
            NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
            NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);
            NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
            NRF24L01_FlushRx();
            NRF24L01_FlushTx();
          }
        }
        else if ('z' == val)
        {
          ssv_init_reg();
        }
        else if ('x' == val)
        {
          promisc_parse_esb = !promisc_parse_esb;
        }
        else if ('p' == val)
        {
          promisc_scan = false;
          if (promisc_chan == 0)
            promisc_chan = 125;
          else
            promisc_chan--;

          NRF24L01_WriteReg(NRF24L01_05_RF_CH, promisc_chan);
          NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
          NRF24L01_FlushRx();
          NRF24L01_FlushTx();
        }
        else if ('r' == val)
        {
          if (promisc_radio_off || !promisc_rx_mode)
          {
            NRF24L01_SetTxRxMode(RX_EN);
            NRF24L01_WriteReg(NRF24L01_00_CONFIG, promisc_crc_flags | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
            NRF24L01_FlushRx();
            NRF24L01_FlushTx();
            promisc_radio_off = false;
            promisc_rx_mode = true;
          }
          else 
          {
            NRF24L01_SetTxRxMode(TXRX_OFF);
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
            NRF24L01_FlushRx();
            NRF24L01_FlushTx();
            promisc_radio_off = true;
          }
          
        }
        else if ('t' == val)
        {
          // toggle tx on/off
          if (promisc_radio_off || promisc_rx_mode)
          {
            if (promisc_rx_mode)
              NRF24L01_SetTxRxMode(TXRX_OFF);
            NRF24L01_SetTxRxMode(TX_EN);
            
            NRF24L01_WriteReg(NRF24L01_00_CONFIG, promisc_crc_flags | _BV(NRF24L01_00_PWR_UP) );
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
            NRF24L01_FlushRx();
            NRF24L01_FlushTx();
            promisc_radio_off = false;
            promisc_rx_mode = false;
          }
          else 
          {
            NRF24L01_SetTxRxMode(TXRX_OFF);
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
            NRF24L01_FlushRx();
            NRF24L01_FlushTx();
            promisc_radio_off = true;
          }
        }
        else if ('q' == val)
        {
          state = 5;
        }
      }
      else if (1 == state)
      {
        if ('0' <= val && val <= '9')
        {
          if (0 != i)
            chan = chan * 10;
          chan += val - '0';
          ++i;
        }
        else
        {
          state = 0;
          i = 0;
          continue;
        }
      }
      else if (2 == state)
      { // address
        uint8_t pos = i / 2;
        uint8_t nib = i % 2;
        uint8_t n = 0;
        if ('0' <= val && val <= '9')
        {
          n = val - '0';
          promisc_addr[pos] = (promisc_addr[pos] & (nib == 0 ? 0x0F : 0xF0)) + (nib == 0 ? (n << 4) : n);
          promisc_addr_len = pos + 1;
          ++i;
        }
        else if ('a' <= val && val <= 'f')
        {
          n = val - 'a' + 10;
          promisc_addr[pos] = (promisc_addr[pos] & (nib == 0 ? 0x0F : 0xF0)) + (nib == 0 ? (n << 4) : n);
          promisc_addr_len = pos + 1;
          ++i;
        }
        else if ('A' <= val && val <= 'F')
        {
          n = val - 'A' + 10;
          promisc_addr[pos] = (promisc_addr[pos] & (nib == 0 ? 0x0F : 0xF0)) + (nib == 0 ? (n << 4) : n);
          promisc_addr_len = pos + 1;
          ++i;
        }
        else
        {
          i = 0;
          state = 0;
          continue;
        }          
      }
      else if (3 == state)
      {
        if ('0' <= val && val <= '2')
        {
          bitrate = val - '0';
          if (0 == bitrate)
          {
            NRF24L01_SetBitrate(NRF24L01_BR_1M);
          }
          else if (1 == bitrate)
          {
            NRF24L01_SetBitrate(NRF24L01_BR_2M);
          }
          else if (2 == bitrate)
          {
            NRF24L01_SetBitrate(NRF24L01_BR_250K);
          }
          NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
          NRF24L01_FlushRx();
          NRF24L01_FlushTx();
  
          promisc_bitrate = bitrate;

        }
        
      }
      else if (4 == state)
      {
        if (!promisc_rx_mode)
        {
          uint8_t pos = i / 2;
          uint8_t nib = i % 2;
          uint8_t n = 0;
          Serial.print("i = ");
          Serial.print(i, DEC);
          Serial.print(" pos = ");
          Serial.print(pos, DEC);
          Serial.print("nib = ");
          Serial.print(nib, DEC);
          Serial.println();
          if ('0' <= val && val <= '9')
          {
            n = val - '0';
            promisc_payload[pos] = (promisc_payload[pos] & (nib == 0 ? 0x0F : 0xF0)) + (nib == 0 ? (n << 4) : n);
            promisc_payload_len = pos + 1;
            ++i;
          }else if ('a' <= val && val <= 'f')
          {
            n = val - 'a' + 10;
            promisc_payload[pos] = (promisc_payload[pos] & (nib == 0 ? 0x0F : 0xF0)) + (nib == 0 ? (n << 4) : n);
            promisc_payload_len = pos + 1;
            ++i;
          }
          else if ('A' <= val && val <= 'F')
          {
            n = val - 'A' + 10;
            promisc_payload[pos] = (promisc_payload[pos] & (nib == 0 ? 0x0F : 0xF0)) + (nib == 0 ? (n << 4) : n);
            promisc_payload_len = pos + 1;
            ++i;
          }
          else
          {
            i = 0;
            state = 0;
            continue;
          }
        }
        else
        {
          if ('0' <= val && val <= '9')
          {
            if (0 != i)
              promisc_payload_len = promisc_payload_len * 10;
            promisc_payload_len += val - '0';
            ++i;
          }
          else
          {
            state = 0;
            i = 0;
            continue;
          }          
        }
      }
      else if (5 == state)
      {
        if ('0' <= val && val <= '2')
          {
            promisc_crc_len = val - '0';
            promisc_crc_flags = 0;
            if (1 == promisc_crc_len)
            {
              promisc_crc_flags = _BV(NRF24L01_00_EN_CRC);
            }
            else if (2 == promisc_crc_len)
            {
              promisc_crc_flags = _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO);
            }
            if (!promisc_radio_off)
            {
              if (promisc_rx_mode)
              {
                NRF24L01_WriteReg(NRF24L01_00_CONFIG, promisc_crc_flags | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();
              }
              else
              {
                NRF24L01_WriteReg(NRF24L01_00_CONFIG, promisc_crc_flags | _BV(NRF24L01_00_PWR_UP));
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushTx();
              }
            }
          }
          else
          {
            state = 0;
            i = 0;
            continue;
          }
      }

      delay(5);

      if (Serial.available())
      {
        val = Serial.read();
      }
      else
      {
        if (gotchan)
        {
          if (0 <= chan && chan < 126)
          {
            promisc_chan = chan;
            NRF24L01_WriteReg(NRF24L01_05_RF_CH, chan);
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
            NRF24L01_FlushRx();
            NRF24L01_FlushTx();
            promisc_scan = false;
          }
        }

        if (gotaddr)
        {
  
          NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, promisc_addr_len-2);
          NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, promisc_addr, promisc_addr_len);
          NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, promisc_addr, promisc_addr_len);
          NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
          NRF24L01_FlushRx();
          NRF24L01_FlushTx();
        }
        if (gotpayload)
        {
          NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, promisc_payload_len); // rx pipe 0
          NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
          NRF24L01_FlushRx();
        }
        Promisc_print_info();
        break;
      }
    }
  }
}


void Promisc_recv_packet()
{
  static bool first = true;
  static bool scan_started = false;

  static uint32_t next_change_ms = 0;

  if (first)
  {
    first = false;
    Promisc_print_help();
  }

  Promisc_check_for_user_input();

  if (promisc_radio_off)
    return;

  if (!promisc_scan)
  {
    scan_started = false;
  }
  else if (next_change_ms < millis())
  {
    next_change_ms = millis() + promisc_scan_ms;

    if (scan_started)
    {
      ++promisc_chan;
      promisc_chan %= 126;
    }
    
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, promisc_chan);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();

    Promisc_print_info();

    scan_started = true;
  }

  if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) 
  { // data received from tx

    int sum = 0;
    uint16_t roll, pitch, yaw, throttle;
    NRF24L01_ReadPayload(packet, promisc_payload_len);
    
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();

    Serial.print("PACKET RECV: ");
    if (!promisc_parse_esb || promisc_payload_len <= 2)
    {
      for (int i = 0; i < promisc_payload_len; ++i)
      {
        Serial.print(packet[i], HEX);
        Serial.print(" ");
      }
    }
    else
    {
      // first 9 bits are packet control field
      // 6 payload len
      // 2 pid
      // 1 noack

      union {
        u16 value;
        struct {
          u8 lsb;
          u8 msb;
        } bytes;
      } val;

      uint8_t len_pid = packet[0];
      uint8_t noack   = (packet[1] >> 7);
      Serial.print("len: ");
      Serial.print((len_pid >> 2), DEC);
      Serial.print(", pid: ");
      Serial.print((len_pid &0x3), DEC);
      Serial.print(", nack: ");
      Serial.print((noack >> 7), DEC);
      Serial.print(", data: ");
      for (int i = 1; i < promisc_payload_len-1; ++i)
      {
        // each packet is shifted by one to the right
        val.bytes.msb = packet[i];
        val.bytes.lsb = packet[i+1];
        val.value = val.value << 1;
        
        Serial.print(val.bytes.msb, HEX);
        Serial.print(" ");
      }
    }

    Serial.println(" ");
   }
}

void Promisc_send_packet()
{

  Promisc_check_for_user_input();
  
  if (promisc_radio_off)
    return;

  NRF24L01_WriteReg(NRF24L01_00_CONFIG, promisc_crc_flags | _BV(NRF24L01_00_PWR_UP));
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, promisc_chan);
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
  NRF24L01_FlushTx();
  NRF24L01_WritePayload(promisc_payload, promisc_payload_len);
}
#endif // ENABLE_PROMISC

