/* RN
 ##########################################
 #####   MultiProtocol nRF24L01 Tx   ######
 ##########################################
 #        by goebish on rcgroups          #
 #                                        #
 #   Parts of this project are derived    #
 #     from existing work, thanks to:     #
 #                                        #
 #   - PhracturedBlue for DeviationTX     #
 #   - victzh for XN297 emulation layer   #
 #   - Hasi for Arduino PPM decoder       #
 #   - hexfet, midelic, closedsink ...    #
 ##########################################


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

#include <util/atomic.h>
#include "iface_nrf24l01.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define RX 2
#define TX 5

SoftwareSerial Serial(RX, TX);

Servo servo;

// uncomment the below define to use nRF24_multipro as a receiver
// instead of a transmitter. you must also explicity set the
// 'current_protocol' variable below the the protocol you wish
// the receiver to use.
// Currently only the following RX protocols have been implemented:
// PROTO_BAYANG
#define RX_MODE

// ############ Wiring ################
#define PPM_pin   1  // PPM in

//SPI Comm.pins with nRF24L01
#define MOSI_pin  6  // MOSI - D6
#define SCK_pin   4  // SCK  - D4
#define CE_pin    3  // CE   - D3
#define MISO_pin  5  // MISO - D5
#define CS_pin    2  // CS   - D2

//#define ledPin    0  // LED  - D0

/* AtTiny44
pin port Arduino 
----------------
2  PB0 10
3  PB1  9
4  PB3 11
5  PB2  8
6  PA7  7
7  PA6  6

13 PA0  0
12 PA1  1
11 PA2  2
10 PA3  3
9  BA4  4
8  PA5  5
*/

// SPI outputs
#define MOSI_on PORTA |= _BV(6)  // PD3 x = x | y (PORTA = PORTA | B011
#define MOSI_off PORTA &= ~_BV(6)// PD3
#define SCK_on PORTA |= _BV(4)   // PD4
#define SCK_off PORTA &= ~_BV(4) // PD4
#define CE_on PORTA |= _BV(3)    // PD5
#define CE_off PORTA &= ~_BV(3)  // PD5
#define CS_on PORTA |= _BV(2)    // PC1
#define CS_off PORTA &= ~_BV(2)  // PC1
// SPI input
#define  MISO_on (PINA & _BV(0)) // PC0

#define RF_POWER TX_POWER_80mW

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// supported protocols
enum {
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,           // EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
    PROTO_SYMAXOLD,     // Syma X5C, X2
    PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
    PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    PROTO_END
};

// EEPROM locationss
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

uint8_t transmitterID[4];
uint8_t current_protocol = PROTO_BAYANG;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

void setup()
{
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(PPM_pin, INPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

#ifndef RX_MODE
    // PPM ISR setup
    attachInterrupt(PPM_pin - 2, ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

#endif
    set_txid(false);

    Serial.begin( 9600 );
    Serial.println( "Start" );
    servo.attach( 2 );
    pinMode( 6, OUTPUT );
}

void loop()
{
    uint32_t timeout=0;
    // reset / rebind
#ifndef RX_MODE
    if(reset || ppm[AUX8] > PPM_MAX_COMMAND) {
#else
    if(reset) {
#endif
        reset = false;
#ifndef RX_MODE
        selectProtocol();
#endif
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();
    }
    // process protocol
    switch(current_protocol) {
        case PROTO_BAYANG:
            timeout = process_Bayang();
            break;
    }
#ifndef RX_MODE
    // updates ppm values out of ISR
    update_ppm();
#endif
    // wait before sending next packet
    while(micros() < timeout)
    {   };
}

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]);
        }
    }
}

void selectProtocol()
{
    // wait for multiple complete ppm frames
    ppm_ok = false;
    uint8_t count = 10;
    while(count) {
        while(!ppm_ok) {} // wait
        update_ppm();
        if(ppm[AUX8] < PPM_MAX_COMMAND) // reset chan released
            count--;
        ppm_ok = false;
    }

    // startup stick commands

    if(ppm[RUDDER] < PPM_MIN_COMMAND)        // Rudder left
        set_txid(true);                      // Renew Transmitter ID

    // read last used protocol from eeprom
    else
        current_protocol = constrain(EEPROM.read(ee_PROTOCOL_ID),0,PROTO_END-1);
    // update eeprom
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    // wait for safe throttle
    while(ppm[THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        update_ppm();
    }
}

void init_protocol()
{
    Bayang_init();
    Bayang_bind();
}

// update ppm values out of ISR
void update_ppm()
{
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    }
}

void ISR_ppm()
{
    #if F_CPU == 16000000
        #define PPM_SCALE 1L
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L
    #else
        #error // 8 or 16MHz only !
    #endif
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    ppm_ok=false;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    }
    else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    }
    else{  //servo values between 510us and 2420us will end up here
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
            if(chan==3)
                ppm_ok = true; // 4 first channels Ok
        }
        chan++;
    }
}
