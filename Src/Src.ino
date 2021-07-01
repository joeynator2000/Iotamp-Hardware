#include "Arduino.h"
#include "heltec.h"

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <PZEM004T.h>
#include <HardwareSerial.h>
#include <TimeLib.h>

static const u1_t PROGMEM APPEUI[8]={ 0x65, 0x1A, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ 0xBE, 0x4E, 0x9C, 0xCD, 0x7F, 0x94, 0xAA, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x06, 0xC3, 0x99, 0x70, 0x83, 0x27, 0x36, 0x7B, 0x5E, 0x79, 0xB3, 0xFD, 0x0A, 0x14, 0xFA, 0x4D };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//variables
float storedEnergy = 0; //watt hours
float energyUsed = 0;
float kWhReading = 0;
boolean ledIsOn = false;
time_t lastTimeCheck = now();

HardwareSerial PzemSerial2(2);
PZEM004T pzem(&PzemSerial2, 13, 17);
IPAddress ip(192,168,1,1);

byte payload[4];

double getEnergyUsed()
{
  return energyUsed;
}

double getEnergyStored()
{
  return storedEnergy;
}

byte returnPayload()
{
  uint32_t energyUsed = getEnergyUsed() * 100;
  uint32_t energyStored = getEnergyStored() * 100;
  payload[0] = highByte(energyUsed);
  payload[1] = lowByte(energyUsed);
  payload[2] = highByte(energyStored);
  payload[3] = lowByte(energyStored);
}


static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).

const unsigned TX_INTERVAL = 60;

// pin mapping for v2
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 12,
  .dio = {26, 34, 35},
};

#define LED 25

String HexDownlink = "";

unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }

  return decValue;
}

void onEvent (ev_t ev) {
    //Serial.print(os_getTime());
    //Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_SCAN_TIMEOUT");
            Heltec.display->display();
            //Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_BEACON_FOUND");
            Heltec.display->display();
            //Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_BEACON_MISSED");
            Heltec.display->display();
            //Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_BEACON_TRACKED");
            Heltec.display->display();
            //Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_JOININGINED");
            Heltec.display->display();
            //Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_JOINED");
            Heltec.display->display();
            //Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            }
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_JOIN_FAILED");
            Heltec.display->display();
            //Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_REJOIN_FAILED");
            Heltec.display->display();
            //Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            if (LMIC.dataLen) {
                for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                        //Serial.print(F("0"));
                    }
                   HexDownlink += String(LMIC.frame[LMIC.dataBeg + i], HEX);
                }
                String receivedAmount = String(hexToDec(HexDownlink))+ " kWh";
                storedEnergy += (hexToDec(HexDownlink));
                Heltec.display->clear();
                Heltec.display->drawString(0, 1, "Received " + String(hexToDec(HexDownlink)) + "kWh. Thank you!");
                Heltec.display->display();
                HexDownlink = "";
                
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_LOST_TSYNC");
            Heltec.display->display();
            //Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_RESET");
            Heltec.display->display();
            //Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_RXCOMPLETE");
            Heltec.display->display();
            // data received in ping slot
            //Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_LINK_DEAD");
            Heltec.display->display();
            //Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_LINK_ALIVE");
            Heltec.display->display();
            //Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "EV_TXSTART");
            Heltec.display->display();
            //Serial.println(F("EV_TXSTART"));
            break;
        default:
            Heltec.display->clear();
            Heltec.display->drawString(0, 0, "Unknown event");
            Heltec.display->display();
            //Serial.print(F("Unknown event: "));
            //Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      Heltec.display->clear();
      Heltec.display->drawString(0, 0, "Packet not sending");
      Heltec.display->display();
       
    } else {
      Heltec.display->clear();
      Heltec.display->drawString(0, 0, "Sending");
      Heltec.display->display();
        returnPayload();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
      
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  while (true) {
       Heltec.display->drawString(0, 0, "Connecting to PZEM..."); 
       Heltec.display->display();
       if(pzem.setAddress(ip))
         break;
       delay(1000);
    }

  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  pinMode(LED, OUTPUT);
  delay(10);
        
  #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    if(energyUsed >= storedEnergy)
    {
      digitalWrite(LED,LOW);
      ledIsOn = false;
    } else {
      digitalWrite(LED,HIGH);
      ledIsOn = true;
    }
    
    time_t currentTime = now();
    if(lastTimeCheck < currentTime)
    {
      int difference = currentTime - lastTimeCheck;
      
      if(difference >= 60)
      {
        if(ledIsOn)
        {
          float wattHour = (pzem.energy(ip) >= 0 ) ? pzem.energy(ip) : 0;
          float wattMinute = wattHour / 60;
          energyUsed += wattMinute;
          lastTimeCheck = now();
          Heltec.display->clear();
          Heltec.display->drawString(0, 0, "Energy used: " + String(energyUsed));
          Heltec.display->display();
        }
      }
    }
    
    os_runloop_once();
}
