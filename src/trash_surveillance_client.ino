//-----INCLUDES-----
#include <Wire.h>
#include <HCSR04.h> //Distance sensor lib
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//-----CONSTANTS-----
#define TrigPin 45 //HCSR04 sensor trig pin
#define EchoPin 47 //HCSR04 sensor echo pin
#define MPU 0x68 //IMU (accelerometer) adress

#define MEASUREMENT_PERIOD_S 10 * 1000
#define LID_OPEN_RETRIES 10
uint32_t LID_OPEN_TIMER_S = 60 * 1000;

//-----FUNCTION PROTOTYPES-----
UltraSonicDistanceSensor distanceSensor(TrigPin, EchoPin);  //Initialize distance sensor

//-----GLOBAL VARIABLES-----
double dist, prevDist = 0;
int16_t AcX = 0; //MPU-6050 gives 16 bits integer values
uint16_t iteration = 0; //how long there was no update to server

bool error_height = 0; //distance sensor returned "-1" 100x  (KODA=1)
bool error_lid_open = 0; //accelerometer returned >4000 in y axes therefore lid is open
bool error_theft = 0; //distance changed for more than 20cm from previous measurement   (KODA=3)


// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x6F, 0x28, 0x0E, 0xD3, 0x43, 0x3D, 0x9C, 0x37, 0xF0, 0xC1, 0xD2, 0x0A, 0xC3, 0x58, 0x48, 0x28 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xD7, 0x54, 0x84, 0x76, 0xEC, 0x25, 0xA1, 0x12, 0x60, 0x77, 0xCF, 0x3B, 0x89, 0x9A, 0x9B, 0x44 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260111AA;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations)
const unsigned TX_INTERVAL = 5;

// library requires these
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

class loraClient {
  private:
  public:
    uint8_t payload_data[3] = {0, 0, 0};
    uint8_t payload_port = 0;

  void setErrorHeight() {
    payload_data[2] = payload_data[2] | 0x04;
  }

  void setErrorLidOpen() {
    payload_data[2] = payload_data[2] | 0x02;
  }

  void setErrorTheft() {
    payload_data[2] = payload_data[2] | 0x01;
  }

  void resetErrorHeight() {
    payload_data[2] = payload_data[2] & 0xFB;
  }

  void resetErrorLidOpen() {
    payload_data[2] = payload_data[2] & 0xFD;
  }

  void resetErrorTheft() {
    payload_data[2] = payload_data[2] & 0xFE;
  }

  void setHeight(double height) {
      if(height > 65536) {
      height = 0;
      Serial.println("Height overflow");
    }
    
    unsigned int r_height = static_cast<unsigned int> (height);
    Serial.print("Input height: ");
    Serial.println(r_height);
    uint8_t lsb = r_height & 0xFF;
    uint8_t msb = 0;

    if(r_height > 255) {
      msb = (r_height >> 1) & 0xFF;
    }
    
    Serial.print("msb: ");
    Serial.println(msb);
    Serial.print("lsb: ");
    Serial.println(lsb);

    payload_data[0] = msb;
    payload_data[1] = lsb;
  }

  void setPayloadData(double height, bool theft, bool lid_open, bool distance_sensor) {
    setHeight(height);
    payload_data[2] = 0x00;

    if(theft == true) {
      payload_data[2] = payload_data[2] | 0x01;
    }
    
    if(lid_open == true) {
      payload_data[2] = payload_data[2] | 0x02;
    }
    
    if(distance_sensor == true) {
      payload_data[2] = payload_data[2] | 0x04;
    }

    payload_port = 1;
    if(payload_data[2] > 0) {
      payload_port = 2;
    }
    
    Serial.print("alarm flags: ");
    Serial.println(payload_data[2]);
  }
    
} lora_client;

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println("OP_TXRXPEND, not sending");
  } else {
      // Prepare upstream data transmission at the next possible time.
      //LMIC_setTxData2(2, mydata, sizeof(mydata)-1, 0);
      LMIC_setTxData2(lora_client.payload_port, lora_client.payload_data, sizeof(lora_client.payload_data), 0);
      Serial.print("DataSize");
      Serial.println(sizeof(lora_client.payload_data));
      Serial.println("Packet queued");
      Serial.println(LMIC.freq);
  }
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.println(ev);
  switch(ev) {
    case EV_SCAN_TIMEOUT:
        Serial.println("EV_SCAN_TIMEOUT");
        break;
    case EV_BEACON_FOUND:
        Serial.println("EV_BEACON_FOUND");
        break;
    case EV_BEACON_MISSED:
        Serial.println("EV_BEACON_MISSED");
        break;
    case EV_BEACON_TRACKED:
        Serial.println("EV_BEACON_TRACKED");
        break;
    case EV_JOINING:
        Serial.println("EV_JOINING");
        break;
    case EV_JOINED:
        Serial.println("EV_JOINED");
        break;
    case EV_RFU1:
        Serial.println("EV_RFU1");
        break;
    case EV_JOIN_FAILED:
        Serial.println("EV_JOIN_FAILED");
        break;
    case EV_REJOIN_FAILED:
        Serial.println("EV_REJOIN_FAILED");
        break;
    case EV_TXCOMPLETE:
        Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
        if(LMIC.dataLen) {
          // data received in rx slot after tx
          Serial.print("Data Received: ");
          Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
          Serial.println();
        }

        lora_client.resetErrorTheft();

        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println("EV_LOST_TSYNC");
        break;
    case EV_RESET:
        Serial.println("EV_RESET");
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println("EV_RXCOMPLETE");
        break;
    case EV_LINK_DEAD:
        Serial.println("EV_LINK_DEAD");
        break;
    case EV_LINK_ALIVE:
        Serial.println("EV_LINK_ALIVE");
        break;
      default:
        Serial.println("Unknown event");
        break;
  }
}


void setup() 
{
  // accelerometer init
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  //i2c adress
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(9600);
  Serial.println("START SETUP");
  measurement();
  prevDist = dist;
  Serial.println("EXIT SETUP");

  // lora init

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  os_init();
  LMIC_reset();
#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7,14);

  // begin sending data
  lora_client.setPayloadData(65535.0, 1, 1, 1);
  do_send(&sendjob);
}

void loop() 
{
  measurement();
  Serial.print("Distance [cm] = ");
  Serial.println(dist);
  Serial.print("ACC Y = ");
  Serial.println(AcX); //AcX = Ker je pac koda z neta smotana, zato!
  Serial.print("Error1 = ");
  Serial.println(error_height);
  Serial.print("Error2 = ");
  Serial.println(error_lid_open);
  Serial.println();
  delay(1000);
/*  iteration++;
  delay(MEASUREMENT_PERIOD_S);
  measurement();
  os_runloop_once();

  // lid open
  if (error_lid_open) {
    for (int i = 0; i < LID_OPEN_RETRIES; i++) {
      delay(LID_OPEN_TIMER_S);
      measurement();
      if (!error_lid_open) {
        break;
      }
      if (i == LID_OPEN_RETRIES - 1) {
        lora_client.setErrorLidOpen();
      }
    }
  }

  // lid closed
  if(!error_lid_open) {
    if(error_height) {
      //pokrov zaprt + senzor razdalje ne dela
      lora_client.setErrorHeight();
    }
    else {
      
      if (prevDist - dist > 20.0) {
        // this error is always reset on lora tx callback
        lora_client.setErrorTheft();
        //error_theft = 1;
        //na server poslji dist + error_theft=1
        //dist naj bo pri posiljanju uint16_t
        //lora_client.resetErrorTheft();
        //error_theft = 0; // reset theft errora po posiljanju
      }

      lora_client.setHeight(dist);
      //previous measurement storage for theft detection
      prevDist = dist;
    }
  }
*/
}

//-----FUNCTION DEFINITIONS-----
void measurement() {
  lora_client.resetErrorHeight();
  lora_client.resetErrorLidOpen();
  //error_height = 0; //distance sensor returned "-1" 100x
  //error_lid_open = 0; //accelerometer returned <0 in y axes therefore lid is open
  dist = -1;

  updateAccel();
  if (AcX < 0) {
    error_lid_open = 1;
  }
  else {
    updateDist(); //prebere razdaljo v [cm]
  }
}

void updateAccel() {
   // read the acceleration values from the module
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); // ask for 0x3B register - correspond to AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //From 0x3B, we ask for 6 register
   AcX=Wire.read()<<8|Wire.read(); //Each values ahs 2 bytes
}

void updateDist() {
  int i, tmp, cnt1 = 0;

  while (cnt1 != 20) {
    tmp = distanceSensor.measureDistanceCm();
    if (tmp > 0) { //ce vrne -1 zavrzi meritev
      dist = dist + tmp;
      cnt1++;
    }
    i++;
    if (i == 100) break;
    delay(20);
  }
  if (dist == 0) {
    error_height = 1;
  }
  else {
    dist = round(dist / cnt1);
  }
}

