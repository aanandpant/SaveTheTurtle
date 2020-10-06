#include <Adafruit_TPA2016.h>

#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TPA2016.h"
#include <OSCBundle.h>
#include <OSCMessage.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

#define VibPin 12

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

// for system ========================================================
bool motorState = false;
bool vibState = false;

char VibAddress[20] = "/vib_";
// ===================================================================

// for motor =========================================================
bool sweepBack = false;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long motorDelay = 15;
unsigned long motorTimer = 0;

Servo servo;
int angle = 0;
// ===================================================================

// for vibrator ======================================================
void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();
void vibrate(int pinNum, int half_wavelength, int vibrateTime);

int timer = 0;// timer base 1ms

int isVibrate = 0;
int vibrateFreq = 0;
int vibrateTime = 0;

unsigned long vibDelay = 10;
unsigned long vibTimer = 0;
unsigned long vibStart = 10;
int vibSecs = 10;

Adafruit_TPA2016 audioamp01 = Adafruit_TPA2016();
// ===================================================================

//wifi and osc setting ===============================================
int status = WL_IDLE_STATUS;
//char ssid[] = "NextInterfaces Lab";
//char pass[] = "nextinterfaces";
char ssid[] = "AR";
char pass[] = "nctuictar";
int keyIndex = 0;

IPAddress sendToUnityPC_Ip(192, 168, 0, 160);
unsigned int sendToUnityPC_Port = 8100;
unsigned int listenPort = 9100;

char packetBuffer[255];
char ReplyBuffer[] = "acknowledged";

WiFiUDP Udp_send;
WiFiUDP Udp_listen;
// ===================================================================




void setup() {
  //pinMode setting
  pinMode(VibPin, OUTPUT);

  initWifi();

  //interrupt setting
  startTimer(1000); //1ms  //1-> 1s //100->10ms

  //audioamp setting
  Serial.begin(9600);
  audioamp01.begin();
  audioamp01.enableChannel(true, true);
  audioamp01.setLimitLevelOff();
  audioamp01.setAGCCompression(TPA2016_AGC_OFF);

  Serial.println("Setup finished");
}

void loop() {
  OSC_in();

  //vibrator
  if ((millis() - vibTimer) >= vibDelay)
  {
    vibTimer = millis();
    if (vibState && (millis()-vibStart)<vibSecs*1000) {
      isVibrate = 1;
      vibrateFreq = 25;
      vibrateTime = 1000;
    }
    else {
      vibState = false;
      isVibrate = 0;
      vibrateFreq = 0;
      vibrateTime = 0;
    }
    audioamp01.setGain(30);
    vibrate(VibPin, vibrateFreq, vibrateTime);
  }
}

void OSC_in() {
  OSCMessage msgIn;
  int size;
  if ((size = Udp_listen.parsePacket()) > 0) {
    while (size--)
      msgIn.fill(Udp_listen.read());

    if (!msgIn.hasError()) {
      char temp[25];
      msgIn.getAddress(temp);
      //Serial.println(temp);
      int active = msgIn.getInt(0);
      if ( strcmp(temp, VibAddress) == 0 ) { //Vibrator
        vibState = (active > 0 ? true : false);
        if(vibState)
        {
          vibStart = millis();
          vibSecs = active;
        }
        Serial.println("Vib");
      }
    } else {
      //Serial.println("n");
    }
  }
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    if (isVibrate == 1)
      timer ++;
    else
      timer = 0;
    //if(timer>30000)
    //timer = 0;
  }
}

void vibrate(int pinNum, int half_wavelength, int vibrateTime) {
  if (timer < vibrateTime && isVibrate == 1) {
    if ((timer / half_wavelength) % 2 == 0)
      digitalWrite(pinNum, HIGH);
    else
      digitalWrite(pinNum, LOW);
  }
  else {
    isVibrate = 0;
  }
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void initWifi()
{
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8, 7, 4, 2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp_send.begin(sendToUnityPC_Port);
  Udp_listen.begin(listenPort);
}


void OSC_out(int val, char Address[], IPAddress dstIp, unsigned int dstPort) {
  OSCMessage msgOut(Address);
  msgOut.add(val);
  Udp_send.beginPacket(dstIp, dstPort);
  msgOut.send(Udp_send);
  Udp_send.endPacket();
  msgOut.empty();
}
void OSC_sendBack(int val, char Address[]) {
  //dstIp = ~WiFi.subnetMask() | WiFi.gatewayIP(); // broadcast
  OSCMessage msgOut(Address);
  msgOut.add(val);
  Udp_send.beginPacket(Udp_listen.remoteIP(), Udp_listen.remotePort());
  msgOut.send(Udp_send);
  Udp_send.endPacket();
  msgOut.empty();
}
