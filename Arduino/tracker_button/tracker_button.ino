#include <Wire.h>
#include <OSCBundle.h>
#include <OSCMessage.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

#define ButtonPin 12
#define signalPIN 13

int buttonState = 0;
int lastbuttonState = 0;

// ID -----------------------------------------------
int id_ = 1;
// --------------------------------------------------
//wifi setting-----------------------------------------------
//osc
int status = WL_IDLE_STATUS;
//char ssid[] = "NextInterfaces Lab";
//char pass[] = "nextinterfaces";
char ssid[] = "AR";
char pass[] = "nctuictar";

IPAddress dstIp(140, 113, 30, 135);
unsigned int dstPort = 8100;
unsigned int localPort = 9100;

char targetAddress[30] = "";
char localAddress[30] = ""; // the receiver address
char deviceAddress[] = "/button"; // the start of address(the type of device) like "/light" or "/vib"

//char packetBuffer[255];
//char ReplyBuffer[] = "acknowledged";

WiFiUDP Udp;
//-----------------------------------------------------------

void setup() {
  //pinMode setting
  pinMode(ButtonPin, INPUT);
  
  Serial.begin(9600);
  
  initWifi();
  
  OSC_setup();
  
  Serial.println("Setup finished");
}
 
void loop() {
  buttonState = digitalRead(ButtonPin);
  if(buttonState != lastbuttonState){
    //send message when button release
    Serial.println("Button stage change!");
    if(buttonState)
    {
      //OSC send
      OSC_out(1, localAddress);
      Serial.println("Button released!");
    }
    else
    {
      //OSC send
      OSC_out(0, localAddress);
      Serial.println("Button released!");
    }
  }
  lastbuttonState = buttonState;
  delay(50);
//  Serial.print(buttonState);
//  Serial.print(", ");
//  Serial.println(lastbuttonState);
}


void OSC_setup() {
  
  //Serial.begin(9600);
  
  pinMode(signalPIN, OUTPUT);
  digitalWrite(signalPIN, HIGH);
  initWifi();
  digitalWrite(signalPIN, LOW);

  strcpy(localAddress, deviceAddress);

  Serial.println();
  Serial.print("local Address = ");
  Serial.println(localAddress);
  
}

void initWifi(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8, 7, 4, 2);
  //Initialize serial and wait for port to open:

  /*
  while (!Serial){
  ; // wait for serial port to connect. Needed for native USB port only
  }
  */
  
  //  check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD){
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 30 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void printWifiStatus(){
  Serial.println("loopING");
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

void OSC_out(int val, char Address[]){
  //oscOUT
  dstIp = ~WiFi.subnetMask() | WiFi.gatewayIP(); // broadcast
  OSCMessage msgOut(Address);
  msgOut.add(val);
  Udp.beginPacket(dstIp, dstPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}
void OSC_out(int val){
  OSC_out(val, localAddress);
}
