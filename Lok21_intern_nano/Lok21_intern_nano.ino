

/* Lok21_intern
 * Version 0.0
 * 
 */
#include <SPI.h>
#include <NRFLite.h>            // https://github.com/dparson55/NRFLite
#include <Wire.h>
#include <Packet.h>
#include <PacketCRC.h>
#include <SerialTransfer.h>     // https://github.com/PowerBroker2/SerialTransfer



//Definitionen der Pinnummern

// Definitionen für Funkverbindung mit NRF24
const static uint8_t RADIO_ID = 3;              // Our radio's id.  The transmitter will send to this id.
const static uint8_t DESTINATION_RADIO_ID = 0;  //
const static uint8_t PIN_RADIO_CE = 7;         // Mega: 18, Nano: 6
const static uint8_t PIN_RADIO_CSN = 8;        // Mega: 53, Nano: 7
unsigned long RXOutTime = 2000;  // Karrenzzeit für Ausfall Empfang

//const static uint32_t MaxFailedTXCount = 30; // Maximalzahl der fehlgeschlagenen Übertragungen
enum RadioPacketType {
  AcknowledgementData,  // Produced by the primary receiver and provided to the transmitter via an acknowledgement data packet.
  Heartbeat,            // Sent by the primary transmitter. (not used)
  BeginGetData,         // Sent by the primary transmitter to tell the receiver it should load itself with an acknowledgement data packet.
  EndGetData            // Sent by the primary transmitter to receive the acknowledgement data packet from the receiver.
};

struct RadioPacketTX  // Paket von Lok
{
  RadioPacketType PacketType;
  int FromRadioId;
  int kmh;
  int Volt;
  int Ampere;
};
struct RadioPacketRX  // Paket vom Fahrpult
{
  RadioPacketType PacketType;
  int FromRadioId;
  char RiWend;
  char Fahrstufe;
  int Li_V;
  int Schluss_V;
  int Li_H;
  int Schluss_H;
  int Li_F;
  int Laeuten;
  int Pfeifen;
  uint32_t FailedTXCount;
};

NRFLite _radio;
RadioPacketRX DataRX;
RadioPacketTX DataTX;



unsigned long currentMillis = 0;
unsigned long prevRXMillis = 0;
unsigned long FailedRXCount = 0;  // Fehlgeschlagene Empfangsvorgänge
unsigned long MaxFailedRXCount = 100;


String msg = " ";
char prevFahrstufe;

SerialTransfer myTransfer;
struct __attribute__((packed)) STRUCTTX  {
  byte FromRadioId;
  byte RiWend;
  byte Fahrstufe;
  byte Li_V;
  byte Schluss_V;
  byte Li_H;
  byte Schluss_H;
  byte Li_F;
  byte Laeuten;
  byte Pfeifen;
} SerTX;

struct __attribute__((packed)) STRUCTRX {
  byte FromRadioId;
  int kmh;
  int Volt;
  int Ampere;
} SerRX;
String serIn;

 //-----------------------------------------------------------------------------------------------------------        
void setup() {
  //Pins in PCF-Modulen setzen
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Start");
  
  Serial.println("Init Radio");
  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS)) {
    
    Serial.println("Cannot communicate with radio");
    //while (1)
      ;  // Wait here forever.
  }

  myTransfer.begin(Serial);
   
}

//-------------------------------------------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:
  
delay(50);
  //Serial.println("Loop");
  _receive();
  if (FailedRXCount > MaxFailedRXCount) {
    SerTX.FromRadioId = 99;
  } else {
    SerTX.FromRadioId = DataRX.FromRadioId;
  }
  _repackit();
  _transmit();
  /*String msg = "Nano FromRadioId = ";
    msg += SerTX.FromRadioId;
    msg += ", Volt: ";
    msg += SerRX.Volt;
    msg += ", Tacho: ";
    msg += SerRX.kmh;
    msg += ", Strom: ";
    msg += SerRX.Ampere;
    msg += ", tVolt: ";
    msg += DataTX.Volt;
    msg += ", RiWend: ";
    msg += DataRX.RiWend;
    msg += ", Fahrstufe: ";
    msg += DataRX.Fahrstufe;
    msg += ", prevFahrstufe: ";
    msg += prevFahrstufe;
    msg += ", SerTX: ";
    msg += SerTX.Fahrstufe;

    Serial.println(msg);
    Serial.println(" ");*/
    String msg = ", Tacho: ";
    msg += SerRX.kmh;
    msg += ", Strom: ";
    msg += SerRX.Ampere;
    msg += ", tVolt: ";
    msg += DataTX.Volt;
    Serial.println(" ");
    Serial.println(msg);
    Serial.println(" ");
  
}

//-----------------------------------------------------------------------------------------------------

void _receive() {
  
  FailedRXCount++;
  while (_radio.hasData()) {
    //Serial.println("hasData");
    _radio.readData(&DataRX);
    prevRXMillis = millis();
    if (DataRX.PacketType == BeginGetData) {
      //Serial.println("Received data request, adding ACK data packet");
      FailedRXCount = 0;      
      _radio.addAckData(&DataTX, sizeof(DataTX));
    } else if (DataRX.PacketType == EndGetData) {
      // dummy?
    }
 
  
    //Serial.println(" - - - - - - ");
   /* String msg = "Nano FromRadioId = ";
    msg += DataRX.FromRadioId;
    msg += ", RiWend: ";
    msg += DataRX.RiWend;
    msg += ", Fahrstufe: ";
    msg += DataRX.Fahrstufe;
    msg += ", Li_V = ";
    msg += DataRX.Li_V;
    msg += ", Schluss_V = ";
    msg += DataRX.Schluss_V;
    msg += ", Li_H = ";
    msg += DataRX.Li_H;
    msg += ", Schluss_H = ";
    msg += DataRX.Schluss_H;
    Serial.println (msg);
    msg = "Li_PWM: "; 
    msg += DataRX.Li_F;
    msg += ", Pfeifen = ";
    msg += DataRX.Pfeifen;
    msg += ", Laeuten = ";
    msg += DataRX.Laeuten;
    msg += ", Volt: ";
    msg += SerRX.Volt;
    msg += ", Tacho: ";
    msg += SerRX.kmh;
    msg += ", Strom: ";
    msg += SerRX.Ampere;

    Serial.println(msg);
    Serial.println(" ");*/
   

  }
}

void _repackit() {  int tVolt;
  //SerTX.FromRadioId = DataRX.FromRadioId;
  if (DataRX.Fahrstufe == 'F') {
    if (prevFahrstufe == 'O') {
      SerTX.Fahrstufe = 'O';
      prevFahrstufe = 'O';
    } else if (prevFahrstufe == 'P') {
      SerTX.Fahrstufe = 'P';
      prevFahrstufe = 'P';
    } else  {
      SerTX.Fahrstufe = DataRX.Fahrstufe;
      prevFahrstufe = SerTX.Fahrstufe;
    } 
  }  else  {
      SerTX.Fahrstufe = DataRX.Fahrstufe;
      prevFahrstufe = SerTX.Fahrstufe;
  }
  

  


  SerTX.RiWend = DataRX.RiWend;
  
  SerTX.Li_V = DataRX.Li_V;
  SerTX.Schluss_V = DataRX.Schluss_V;
  SerTX.Li_H = DataRX.Li_H;
  SerTX.Schluss_H = DataRX.Schluss_H;
  SerTX.Li_F = DataRX.Li_F;
  SerTX.Pfeifen = DataRX.Pfeifen;
  SerTX.Laeuten = DataRX.Laeuten;
  DataTX.FromRadioId = SerRX.FromRadioId;
  tVolt = SerRX.Ampere / 3.43;
  DataTX.Volt = tVolt;
  //DataTX.Volt = SerRX.Volt;
  DataTX.kmh = SerRX.kmh;
  //DataTX.Ampere = SerRX.Ampere;
  DataTX.Ampere = 0;
}

void _transmit() {
  digitalWrite(LED_BUILTIN, HIGH); 
  uint16_t sendSize = 0;
  sendSize = myTransfer.txObj(SerTX, sendSize);
  myTransfer.sendData(sendSize);
  digitalWrite(LED_BUILTIN, LOW);
  //Serial.println(sendSize);
  
  if(myTransfer.available()) {
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(SerRX, recSize);
    
  }
  
}
