


/* Lok21_intern
 * Version 0.1
 * Interimsversion mit Nano als Protokollwandler
 * Tachometer: 24 Pulse/U bei CHANGE. 24V -> 820 U/min = 13,8 km/h
 * DataTX.kmh = 175 ist der Maximalwert für Fahrpult der Lok 3
 * Neues Pulsmuster für Fahren
 * Unbedingt esp32 in Version 2.0.17 im Boardverwalter installieren!!
 */
#include <SPI.h>
//#include <NRFLite.h>           // https://github.com/dparson55/NRFLite
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>  // https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/Multiple/Multiple.ino
#include <SoftwareSerial.h>     // https://github.com/plerup/espsoftwareserial/
#include <Packet.h>
#include <PacketCRC.h>
#include <SerialTransfer.h>  // https://github.com/PowerBroker2/SerialTransfer
#include <pu2clr_pcf8574.h>  // https://github.com/pu2clr/PCF8574/blob/main/examples/poc_00/poc_00.ino

#include <Servo.h>  // https://github.com/alunit3/ServoESP32/
#include <ESP32PWM.h>
#include <NonBlockingTimer.h>  // https://github.com/richardsibanda/nonblockingtimer/blob/main/examples/example1/example1.ino
#include <MovingAverage.h>





//Definitionen der Pinnummern
// ESP32
#define R_PWM 32  //IBT-2-Baustein
#define L_PWM 33
#define R_EN 25
#define L_EN 26
#define SENSE_STROM 35  // Analogeingang Stromsensor
#define REV_NA 14       // Hallsensor Laufdrehgestell
#define REV_MOT 12      // Hallsensor Motordrehgestell
#define SENSE_24V 13    // Spannungsmessung Batterie

#define SCL 22  // I²C
#define SDA 21
#define PantoPinV 17
#define PantoPinH 16
#define TEMP_PIN 27    // Dallas-Temperatursensor
#define SOUND_PLAY 34  // Soundmodul aktiv

// Definitionen für die PCF8574-Module
PCF PCF_I;
PCF PCF_II;
PCF PCF_SOUND;
PCF PCF_BRAKE;

//Definitionen PCF8574-Pins
#define DLICHT 0  // Führerstandsbeleuchtung
#define R_B 1     // Rot - Beimann
#define R_F 2     // Rot - Führer
#define R_D 3     // Rot - Dach
#define W_B 4     // Weiss - Beimann
#define W_F 5     // Weiss - Führer
#define W_D 6     // Weiss - Dach

#define SOUND1 7
#define SOUND2 6
#define SOUND3 5
#define SOUND4 4
#define SOUND5 3
#define SOUND6 2
#define SOUND7 1
#define SOUND8 0
#define SOUND9 7  // auf PCF_BRAKE!

#define BR_EN 1    // Bremse aktivieren
#define BR_UP 2    // Bremse anlegen
#define BR_DOWN 3  // Bremse lösen
#define FAN_MOT 4
#define FAN_ELEK 5

#define MYPORT_TX 5
#define MYPORT_RX 19

Servo Panto_V;
Servo Panto_H;
int Panto_auf = 180;
int Panto_nieder = 0;
bool Panto_active = false;

// Temperatursensoren
#define TEMPERATURE_PRECISION 9
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress MotorThermometer;

// Assign address manually. The addresses below will need to be changed
// to valid device addresses on your bus. Device address can be retrieved
// by using either oneWire.search(deviceAddress) or individually via
// sensors.getAddress(deviceAddress, index)
DeviceAddress ElektronikThermometer = { 0x28, 0x61, 0x64, 0x0A, 0x7B, 0x0B, 0x2C, 0xA3 };
// DeviceAddress outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

/*
// Definitionen für Funkverbindung mit NRF24
const static int RADIO_ID = 10;
const static uint8_t PIN_RADIO_CE = 4;         // Mega: 18, Nano: 6  ESP: 4
const static uint8_t PIN_RADIO_CSN = 15;       // Mega: 53, Nano: 7 ESP: 15
const static uint8_t PIN_RADIO_MOSI = 23;
const static uint8_t PIN_RADIO_MISO = 19;
const static uint8_t PIN_RADIO_SCK = 18;
const static uint8_t PIN_RADIO_SS = 5;

const static int DESTINATION_RADIO_ID = 1;  //

//const static uint32_t MaxFailedTXCount = 30; // Maximalzahl der fehlgeschlagenen Übertragungen
*/
enum RadioPacketType {
  AcknowledgementData,  // Produced by the primary receiver and provided to the transmitter via an acknowledgement data packet.
  Heartbeat,            // Sent by the primary transmitter. (not used)
  BeginGetData,         // Sent by the primary transmitter to tell the receiver it should load itself with an acknowledgement data packet.
  EndGetData            // Sent by the primary transmitter to receive the acknowledgement data packet from the receiver.
};

struct __attribute__((packed)) RadioPacketTX  // Paket von Lok
//struct RadioPacketTX  // Paket von Lok
{
  RadioPacketType PacketType;
  int FromRadioID;
  int kmh;
  int Volt;
  int Ampere;
};
struct __attribute__((packed)) RadioPacketRX  // Paket vom Fahrpult
//struct RadioPacketRX  // Paket vom Fahrpult
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

//NRFLite _radio(Serial);
RadioPacketRX DataRX;
RadioPacketTX DataTX;

int Li_V_prev;
int Schluss_V_prev;
int Li_H_prev;
int Schluss_H_prev;
int Li_F_prev;
int Pfeifen_prev;
int Laeuten_prev;
bool light_sound_changed = false;
unsigned long previousMillis = millis();  // Intervalltimer für Stoermelder
unsigned long previousRiWendMillis = millis();   //Intervalltimer für Richtungswender, Laufzeit Stromabnehmer
unsigned long currentMillis = 0;
unsigned long pMillis = 0;           // Intervalltimer für Speedmessung
unsigned long cMillis = 0;           // Intervalltimer für Aufschalten Feinstufen
unsigned long prevRXMillis = 0;      // Intervalltimer für Empfänger
unsigned long prevPWMMotMillis = 0;  // Intervalltimer für Schaltwerk
unsigned long prevPWMMotDownMillis = 0;
unsigned long FailedRXCount = 0;  // Fehlgeschlagene Empfangsvorgänge
unsigned long MaxFailedRXCount = 30;
unsigned long previousPMillis = millis();  // Intervalltimer für Pfeife
bool RiWend = false;                       // Logik für den Richtungswender, damit Panto etc. funktioniert
//bool pActive = false;                      // true, wenn Pfeife auf PWM läuft
bool ShutDown = false;       // Löst Nullstellung bei Empfangsstörung aus
bool AfterShutDown = false;  // Verwendet, um Nullstellung nach Wiederaufnahme zu erzwingen
bool AfterNeutral = false;   //
//bool Active = false;         // true, wenn im aktiven Fahrbetrieb, also nicht "N"
bool brake = false;   // Fahren: true, Bremsen: false
bool PFlag = false;   // Unterscheidung langer - kurzer Pfiff
bool SoundFlagRW;     // Damit der Sound nur ein mal abgespielt und I2C nicht unnötig belastet wird!
bool SoundFlagUD;     // Damit der Sound nur ein mal abgespielt wird
int PWMMot_soll = 0;  // PWM zu den IBT-Bausteinen
int PWMMot_ist = 0;

int PWMFahren[12];
int PWMFahren_schnell[12] = { 0, 8, 20, 45, 70, 90, 110, 130, 155, 180, 205, 240};  // PWM-Werte für Fahrstufen Schnellgang  Vmax 105
int PWMFahren_langsam[12] = { 0, 5, 15, 25, 35, 45, 65, 80, 95, 110, 125, 140};     // PWM-Werte für Fahrstufen Langsamgang  Vmax 52
int PWMBremsen[6] = { 0, 5, 40, 100, 150, 254 };                                   // PWM-Werte für Bremsstufen
int PWMSpike_orig = 8;                                                               // PWM-Wert für Anfangspuls, addiert sich zu akt. PWM-Wert
int PWMSpike = 0;
unsigned long SpikeTime = 60;  // Dauer PWM-Puls am Anfang des Schaltvorgangs
int Fahrstufe_soll = 0;
int Fahrstufe_ist = 0;
int Fahrstufe_ist_prev = 0;

bool microPWM = false;
bool Rangiergang = true;
bool FstError = false;
unsigned int speedCounterMot = 0;
unsigned int speedCounterNA = 0;
unsigned long mTime = 410;  // Messzeit Speedmessung
unsigned int speedMot = 0;
unsigned int speedNA = 0;
int speedDiff = 0;
unsigned int speedFiltered = 0;

unsigned long RXOutTime = 2000;  // Karrenzzeit für Ausfall Empfang
unsigned long PWMUPTime = 400;   // Verzögerung für Schaltwerk
unsigned long PWMDOWNTime = 10;
unsigned long BrakeTime = 1000;  // Zeit, die der Bremsaktuator für 1/5 braucht
unsigned long BrakeStart;        // Startwert für BrakeTime
int BrakeStep = 0;               // Stufenzähler für Bremsbetätigung

int TempMot = 20;     // Messwert Temperatur
int TempMotMax = 35;  // Schaltschwelle für Lüfter

// PWM auf ESP32
const int PWMFreq = 400;  // PWM-Frequenz
ESP32PWM RPWM;
ESP32PWM LPWM;
ESP32PWM REN;
ESP32PWM LEN;

// Moving Average
MovingAverage total_speed(6);
MovingAverage total_BatVolt(10);
MovingAverage total_Current(10);

bool DownFirstTime = false;  // rettet PWM-Wert beim Abschalten

NonBlockingTimer s7Timer(3350);  // Create a timer with an interval
NonBlockingTimer s6Timer(620);
NonBlockingTimer s9Timer(520);


unsigned long startTime;
unsigned long interval;
unsigned long brakeFlag;

SerialTransfer myTransfer;
struct __attribute__((packed)) StructSerRX {
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
};

struct __attribute__((packed)) StructSerTX {

  byte FromRadioID;
  int kmh;
  int Volt;
  int Ampere;
};

StructSerRX SerRX;
StructSerTX SerTX;



//-----------------------------------------------------------------------------------------------------------
void setup() {
  //Pins in PCF-Modulen setzen
  delay(1000);
  Serial.begin(115200);
  Serial.println("Start");
  Wire.begin(21, 22);
  PCF_I.setup(0x20);
  PCF_II.setup(0x21);
  PCF_SOUND.setup(0x22);
  PCF_BRAKE.setup(0x23);
  PCF_I.write(0x00);
  PCF_II.write(0x00);
  PCF_SOUND.write(0xFF);
  PCF_BRAKE.write(0B10000000);

  pinMode(REV_NA, INPUT_PULLUP);
  pinMode(REV_MOT, INPUT_PULLUP);
  pinMode(SENSE_STROM, INPUT);  // 0A = 800
  pinMode(SENSE_24V, INPUT);    // 20V = 2350
  pinMode(SOUND_PLAY, INPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  // Verbindung zu Nano mit NRF24 starten
  myTransfer.begin(Serial);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Panto_V.attach(PantoPinV);
  Panto_H.attach(PantoPinH);

  // PWM-Settings
  RPWM.attachPin(R_PWM, PWMFreq, 8);
  LPWM.attachPin(L_PWM, PWMFreq, 8);
  LEN.attachPin(L_EN, PWMFreq, 8);
  REN.attachPin(R_EN, PWMFreq, 8);

  attachInterrupt(REV_MOT, _speedCountMot, CHANGE);
  attachInterrupt(REV_NA, _speedCountNA, CHANGE);
  /* NRF24
  Serial.println("Init Radio");
   // Configure SPI pins.
    SPI.begin(PIN_RADIO_SCK, PIN_RADIO_MISO, PIN_RADIO_MOSI, PIN_RADIO_SS);
    //SPI.setClockDivider(64);  
    // Indicate to NRFLite that it should not call SPI.begin() during initialization since it has already been done.
    uint8_t callSpiBegin = 0;
  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS), 100, callSpiBegin) {
    //morse.send("SOS");   // 5x lang, 5x kurz
    Serial.println("Cannot communicate with radio");
    //while (1)
      ;  // Wait here forever.
  }
  _radio.printDetails();
  if (_radio.hasData()) {
    // Einmal lesen, um Schalterstellungen des Bedienteils zu bekommen
    _radio.readData(&DataRX);
    Serial.println("radio OK");
    String msg = "Radio ";b 
        msg += _radioData.FromRadioId;
        msg += ", ";
        msg += _radioData.OnTimeMillis;
        msg += " ms, ";
        //msg += _radioData.FailedTxCount;
       // msg += " Failed TX";

        Serial.println(msg);
   } */
  if (DataRX.Li_F == 1) {  // "Sanfte" Schaltstufen, wenn Fernlicht beim Einschalten auf AN, bleibt für Session bestehen
  //PWMSpike_orig = 1;
  }

  // Temperatursensoren
  s6Timer.start();
  //s7Timer.start();
  //s9Timer.start();
  sensors.begin();
  // Einmal kurz pfeifen als Bestätigung für setup
  PCF_SOUND.digitalWrite(SOUND1, LOW);       
  delay(30);
   PCF_SOUND.write(0xFF);

  

}

//-------------------------------------------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:

  _receive();
  _speed();
  /*
  String msg = "RiWend: ";
    msg += RiWend;
    msg += ", ShutDown: ";
    msg += ShutDown;
    msg += ", AfterShutDown: ";
    msg += AfterShutDown;
    msg += ", Fahrstufe: ";
    msg += Fahrstufe_soll;
    msg += ", PWMMot_soll: ";
    msg += PWMMot_soll;
    msg += ", PWMMot_ist: ";
    msg += PWMMot_ist;
    msg += ", DataRX.FromRadioId: ";
    msg += DataRX.FromRadioId;

    Serial.println(" ");
    Serial.println(msg);
   */
  /*
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();
  Serial.println("DONE");

  // print the device information
  printData(insideThermometer);
  printData(outsideThermometer);  
  */
  
}
//-----------------------------------------------------------------------------------------------------

void _speed() {
  if (millis() - pMillis > mTime) {
    detachInterrupt(REV_MOT);
    detachInterrupt(REV_NA);
    //stop the timer
    speedMot = speedCounterMot;
    speedNA = speedCounterNA;
    speedCounterMot = 0;  //  reset counter to zero
    speedCounterNA = 0;
    pMillis = millis();

    attachInterrupt(REV_MOT, _speedCountMot, CHANGE);
    attachInterrupt(REV_NA, _speedCountMot, CHANGE);
  }
}


void _speedCountMot() {
  speedCounterMot++;
}

void _speedCountNA() {
  speedCounterNA++;
}

//--------------------------------------------------------------------------------------------------
void _receive() {
  //FailedRXCount++;
  _transmit();
  _repackit();
  if (DataRX.FromRadioId == 99) {  // Nano mit NRF24 hat Verbindung zum Steuergerät verloren
    _ShutDown();
    AfterShutDown = true;  // Nullstellungszwang aktivieren
  }
  // Aktionen, die nur bei Empfang von Daten laufen müssen
  _direction();
  _drive();
  _light_sound_check();
  /*
    Serial.println("     DataRX");
    String msg = "FromRadioId = ";
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
    msg += DataTX.Volt;
    msg += ", Tacho: ";
    msg += DataTX.kmh;
    msg += ", Strom: ";
    msg += DataTX.Ampere;

    String msg = "speedCounterMot: ";
    msg += speedCounterMot;
    msg += ", Tacho: ";
    msg += DataTX.kmh;
    msg += ", Filtered: ";
    msg += speedFiltered;
    Serial.println(msg);
    Serial.println(" ");
 */
}

void _repackit() {

  DataRX.FromRadioId = SerRX.FromRadioId;
  DataRX.RiWend = SerRX.RiWend;
  DataRX.Fahrstufe = SerRX.Fahrstufe;
  DataRX.Li_V = SerRX.Li_V;
  DataRX.Schluss_V = SerRX.Schluss_V;
  DataRX.Li_H = SerRX.Li_H;
  DataRX.Schluss_H = SerRX.Schluss_H;
  DataRX.Li_F = SerRX.Li_F;
  DataRX.Pfeifen = SerRX.Pfeifen;
  DataRX.Laeuten = SerRX.Laeuten;

  DataTX.FromRadioID = 0;
  DataTX.kmh = total_speed.addSample(speedMot);
  // Tachoüberlauf verhindern
  if (DataTX.kmh > 175) {
    DataTX.kmh = 175;
  }
  DataTX.Volt = total_BatVolt.addSample(analogRead(SENSE_24V));
  DataTX.Ampere = total_Current.addSample(analogRead(SENSE_STROM)) - 803;
  SerTX.FromRadioID = DataTX.FromRadioID;
  SerTX.Volt = DataTX.Volt;
  SerTX.kmh = DataTX.kmh;
  SerTX.Ampere = DataTX.Ampere;
  speedFiltered = total_speed.addSample(speedMot);
  /*String msg = "Fahrstufe = ";
    msg += DataRX.Fahrstufe;    
    msg += ", RiWend: ";
    msg += SerRX.RiWend;
    msg += ", Fahrstufe: ";
    msg += SerRX.Fahrstufe;
    msg += ", Li_V = ";
    msg += SerRX.Li_V;
    msg += ", Schluss_V = ";
    msg += SerRX.Schluss_V;
    msg += ", Li_H = ";
    msg += SerRX.Li_H;
    msg += ", Schluss_H = ";
    msg += SerRX.Schluss_H;
    Serial.println (msg);
    msg = "Li_PWM: ";
    msg += SerRX.Li_F;
    msg += ", Pfeifen = ";
    msg += SerRX.Pfeifen;
    msg += ", Laeuten = ";
    msg += SerRX.Laeuten;
    msg += "; RiWend: ";
    msg += RiWend; 
   
  
    
    
    
    /*msg += ", speedCounterMot: ";
    msg += speedCounterMot;
    msg += ", Volt: ";
    msg += SerTX.Volt;
    msg += ", SENSE_24V: ";
    msg += analogRead(SENSE_24V);
    msg += ", Strom: ";
    msg += SerTX.Ampere;
    msg += ", SENSE_STROM: ";
    msg += analogRead(SENSE_STROM);
    
    msg += ", Fahrstufe_soll: ";
    msg += Fahrstufe_soll;
    msg += ", Fahrstufe_ist: ";
    msg += Fahrstufe_ist;
    msg += ", PWMMot_soll: ";
    msg += PWMMot_soll,
    msg += ", PWMMot_ist: ";
    msg += PWMMot_ist;
    msg += ", Tacho: ";
    msg += SerTX.kmh;
   
   
    Serial.println(msg);
    Serial.println(" ");*/
}
//-----------------------------------------------------------------------------------
void _transmit() {
  String arr;
  String myStr;
  if (myTransfer.available()) {
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(SerRX, recSize);
    /*Serial.print("received ");
    Serial.print(recSize);
    Serial.println(" bytes");*/
  }
  uint16_t sendSize = 0;
  sendSize = myTransfer.txObj(SerTX, sendSize);
  myTransfer.sendData(sendSize);
  /*Serial.print("sent ");
  Serial.print(sendSize);
  Serial.println(" bytes");*/
}
//-----------------------------------------------------------------------------------
void _light_sound_check() {

  light_sound_changed = false;
  if (Li_V_prev != DataRX.Li_V) {
    light_sound_changed = true;
  }
  if (Li_H_prev != DataRX.Li_H) {
    light_sound_changed = true;
  }
  if (Schluss_V_prev != DataRX.Schluss_V) {
    light_sound_changed = true;
  }
  if (Schluss_H_prev != DataRX.Schluss_H) {
    light_sound_changed = true;
  }
  if (Li_F_prev != DataRX.Li_F) {
    light_sound_changed = true;
  }
  if (Pfeifen_prev != DataRX.Pfeifen) {
    light_sound_changed = true;
  }
  if (Laeuten_prev != DataRX.Laeuten) {
    light_sound_changed = true;
  }
  Li_H_prev = DataRX.Li_H;
  Li_V_prev = DataRX.Li_V;
  Schluss_H_prev = DataRX.Schluss_H;
  Schluss_V_prev = DataRX.Schluss_V;
  Li_F_prev = DataRX.Li_F;
  Pfeifen_prev = DataRX.Pfeifen;
  Laeuten_prev = DataRX.Laeuten;

  if (light_sound_changed == true || PFlag == true) {
    _light_sound();
  }
}

void _light_sound() {

  // Steuerung Front- und Schlusslicht, Pfeife
  if ((DataRX.Li_V > 0) && (DataRX.Li_H > 0)) {  // Rangierlicht
    PCF_I.digitalWrite(W_B, HIGH);
    PCF_II.digitalWrite(W_B, HIGH);
    PCF_I.digitalWrite(W_F, LOW);
    PCF_I.digitalWrite(W_D, LOW);
    PCF_II.digitalWrite(W_F, LOW);
    PCF_II.digitalWrite(W_D, LOW);
    goto weiter;
  } else {
    PCF_I.digitalWrite(W_B, LOW);
    PCF_II.digitalWrite(W_B, LOW);
  }
  if ((DataRX.Schluss_V > 0) && (DataRX.Schluss_H > 0)) {  //
    PCF_I.digitalWrite(R_D, HIGH);
    PCF_II.digitalWrite(R_D, HIGH);
    PCF_I.digitalWrite(R_B, LOW);
    PCF_I.digitalWrite(R_F, LOW);
    PCF_II.digitalWrite(R_B, LOW);
    PCF_II.digitalWrite(R_F, LOW);
    goto weiter;
  } else {
    PCF_I.digitalWrite(R_D, LOW);
    PCF_II.digitalWrite(R_D, LOW);
  }


  if (DataRX.Li_V > 0) {
    PCF_I.digitalWrite(W_B, HIGH);
    PCF_I.digitalWrite(W_F, HIGH);
    PCF_I.digitalWrite(W_D, HIGH);
    PCF_II.digitalWrite(W_B, HIGH);
	goto weiter;
  } else {
    PCF_I.digitalWrite(W_B, LOW);
    PCF_I.digitalWrite(W_F, LOW);
    PCF_I.digitalWrite(W_D, LOW);
    PCF_II.digitalWrite(W_B, LOW);
  }
  if (DataRX.Li_H > 0) {
    PCF_II.digitalWrite(W_B, HIGH);
    PCF_II.digitalWrite(W_F, HIGH);
    PCF_II.digitalWrite(W_D, HIGH);
    PCF_I.digitalWrite(W_B, HIGH);
	goto weiter;
  } else {
    PCF_II.digitalWrite(W_B, LOW);
    PCF_II.digitalWrite(W_F, LOW);
    PCF_II.digitalWrite(W_D, LOW);
    PCF_I.digitalWrite(W_B, LOW);
  }
  
weiter:
  if (DataRX.Schluss_V > 0) {
    PCF_I.digitalWrite(R_B, HIGH);
    PCF_I.digitalWrite(R_F, HIGH);
  } else {
    PCF_I.digitalWrite(R_B, LOW);
    PCF_I.digitalWrite(R_F, LOW);
  }
  if (DataRX.Schluss_H > 0) {
    PCF_II.digitalWrite(R_B, HIGH);
    PCF_II.digitalWrite(R_F, HIGH);
  } else {
    PCF_II.digitalWrite(R_B, LOW);
    PCF_II.digitalWrite(R_F, LOW);
  }

  // Führerstandsbeleuchtung ein
  if (DataRX.Li_F > 0) {
    PCF_I.digitalWrite(DLICHT, HIGH);
    PCF_II.digitalWrite(DLICHT, HIGH);
  } else {
    PCF_I.digitalWrite(DLICHT, LOW);
    PCF_II.digitalWrite(DLICHT, LOW);
  }

  if (PFlag == false) {
    if (DataRX.Pfeifen > 0) {
      PFlag = true;
      previousPMillis = millis();
    }
  } else {
    if ((millis() - previousPMillis) > 400) {
      if (DataRX.Pfeifen > 0) {
        // langer Pfiff
        if (digitalRead(SOUND_PLAY) == LOW) {
          PCF_SOUND.digitalWrite(SOUND2, LOW);
          delay(30);
          PCF_SOUND.write(0xFF);
        }
      } else {
        if (DataRX.Pfeifen < 1) {
          // kurzer Pfiff
          if (digitalRead(SOUND_PLAY) == LOW) {
            PCF_SOUND.digitalWrite(SOUND1, LOW);
            delay(30);
            PCF_SOUND.write(0xFF);
            PFlag = false;
          }
        }
      }
      if ((millis() - previousPMillis) > 1500) {
        // Totzeit für Pfeife
        PFlag = false;
      }
    }
  }
}

//------------------------------------------------------------------------------------------------//

void _direction() {

  switch (DataRX.RiWend) {
    case 'N':

      if (RiWend == true) {  // Abschalten aus der fahrtstellung heraus
        PCF_SOUND.digitalWrite(SOUND4, LOW);
        delay(30);
        PCF_SOUND.write(0xFF);

        previousRiWendMillis = millis();
        Panto_active = true;
        Panto_V.attach(PantoPinV);
        Panto_H.attach(PantoPinH);
        Panto_V.write(Panto_nieder);
        Panto_H.write(Panto_nieder);
        RiWend = false;
        SoundFlagRW = false;
        Bremse_Neutral();
      }
      if (Panto_active == true) {
        if ((millis() - previousRiWendMillis) > 1000) {
          Panto_V.detach();
          Panto_H.detach();
          Elektronikluefter_aus();
          Motorluefter_aus();
          Panto_active = false;
        }
      }

      AfterShutDown = false;  // Nullstellungszwang nach Wiederaufnahme auflösen
      ShutDown = false;
      PCF_I.digitalWrite(R_D, LOW);
      PCF_II.digitalWrite(R_D, LOW);
      break;
    case 'V':
      if (AfterShutDown == false) {  // Nullstellungszwang nach Wiederaufnahme
        if (RiWend == false) {
          PWMMot_ist = 0;
          previousRiWendMillis = millis();
          RiWend = true;  // Richtungswender in Fahrtstellung
          Panto_active = true;
          Panto_H.attach(PantoPinH);
          Panto_H.write(Panto_auf);

          if ((DataRX.Li_V > 0) && (DataRX.Li_H > 0)) {  // Rangierlicht, steht für Langsamgang
            memmove(PWMFahren, PWMFahren_langsam, sizeof(PWMFahren_langsam));
            PWMSpike = PWMSpike_orig/2;
            Rangiergang = true;
          } else {
            memmove(PWMFahren, PWMFahren_schnell, sizeof(PWMFahren_langsam));
            PWMSpike = PWMSpike_orig;
            Rangiergang = false;
          }
        } else {			// Routine für Panto und Sound		
          if (Panto_active == true) {
            if ((millis() - previousRiWendMillis) > 1000) {
              if (SoundFlagRW == false) {
                Panto_H.detach();
                PCF_SOUND.digitalWrite(SOUND5, LOW);  // Hauptschalter ein Sound
                delay(30);
                PCF_SOUND.write(0xFF);
                SoundFlagRW = true;
                Elektronikluefter_ein();
                Motorluefter_ein();
                Panto_active = false;
              }
            }
          }
        }
      }
      break;
    case 'R':
      if (AfterShutDown == false) {  // Nullstellungszwang nach Wiederaufnahme

        if (RiWend == false) {
          PWMMot_ist = 0;
          previousRiWendMillis = millis();
          RiWend = true;
          Panto_active = true;
          Panto_V.attach(PantoPinV);
          Panto_V.write(Panto_auf);
          Bremse_Neutral();
          if ((DataRX.Li_V > 0) && (DataRX.Li_H > 0)) {  // Rangierlicht, steht für Langsamgang
            memmove(PWMFahren, PWMFahren_langsam, sizeof(PWMFahren_langsam));
            PWMSpike = PWMSpike_orig/2;
            Rangiergang = true;
          } else {
            memmove(PWMFahren, PWMFahren_schnell, sizeof(PWMFahren_langsam));
            PWMSpike = PWMSpike_orig;
            Rangiergang = false;
          }
        } else {
          if (Panto_active == true) {
            if ((millis() - previousRiWendMillis) > 1200) {
              if (SoundFlagRW == false) {
                Panto_V.detach();
                PCF_SOUND.digitalWrite(SOUND5, LOW);
                delay(30);
                PCF_SOUND.write(0xFF);
                SoundFlagRW = true;
                Elektronikluefter_ein();
                Motorluefter_ein();
                Panto_active = false;
              }
            }
          }
        }
        break;
        default:
          //ShutDown = true;

          break;
      }
  }
}

void _ShutDown() {
  PWMMot_soll = PWMMot_ist = 0;
  //Serial.println ("_ShutDown angesteuert!!");
  if (RiWend == true) {
    RiWend = false;
    previousRiWendMillis = millis();
    PCF_SOUND.digitalWrite(SOUND4, LOW);
    delay(30);
    PCF_SOUND.write(0xFF);
    delay(900);
    PCF_SOUND.digitalWrite(SOUND8, LOW);
    delay(30);
    PCF_SOUND.write(0xFF);
  }
  if (Panto_active == true) {
    if ((millis() - previousRiWendMillis) > 1500) {
      Panto_V.detach();
      Panto_H.detach();
      Elektronikluefter_aus();
      Panto_active = false;
    }
  } else {
    Panto_active = true;
    Panto_V.attach(PantoPinV);
    Panto_H.attach(PantoPinH);
    Panto_V.write(Panto_nieder);
    Panto_H.write(Panto_nieder);
  }
  AfterShutDown = true;
  PCF_I.digitalWrite(R_D, HIGH);
  PCF_II.digitalWrite(R_D, HIGH);
  _PWM_OUT();
}

void _drive() {
  if (RiWend == true) {
    // PWM-Wert aus Fahrstufe erzeugen
    if (FstError == true) {  // Workaround, weil nach Fst O die Fst F kommt, dann P
      if (DataRX.Fahrstufe == 'F') {
        DataRX.Fahrstufe = 'O';
      }
    }
    switch (DataRX.Fahrstufe) {
      case 'A':
        PWMMot_soll = PWMBremsen[5];
        brake = true;
        Fahrstufe_ist = -5;
        Bremse_Zu();
        break;
      case 'B':
        PWMMot_soll = PWMBremsen[4];
        brake = true;
        Fahrstufe_ist = -4;
        Bremse_Neutral();
        break;
      case 'C':
        PWMMot_soll = PWMBremsen[3];
        brake = true;
        Fahrstufe_ist = -3;
        Bremse_Auf();
        break;
      case 'D':
        PWMMot_soll = PWMBremsen[2];
        brake = true;
        Fahrstufe_ist = -2;
        Bremse_Auf();
        break;
      case 'E':
        PWMMot_soll = PWMBremsen[1];
        brake = true;
        Fahrstufe_ist = -1;
        Bremse_Auf();
        break;

      case 'F':
        PWMMot_soll = PWMFahren[0];
        if (brake == true) {  // Verhindert, dass beim Umschalten auf Fahren PWMMot_ist Werte > 0 hat
          brake = false;
          // Fahrstufe_ist_prev = 0;
          PWMMot_soll = 0;
          PWMMot_ist = 0;
          //speedCounter = 0;
          //DataTX.kmh = 0;
        }
        Fahrstufe_ist = 0;
        Fahrstufe_soll = 0;
        AfterNeutral = true;
        Bremse_Neutral();
        FstError = false;
        break;

      case 'G':
        PWMMot_soll = PWMFahren[1];
        brake = false;
        Fahrstufe_soll = 1;
        FstError = false;
        break;
      case 'H':
        PWMMot_soll = PWMFahren[2];
        brake = false;
        Fahrstufe_soll = 2;
        FstError = false;
        break;
      case 'I':
        PWMMot_soll = PWMFahren[3];
        brake = false;
        Fahrstufe_soll = 3;
        FstError = false;
        break;
      case 'J':
        PWMMot_soll = PWMFahren[4];
        brake = false;
        Fahrstufe_soll = 4;
        FstError = false;
        break;
      case 'K':
        PWMMot_soll = PWMFahren[5];
        brake = false;
        Fahrstufe_soll = 5;
        FstError = false;
        break;
      case 'L':
        PWMMot_soll = PWMFahren[6];
        brake = false;
        Fahrstufe_soll = 6;
        FstError = false;
        break;
      case 'M':
        PWMMot_soll = PWMFahren[7];
        brake = false;
        Fahrstufe_soll = 7;
        FstError = false;
        break;
      case 'N':
        PWMMot_soll = PWMFahren[8];
        brake = false;
        Fahrstufe_soll = 8;
        FstError = false;
        break;
      case 'O':
        PWMMot_soll = PWMFahren[9];
        brake = false;
        Fahrstufe_soll = 9;
        FstError = true;
        break;
      case 'P':
        PWMMot_soll = PWMFahren[10];
        brake = false;
        Fahrstufe_soll = 10;
        FstError = true;
        break;
      case 'Q':
        PWMMot_soll = PWMFahren[11];
        brake = false;
        Fahrstufe_soll = 11;
        FstError = true;

        break;
      default:
        //morse.send("E");
        /*
        PWMMot_soll = 0;
        brake = false;
        Fahrstufe_soll = 0;
        Fahrstufe_ist = 0;
        ShutDown = true;
        */
        break;
    }

    if ((brake == false)) {
      // Schaltwerk

      _UP_DOWN();
      Bremse_Auf();
    } else {
      Fahrstufe_soll = Fahrstufe_ist;
    }
    _Sound_Up_Down();
    _PWM_OUT();
  }
}

void _UP_DOWN() {
  //Serial.print(" UP_DOWN - ");
  if (PWMMot_soll > PWMMot_ist) {  // Aufschalten
    /*Serial.print("AUF : PWMMot_soll > PWMMot_ist : PWMMot_soll = ");
    Serial.print("microPWM = ");
    Serial.print(microPWM);
    Serial.print("  :  PWMMot_ist = ");
    Serial.println(PWMMot_ist);*/
    if (microPWM == true) {
      if (millis() - cMillis >= (PWMUPTime / 10)) {
        if (Rangiergang == false) {
          PWMMot_ist = PWMMot_ist + 3;
        } else {
          PWMMot_ist = PWMMot_ist + 1;
        }
        if (PWMMot_ist >= PWMFahren[Fahrstufe_ist]) {
          PWMMot_ist >= PWMFahren[Fahrstufe_ist];
          microPWM = false;
        }
        _PWM_OUT();
      }
    }
    if (millis() - prevPWMMotMillis > PWMUPTime) {  //Stufenzeit erreicht
      if (PWMMot_soll > PWMMot_ist) {
        
        ++Fahrstufe_ist;
        if (Fahrstufe_ist > Fahrstufe_soll) {
          Fahrstufe_ist = Fahrstufe_soll;
        }
        if (PWMMot_ist < PWMFahren[Fahrstufe_ist]) {  // Stufe 1 kein Spike!
          if (PWMFahren[Fahrstufe_ist] > 1) {
            PWMMot_ist = PWMFahren[Fahrstufe_ist] + PWMSpike;
            _PWM_OUT();
            delay(SpikeTime);
            PWMMot_ist = PWMFahren[Fahrstufe_ist - 1];
            _PWM_OUT();
          } else {
            PWMMot_ist = PWMFahren[Fahrstufe_ist - 1];
            _PWM_OUT();
          }
          microPWM = true;
          cMillis = millis();
        }
        DownFirstTime = false;
        prevPWMMotMillis = millis();
        //Serial.print("AfterNeutral = false : PWMMot_ist = ");

      }
    }
  } else if (PWMMot_soll < PWMMot_ist) {  // Abschalten
    if (PWMMot_soll < PWMMot_ist) {
      
      if (millis() - prevPWMMotMillis > PWMDOWNTime) {
        --Fahrstufe_ist;
        if (Fahrstufe_ist < Fahrstufe_soll) {
          Fahrstufe_ist = Fahrstufe_soll;
        }
        PWMMot_ist = PWMFahren[Fahrstufe_ist];
        prevPWMMotDownMillis = millis();
      }

      /*if (millis() - prevPWMMotMillis > PWMDOWNTime) {
        --Fahrstufe_ist;
        if (PWMMot_soll < PWMMot_ist) {
          if ((Fahrstufe_ist > Fahrstufe_soll)) {

            //Serial.print("Klack : ");
          }
        }
        if (Fahrstufe_ist < Fahrstufe_soll) {
          Fahrstufe_ist = Fahrstufe_soll;
        }
        PWMMot_soll = PWMFahren[Fahrstufe_ist];
        prevPWMMotMillis = millis();
        prevPWMMotDownMillis = millis();
      }
      if (millis() - prevPWMMotDownMillis > PWMDOWNTime) {
        --PWMMot_ist;
        if (PWMMot_ist < 0) {
          PWMMot_ist = 0;
        }
        prevPWMMotDownMillis = millis();
      }*/
    }
  } else {
    DownFirstTime = false;
    
  }
}

void _PWM_OUT() {
  //Serial.println (" PWM_OUT - ");
  /*String msg = "PWMMot_soll: ";
    msg += PWMMot_soll;
    msg += ", PWMMot_ist: ";
    msg += PWMMot_ist;
    msg += ", DataRX.FromRadioId: ";
    msg += DataRX.FromRadioId;

    Serial.println(" ");
    Serial.println(msg);  
    */
  // Freilauf
  if (Fahrstufe_soll == 0) {
    REN.write(0);
    LEN.write(0);
    RPWM.write(0);
    LPWM.write(0);
    /*
      digitalWrite(R_EN, LOW);
      digitalWrite(L_EN, LOW);
      digitalWrite(R_PWM, LOW);
      digitalWrite(L_PWM, LOW);
      */
    if (PWMMot_ist > 0) {

      PWMMot_ist = 0;
    }
    //PWMMot_soll = PWMMot_ist = PWMMot_prev = 0;
  } else if (brake == false) {
    // Richtung berücksichtigen
    if (DataRX.RiWend == 'V') {
      /*
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        digitalWrite(R_PWM, LOW);
        analogWrite(L_PWM, PWMMot_ist);
        */
      REN.write(255);
      LEN.write(PWMMot_ist);
      RPWM.write(255);
      LPWM.write(0);
    } else {
      /*
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        analogWrite(R_PWM, PWMMot_ist);
        digitalWrite(L_PWM, LOW);
        */
      REN.write(PWMMot_ist);
      LEN.write(255);
      LPWM.write(255);
      RPWM.write(0);
    }
  } else if (brake == true) {
    // Richtung berücksichtigen
    if (DataRX.RiWend == 'V') {
      /*
        digitalWrite(R_EN, HIGH);
        analogWrite(L_EN, PWMMot_soll);
        digitalWrite(R_PWM, LOW);
        digitalWrite(L_PWM, LOW);
        */
      REN.write(255);
      LEN.write(PWMMot_soll);
      RPWM.write(0);
      LPWM.write(0);
    } else {
      /*
        analogWrite(R_EN, PWMMot_soll);
        digitalWrite(L_EN, HIGH);
        digitalWrite(R_PWM, LOW);
        digitalWrite(L_PWM, LOW);
        */
      LEN.write(255);
      REN.write(PWMMot_soll);
      RPWM.write(0);
      LPWM.write(0);
    }
  }
}

void _Sound_Up_Down() {
  if (Fahrstufe_ist != Fahrstufe_ist_prev) {
    if ((digitalRead(SOUND_PLAY) == LOW) && s6Timer.hasElapsed()) {  // Soundmodul inaktiv

      if (Fahrstufe_ist > Fahrstufe_ist_prev) {
        if (Fahrstufe_ist_prev == 0) {
          PCF_SOUND.digitalWrite(SOUND7, LOW);
          delay(30);
          PCF_SOUND.digitalWrite(SOUND7, HIGH);
          s6Timer.start();
        } else if (Fahrstufe_ist < 0) {
          PCF_SOUND.digitalWrite(SOUND6, LOW);
          delay(30);
          PCF_SOUND.digitalWrite(SOUND6, HIGH);
          s6Timer.start();
        } else {
          PCF_BRAKE.digitalWrite(SOUND9, LOW);
          delay(30);
          PCF_BRAKE.digitalWrite(SOUND9, HIGH);
          s6Timer.start();
        }
        Fahrstufe_ist_prev++;
      } else {
        if (Fahrstufe_ist_prev == -4) {
          PCF_SOUND.digitalWrite(SOUND3, LOW);
          delay(30);
          PCF_SOUND.digitalWrite(SOUND3, HIGH);
          s6Timer.start();
        } else {
          PCF_SOUND.digitalWrite(SOUND6, LOW);
          delay(30);
          PCF_SOUND.digitalWrite(SOUND6, HIGH);
          s6Timer.start();
        }
        Fahrstufe_ist_prev--;
      }
    }
  }
}

void Elektronikluefter_ein() {
  PCF_BRAKE.digitalWrite(FAN_ELEK, HIGH);
}

void Elektronikluefter_aus() {
  PCF_BRAKE.digitalWrite(FAN_ELEK, LOW);
}

void Motorluefter_aus() {
  PCF_BRAKE.digitalWrite(FAN_MOT, LOW);
}

void Motorluefter_ein() {
  PCF_BRAKE.digitalWrite(FAN_MOT, HIGH);
}


void Bremse_Zu() {
  PCF_BRAKE.digitalWrite(BR_EN, HIGH);
  PCF_BRAKE.digitalWrite(BR_UP, HIGH);
  PCF_BRAKE.digitalWrite(BR_DOWN, LOW);
}

void Bremse_Neutral() {
  PCF_BRAKE.digitalWrite(BR_EN, LOW);
  PCF_BRAKE.digitalWrite(BR_UP, LOW);
  PCF_BRAKE.digitalWrite(BR_DOWN, LOW);
}

void Bremse_Auf() {
  PCF_BRAKE.digitalWrite(BR_UP, LOW);
  PCF_BRAKE.digitalWrite(BR_EN, HIGH);
  PCF_BRAKE.digitalWrite(BR_DOWN, HIGH);
}

// Service-Functions
/*
void PrintAnalogValues(){
  Serial.print("20V = ");
  Serial.println(analogRead(SENSE_24V));
  Serial.print("Strom = ");
  Serial.println(analogRead(SENSE_STROM));
}

void I2CScanner() {
  delay(500);
  Serial.println("\n=== I2C Scanner ===");
  byte error, address;
  int nDevices;
  Serial.println("Starte Scanvorgang");
  nDevices = 0;
  
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C Gerät gefunden - Adresse: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unbekannter Fehler an Adresse: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("Keine I2C Geräte gefunden\n");
  else
    Serial.println("Scanvorgang Abgeschlosse\n");

  delay(10000);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}
*/