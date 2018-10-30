#include <EEPROM.h>

#include <EEPROMAnything.h>

#include <OneWire.h>

// *** SendandReceiveArguments ***

// This example expands the previous SendandReceive example. The Arduino will now receive multiple 
// and sent multiple float values. 
// It adds a demonstration of how to:
// - Return multiple types status; It can return an Acknowlegde and Error command
// - Receive multiple parameters,
// - Send multiple parameters
// - Call a function periodically

#include <CmdMessenger.h>  // CmdMessenger


// configuration
struct config_t
{
    char UUID[36];
} configuration;

// random generate uuid
char alphabet[] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

// record last activated millis for each digital pin
uint32_t activations[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);

// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
  // Commands
  // kAcknowledge         , // 0 Command to acknowledge that cmd was received
  kError               , // 0 Command to report error
  kUUID                , // 1 Command to request UUID
  kUUIDResult          , // 2 Command to return UUID
  kSensorReading       , // 3 Command to request sensor reading (pinType, pinNumber, driver, measurementType)
  kSensorReadingResult , // 4 Command to report sensor result
  kSetRelay            , // 5 Command to set relay
  kSetRelayResult      , // 6 Command to confirm relay
  kDisabledOutput      , // 7 Command to notify of disabled output
};

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.

void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kUUID, OnUUID);
  cmdMessenger.attach(kSensorReading, OnSensorReading);
  cmdMessenger.attach(kSetRelay, OnSetRelay);
}

// ------------------  C A L L B A C K S -----------------------

// Called when a received command has no attached function
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError,"Command without attached callback");
}

// Callback function that responds that Arduino is ready (has booted up)
void OnArduinoReady()
{
  // OnUUID();
}

void OnUUID()
{
  cmdMessenger.sendCmdStart(kUUIDResult);
  cmdMessenger.sendCmdArg(configuration.UUID);
  cmdMessenger.sendCmdEnd();
}

void OnSensorReading()
{
  String pinType = cmdMessenger.readStringArg();
  int sensorPin = cmdMessenger.readBinArg<int>();
  String driver = cmdMessenger.readStringArg();
  String measurementType = cmdMessenger.readStringArg();
  
  if (driver == "onewire" && measurementType == "temp" && sensorPin > -1 && sensorPin < 14 && pinType == "digital") {
    readOneWireTemp(sensorPin);
    return;
  }
  
  cmdMessenger.sendCmd(kError, "No valid driver for the specified sensor configuration.");
}

void readOneWireTemp(int sensorPin)
{
  OneWire ds(sensorPin);
  byte addr[8];
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  float celsius, fahrenheit;
  
  pinMode(sensorPin, INPUT);
  
  if (!ds.search(addr)) {
    cmdMessenger.sendCmd(kError, "No OneWire sensor found on this pin");
    return;
  }
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
    cmdMessenger.sendCmd(kError, "CRC is not valid.");
    return;
  }
  
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      cmdMessenger.sendCmd(kError, "Device is not a DS18x20 family device.");
      return;
  }
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);
  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  
  cmdMessenger.sendCmdStart(kSensorReadingResult);
  cmdMessenger.sendCmdBinArg(sensorPin);
  cmdMessenger.sendCmdBinArg(celsius);
  cmdMessenger.sendCmdEnd();
}

void OnSetRelay()
{
  int relayPin;
  int value;
  
  relayPin = cmdMessenger.readBinArg<int>();
  value = cmdMessenger.readBinArg<int>();
  
  if (relayPin < 0 || relayPin > 13) {
    cmdMessenger.sendCmd(kError, "Not valid pin position");
    return;
  }
  
  if (value < 0 || value > 1) {
    cmdMessenger.sendCmd(kError, "Not valid relay value");
    return;
  }
  
  pinMode(relayPin, OUTPUT);
  if (value == 0) {
    digitalWrite(relayPin, LOW);
    activations[relayPin] = 0;
    cmdMessenger.sendCmdStart(kSetRelayResult);
    cmdMessenger.sendCmdBinArg(relayPin);
    cmdMessenger.sendCmdBinArg(0);
    cmdMessenger.sendCmdEnd();
    return;
  } else {
    digitalWrite(relayPin, HIGH);
    activations[relayPin] = millis();
    cmdMessenger.sendCmdStart(kSetRelayResult);
    cmdMessenger.sendCmdBinArg(relayPin);
    cmdMessenger.sendCmdBinArg(1);
    cmdMessenger.sendCmdEnd();
    return;
  }
}

bool isPinOutput(int pin)
{
  // the first ROM byte indicates which chip
  switch (pin) {
    case 0:
      return DDRD & _BV(0);
    case 1:
      return DDRD & _BV(1);
    case 2:
      return DDRD & _BV(2);
    case 3:
      return DDRD & _BV(3);
    case 4:
      return DDRD & _BV(4);
    case 5:
      return DDRD & _BV(5);
    case 6:
      return DDRD & _BV(6);
    case 7:
      return DDRD & _BV(7);
    case 8:
      return DDRB & _BV(0);
    case 9:
      return DDRB & _BV(1);
    case 10:
      return DDRB & _BV(2);
    case 11:
      return DDRB & _BV(3);
    case 12:
      return DDRB & _BV(4);
    case 13:
      return DDRB & _BV(5);
    default:
      return false;
  }
}

void generateUUID(char* output) {
  output[0] = randomChar();
  output[1] = randomChar();
  output[2] = randomChar();
  output[3] = randomChar();
  output[4] = randomChar();
  output[5] = randomChar();
  output[6] = randomChar();
  output[7] = randomChar();
  output[8] = '-';
  output[9] = randomChar();
  output[10] = randomChar();
  output[11] = randomChar();
  output[12] = randomChar();
  output[13] = '-';
  output[14] = randomChar();
  output[15] = randomChar();
  output[16] = randomChar();
  output[17] = randomChar();
  output[18] = '-';
  output[19] = randomChar();
  output[20] = randomChar();
  output[21] = randomChar();
  output[22] = randomChar();
  output[23] = '-';
  output[24] = randomChar();
  output[25] = randomChar();
  output[26] = randomChar();
  output[27] = randomChar();
  output[28] = randomChar();
  output[29] = randomChar();
  output[30] = randomChar();
  output[31] = randomChar();
  output[32] = randomChar();
  output[33] = randomChar();
  output[34] = randomChar();
  output[35] = randomChar();
}

char randomChar(void) {
  return alphabet[random(1,36) - 1];
}

// Safety check for all relays
// when serial is 0 disable all
// when activation is 0 and pin is output and pin is high disable
// when activation has elapsed over 10s and pin is output and pin is high disable
void checkOutputs()
{
  for (int i = 0; i < 14; i++) {
    
    if (isPinOutput(i) && digitalRead(i) == HIGH) {
      if (!Serial || activations[i] == 0 || (activations[i] > 0 && millis() - activations[i] > 10000)) {
        digitalWrite(i, LOW);
        activations[i] = 0;
        
        // if (Serial) {
        //   cmdMessenger.sendCmdStart(kDisabledOutput);
        //   cmdMessenger.sendCmdArg(i);
        //   cmdMessenger.sendCmdArg(0);
        //   cmdMessenger.sendCmdEnd();
        // }
      }
    }
  }
}

void eepromInitializeUUID()
{
  // random seed
  randomSeed(analogRead(0) * 3 + analogRead(1) * 5 + analogRead(2) * 2 + analogRead(3) + analogRead(4) * 2 + analogRead(5));
  
  // read uuid from EEPROM
  EEPROM_readAnything(0, configuration);
  
  bool is_valid = (strlen(configuration.UUID) == 36 && configuration.UUID[8] == '-' && configuration.UUID[13] == '-' && configuration.UUID[18] == '-' && configuration.UUID[23] == '-');
  
  if (!is_valid) {
    generateUUID(configuration.UUID);
    EEPROM_writeAnything(0, configuration);
  }
}

// ------------------ M A I N  ----------------------

// Setup function
void setup() 
{
  
  // Listen on serial connection for messages from the pc
  Serial.begin(115200); 
  
  // Initialize with all outputs off
  checkOutputs();

  // Adds newline to every command
  cmdMessenger.printLfCr();   

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Send the status to the PC that says the Arduino has booted
  // cmdMessenger.sendCmd(kAcknowledge,"Arduino has started!");

  eepromInitializeUUID();
  
  // account for what seems to be weird behavior on pin 13 on the revision 3 board
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

// Loop function
void loop() 
{
   // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();

  // Disable inactive outputs that have not received a command within the last 10 seconds
  checkOutputs();
}
