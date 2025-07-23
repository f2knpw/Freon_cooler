
#include "driver/adc.h"
#include <esp_bt.h>



int nbTimeout = 0;
int timeOut = 0;
int nbAlarm = 0;
bool AndroidConnected = false;


//Json
#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson
String res;

//temperature sensors
#include <OneWire.h>
#define PIN_WIRE 1

//#define SEARCH_DS18B20 //comment when addresses are found

#ifdef SEARCH_DS18B20
OneWire ds(PIN_WIRE);  //for searching addresses
#else
#include <DallasTemperature.h>
OneWire oneWire(PIN_WIRE);  //for temperature
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

DeviceAddress coldSensor = { 0x28, 0xBC, 0x2E, 0x75, 0xD0, 0x01, 0x3C, 0xD7 };
DeviceAddress hotSensor = { 0x28, 0x43, 0x82, 0x48, 0xF6, 0x82, 0x3C, 0x79 };
#endif

float coldTemp = 0;
float hotTemp = 0;
float targetTemp = 20;  //temperature we want to reach
#define COLD_OFFSET -0.6
#define HOT_OFFSET -1.4

#define PIN_BAT 3  //ADC pin for battery voltage measurement
float vBat;        //battery voltage

//software PWM for Peltier (sub-Hz very low frequency)
#define PIN_COOLER 6                             //Peltier element
const unsigned long pwmPeriodCooler_ms = 10000;  // 0.1 Hz or 10s
int dutyCycleCooler = 10;                        // 10% duty cycle (in percent, 0-100)
unsigned long previousMillis = 0;
bool coolerState = LOW;  // Current state of the cooler pin
long coolerDuration[101];

//hardware PWM for fan (high frequency)
#define PIN_FAN 7                                       // Define the output pin for FAN
const double pwmFrequency = 10000;                      // 0.1 Hz = 10-second period
const int pwmResolutionBits = 10;                       // PWM Resolution (in bits) gives 1024 steps
const int maxDutyCycle = (1 << pwmResolutionBits) - 1;  // Calculate the maximum duty cycle value based on the resolution
int dutyCycleFan = 0;

//mode of operation
int mode = 0;  //0 - automatic ; 1 - manual

//PID for automatic mode
#include <PID_v1.h>
//Define Variables we'll be connecting to
double setpoint, input, output;
double Kp = 1, Ki = 1, Kd = 0;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);  //we are ona cooler so PID should be reversed

//Preferences
#include <Preferences.h>
Preferences preferences;






#define DEBUG_OUT
//#define xDEBUG
//#define TEST
#define PREFERENCES_DEBUG
#define DEBUG_BLE  //debug bluetooth
//#define DEBUG_VBAT  //show battery voltage

//**************
//BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>

long LastBLEnotification;

//BLE declarations
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26d8"

void BLEnotify(String theString) {
  // if (deviceConnected == true)
  {
#ifdef xDEBUG_BLE
    Serial.print("BLE notify : ");
    Serial.println(theString);
#endif
    char message[21];
    String small = "";  //BLE notification MTU is limited to 20 bytes
    while (theString.length() > 0) {
      small = theString.substring(0, 19);  //cut into 20 chars slices
      theString = theString.substring(19);
      small.toCharArray(message, 20);
      pCharacteristic->setValue(message);
      pCharacteristic->notify();
      delay(3);                        // bluetooth stack will go into congestion, if too many packets are sent
      LastBLEnotification = millis();  //will prevent to send new notification before this one is not totally sent
    }
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
#ifdef DEBUG_BLE
    Serial.println("client connected");
#endif
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
#ifdef DEBUG_BLE
    Serial.println("client disconnected");
#endif
    // Start advertising
    pServer->getAdvertising()->stop();
    delay(100);
    pServer->getAdvertising()->start();
#ifdef DEBUG_BLE
    Serial.println("Waiting a client connection to notify...");
#endif
    delay(100);
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    String test = "";
    if (rxValue.length() > 0) {
#ifdef DEBUG_OUT
      Serial.print("Received : ");
#endif
      for (int i = 0; i < rxValue.length(); i++) {
#ifdef DEBUG_OUT
        Serial.print(rxValue[i]);
#endif
        test = test + rxValue[i];
      }
#ifdef DEBUG_OUT
      Serial.println();
#endif
    }
    readCmd(test);
  }
};


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);


  //BLE
  // Create the BLE Device
  BLEDevice::init("JP Freon");

  // Create the BLE Server
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
#ifdef DEBUG_BLE
  Serial.println("Waiting a client connection...");
#endif

  //Cooler peltier element
  pinMode(PIN_COOLER, OUTPUT);
  digitalWrite(PIN_COOLER, HIGH);  // Start with pin HIGH Mosfet blocked

  //FAN Pwm
  bool success = ledcAttach(PIN_FAN, pwmFrequency, pwmResolutionBits);  //Configure LEDC for PWM using the new ledcAttach function

  if (!success) {
    Serial.println("Failed to configure LEDC! Check your pin, frequency, and resolution settings.");
    while (true)
      ;
  }
  // Initial duty cycle (e.g., 50%)
  int initialDutyCycle = maxDutyCycle / 2;
  ledcWrite(PIN_FAN, maxDutyCycle);                     // FAN will be stopped. Note: ledcWrite now takes the pin directly
  for (int i = 0; i < 101; i++) coolerDuration[i] = 0;  // use to keep track of time when power changed onthe peltier (for each dutycycle value)

  //PID
  setpoint = 20;                  //setpoint will be the desired cold temperature
  myPID.SetSampleTime(1000);      //sampling at 1Hz
  myPID.SetOutputLimits(0, 100);  //tell the PID to range between 0 and the full peltier power
  myPID.SetMode(AUTOMATIC);       //turn the PID on


  Serial.println("***************");
  Serial.println("program started");
  Serial.println("***************");
  Serial.println(" ");


  //Preferences
  preferences.begin("JP Freon", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  mode = preferences.getInt("mode", 0);  //by default automatic mode
  dutyCycleCooler = preferences.getInt("pwr", 0);
  targetTemp = preferences.getInt("targetTemp", 20);

#ifdef PREFERENCES_DEBUG
  Serial.println("_________________");
  Serial.println("read preferences :");
  Serial.print("mode ");
  Serial.println(mode);
  Serial.print("power ");
  Serial.println(dutyCycleCooler);
  Serial.print("target temp ");
  Serial.println(targetTemp);
  Serial.println("_________________");
#endif
  //preferences.end();  // Close the Preferences



  Serial.println(" ");
  Serial.println("start monitoring temperature sensors : \n");
}

//***********************************************************************************************************************
void loop() {
  // esp_task_wdt_reset();  //reset the watchdog


  //*******************
  //acquire all sensors
  //*******************
#ifdef SEARCH_DS18B20  // will display DS18B20 addresses
  byte i;
  byte addr[8];

  if (!ds.search(addr)) {
    Serial.println(" No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  Serial.print(" ROM =");
  for (i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
#else  //normal acquisition when addresses found

  sensors.requestTemperatures();  // Send the command to get temperatures
  //Serial.print("cold temp(°C): ");
  coldTemp = sensors.getTempC(coldSensor) + COLD_OFFSET;
  //Serial.println(coldTemp);
  //Serial.print("hot temp(°C): ");
  hotTemp = sensors.getTempC(hotSensor) + HOT_OFFSET;
  //Serial.println(hotTemp);
#endif

  //battery voltage
  vBat = analogRead(PIN_BAT);
#ifdef DEBUG_VBAT
  Serial.print("battery voltage raw = ");
  Serial.println(vBat);
#endif
  vBat = vBat * 1.67 / 2230;
#ifdef DEBUG_VBAT
  Serial.print("battery voltage = ");
  Serial.print(vBat);
  Serial.println(" V");
#endif
  if (vBat < 3) {                //security to avoid draining (and killing) the battery. deepsleep when under 6V
    pinMode(PIN_COOLER, INPUT);  //switch off fan and cooler
    pinMode(PIN_FAN, INPUT);
    Serial.println("Going to sleep now");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }

  //PID
  input = coldTemp;       //with run the PID on the cold temperature input
  setpoint = targetTemp;  //and use the target temerature aas the setpoint
  myPID.Compute();        //will compute PID every 1s (1Hz is enough as the cold metal plate as a lot of thermal inertia)

  switch (mode) {
    case 0:                      //mode auto
      dutyCycleCooler = output;  //output of the PID is directly the dutycyle for cooler
      break;
    case 1:  //manual mode : use last saved value
      dutyCycleCooler = preferences.getInt("pwr", 20);
      break;
    default:
      dutyCycleCooler = preferences.getInt("pwr", 20);
      break;
  }


  //PWM for peltier (software as ledc does not work at low frequencies)
  unsigned long currentMillis = millis();

  // Calculate ON and OFF times based on the period and duty cycle
  unsigned long onTime_ms = (pwmPeriodCooler_ms * dutyCycleCooler) / 100;
  unsigned long offTime_ms = pwmPeriodCooler_ms - onTime_ms;
  if (dutyCycleCooler < 5) digitalWrite(PIN_COOLER, HIGH);
  else if (dutyCycleCooler > 95) digitalWrite(PIN_COOLER, LOW);
  else {
    if (coolerState == HIGH) {
      // If currently LOW, check if it's time to go HIGH
      if (currentMillis - previousMillis >= offTime_ms) {
        digitalWrite(PIN_COOLER, LOW);
        coolerState = LOW;
        previousMillis = millis();  // Reset the timer
      }
    } else {  // pinState == LOW
      // If currently LOW, check if it's time to go HIGH
      if (currentMillis - previousMillis >= onTime_ms) {
        digitalWrite(PIN_COOLER, HIGH);
        coolerState = HIGH;
        previousMillis = millis();  // Reset the timer
      }
    }
  }
  digitalWrite(LED_BUILTIN, digitalRead(PIN_COOLER));  //mirror the cooler power
  for (int i = 0; i < dutyCycleCooler; i++) {
    coolerDuration[i] = millis();
  }
  //PWM for fan
  for (int j = 0; j < 101; j++) {
    if ((millis() - coolerDuration[j]) > 100 * (100 - j)) {  //postpon decrease with 10s
      dutyCycleFan = maxDutyCycle * (100 - j) / 100;         // Adjust dutycycle (0% 100% should be mapped betwwen 1024 and 0 due to mosfets)
      break;
    }
  }
  ledcWrite(PIN_FAN, dutyCycleFan);  //output PWM


  //send sensors data as fast as possible (else uncomment the following line)
  if (((millis() - LastBLEnotification) > 1000))  // send UDP message to Android App (no need to be connected, a simple notification, send and forget)
  {
    res = "{\"C\": " + String(coldTemp) + ",\"H\": " + String(hotTemp) + ",\"V\": " + String(vBat) + ",\"D\": " + String(dutyCycleCooler) + "}";
    LastBLEnotification = millis();
    BLEnotify(res);  //try to send via Bluetooth
  }





  if (((millis() - timeOut) > 6000) && (AndroidConnected == 1)) {
    AndroidConnected = 0;
    Serial.println("lost connection with Android phone, switch to Manual mode");
  }
}  //end of Loop





float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




void readCmd(String test) {
  if (test.startsWith("{"))  //then it may contain JSON
  {
    StaticJsonDocument<2000> doc;
    DeserializationError error = deserializeJson(doc, test);
    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      Serial.println("deserializeJson() failed");  //answer with error : {"answer" : "error","detail":"decoding failed"}

    } else {
      // Fetch values --> {"Cmd":"CalF"}
      String Cmd = doc["Cmd"];
      if (Cmd == "Mode") {
        Serial.println("change mode... ");
        String value = doc["value"];
        mode = value.toInt();
        preferences.putInt("mode", mode);
      } else if (Cmd == "Pwr")  //front scale calibration
      {
        String value = doc["value"];
        dutyCycleCooler = value.toInt();
        preferences.putInt("pwr", dutyCycleCooler);
      } else if (Cmd == "Temp")  //front scale calibration
      {
        String value = doc["value"];
        targetTemp = value.toInt();
        preferences.putInt("targetTemp", targetTemp);
        Serial.print("target temp = ");
        Serial.println(targetTemp);
      }
    }
  }
}





// ---------------------------------------------- end ----------------------------------------------
