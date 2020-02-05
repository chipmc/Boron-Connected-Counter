/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "application.h"
#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Boron-Connected-Counter/src/Boron-Connected-Counter.ino"
/*
* Project Cellular-MMA8452Q - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:10 January 2018
*/

/*  The idea of this release is to use the new sensor model which should work with multiple sensors
    Both utility and solar implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge
*/

//v1  - Adapted from the Particle Electron version Cellular Pressure next


void setup();
void loop();
void recordCount();
void sendEvent();
void UbidotsHandler(const char *event, const char *data);
void takeMeasurements();
void getSignalStrength();
int getTemperature();
void sensorISR();
void watchdogISR();
void petWatchdog();
void PMICreset();
bool connectToParticle();
bool disconnectFromParticle();
bool notConnected();
int resetFRAM(String command);
int resetCounts(String command);
int hardResetNow(String command);
int setDebounce(String command);
int sendNow(String command);
void resetEverything();
int setSolarMode(String command);
int setverboseMode(String command);
int setTimeZone(String command);
int setOpenTime(String command);
int setCloseTime(String command);
int setLowPowerMode(String command);
bool meterParticlePublish(void);
void publishStateTransition(void);
void fullModemReset();
void dailyCleanup();
#line 18 "/Users/chipmc/Documents/Maker/Particle/Projects/Boron-Connected-Counter/src/Boron-Connected-Counter.ino"
namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x20                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 2;                    // Increment this number each time the memory map is changed

struct systemStatus_structure {                     // currently 14 bytes long
  byte structuresVersion;                           // Version of the data structures (system and data)
  uint8_t currentRelease;                           // current software release                          
  bool metricUnits;                                 // Status of key system states
  bool connectedStatus;
  bool verboseMode; 
  bool solarPowerMode;
  bool lowPowerMode;
  bool lowBatteryMode;
  uint8_t stateOfCharge;                            // Battery charge level
  byte powerState;                                  // Stores the current power state
  uint16_t debounce;                                // debounce value in milliseconds (up to 65536)
  uint8_t resetCount;                               // reset count of device (0-256)
  byte timezone;                                    // Time zone value -12 to +12
  byte openTime;                                    // Hour the park opens (0-23)
  byte closeTime;                                   // Hour the park closes (0-23)
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
} sysStatus;

struct currentCounts_structure {                    // currently 10 bytes long
  uint16_t hourlyCount;                             // In period hourly count
  uint16_t hourlyCountInFlight;                     // In flight and waiting for Ubidots to confirm
  uint16_t dailyCount;                              // In period daily count
  unsigned long lastCountTime;                      // When did we record our last count
  uint16_t temperature;                             // Current Temperature
  byte alertCount;                                  // What is the current alert count
  byte maxMinValue;                                 // Highest count in one minute in the current period
} current;

// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "MCP79410RK.h"                             // Real Time Clock
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library

const uint8_t softwareRelease = 1;                  // We set this in the code itself - not memory

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;                           // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                                         //Initalize the PMIC class so you can call the Power Management functions below.
MCP79410 rtc;                                       // Rickkas MCP79410 libarary
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Pin Constants - Boron Carrier Board v1.2a
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int deepSleepPin =  D6;                       // Power Cycles the Electron and the Carrier Board
const int donePin =       D5;                       // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int intPin =        MOSI;                     // Pressure Sensor inerrupt pin
const int disableModule = SS;                       // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      MISO;                     // Allows us to control the indicator LED on the sensor board

// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
unsigned long lastPublish = 0;                      // Can only publish 1/sec on avg and 4/sec burst

// Program Variables
bool awokeFromNap = false;                          // In low power mode, we can't use standard millis to debounce
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentCountsWriteNeeded = false;

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Pressure Sensor Variables
char debounceStr[8] = "NA";                         // String to make debounce more readable on the mobile app
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered


void setup()                                        // Note: Disconnected Setup()
{
  /* Setup is run for three reasons once we deploy a sensor:
       1) When you deploy the sensor
       2) Each hour while the device is sleeping
       3) After a reset event
    All three of these have some common code - this will go first then we will set a conditional
    to determine which of the three we are in and finish the code
  */
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  pinResetFast(donePin);
  pinResetFast(deepSleepPin);
  // Pressure / PIR Module Pin Setup
  pinMode(intPin,INPUT_PULLDOWN);                   // pressure sensor interrupt
  pinMode(disableModule,OUTPUT);                    // Turns on the module when pulled low
  pinResetFast(disableModule);                      // Turn on the module - send high to switch off board
  pinMode(ledPower,OUTPUT);                         // Turn on the lights
  pinSetFast(ledPower);                             // Turns on the LED on the pressure sensor board

  pinMode(donePin,OUTPUT);                          // Allows us to pet the watchdog
  pinMode(deepSleepPin,OUTPUT);                     // For a hard reset active HIGH

  petWatchdog();                                    // Pet the watchdog - This will reset the watchdog time period
  attachInterrupt(wakeUpPin, watchdogISR, RISING);  // The watchdog timer will signal us and we have to respond

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", current.hourlyCount);                // Define my Particle variables
  Particle.variable("DailyCount", current.dailyCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",sysStatus.currentRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",sysStatus.lowPowerMode);
  Particle.variable("OpenTime",sysStatus.openTime);
  Particle.variable("CloseTime",sysStatus.closeTime);
  Particle.variable("Debounce",debounceStr);
  Particle.variable("Alerts",current.alertCount);

  Particle.function("resetFRAM", resetFRAM);                          // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setverboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("Set-Debounce",setDebounce);


  // This block of code overcomes an issue that should be fixed in deviceOS@1.5.0
  detachInterrupt(LOW_BAT_UC);
  // Delay for up to two system power manager loop invocations
  delay(2000);
  // Change PMIC settings
  power.setInputVoltageLimit(4640);

  // Load FRAM and reset variables to their correct values
  fram.begin();                                                       // Initialize the FRAM module

  fram.get(FRAM::systemStatusAddr,sysStatus);                      // Loads the System Status array from FRAM

  int tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                             // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                     // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                         // See if this worked
    if (tempVersion != FRAMversionNumber) state = ERROR_STATE;        // Device will not work without FRAM
  }

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    fram.put(FRAM::systemStatusAddr, sysStatus);  // If so, store incremented number - watchdog must have done This
  }

  Time.zone(sysStatus.timezone);                                   // Set the Time Zone for our device 

  PMICreset();                                                        // Executes commands that set up the PMIC for Solar charging

  if (!digitalRead(userSwitch)) {                                     // Rescue mode to locally take lowPowerMode so you can connect to device
    sysStatus.lowPowerMode = false;                                // Press the user switch while resetting the device
    sysStatus.connectedStatus = true;                               // Set the stage for the devic to get connected
    if ((Time.hour() > sysStatus.closeTime || Time.hour() < sysStatus.openTime))  {  // Device may also be sleeping due to time or TimeZone setting
      sysStatus.openTime = 0;                                       // Only change these values if it is an issue
      sysStatus.closeTime = 23;                                    // Reset open and close time values to ensure device is awake
    }
    fram.put(FRAM::systemStatusAddr,sysStatus);                  // Write it to the register
  }

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  if (Time.day() != Time.day(current.lastCountTime)) {
    resetEverything();                                                // Zero the counts for the new day
    if (sysStatus.solarPowerMode && !sysStatus.lowPowerMode) {
      setLowPowerMode("1");                                           // If we are running on solar, we will reset to lowPowerMode at Midnight
    }
  }
  if ((Time.hour() > sysStatus.closeTime || Time.hour() < sysStatus.openTime)) {}         // The park is closed - sleep
  else {                                                              // Park is open let's get ready for the day
    attachInterrupt(intPin, sensorISR, RISING);                       // Pressure Sensor interrupt from low to high
    if (sysStatus.connectedStatus) connectToParticle();            // Only going to connect if we are in connectionMode
    takeMeasurements();                                               // Populates values so you can read them before the hour
    stayAwake = stayAwakeLong;                                        // Keeps Electron awake after reboot - helps with recovery
  }

  pinResetFast(ledPower);                                             // Turns off the LED on the sensor board

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code
}

void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (watchdogFlag) {
      petWatchdog();                                                  // Watchdog flag is raised - time to pet the watchdog
      if (Particle.connected() && sysStatus.verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("Watchdog","Petted",PRIVATE);
      }
    }
    if (sensorDetect) recordCount();                                  // The ISR had raised the sensor flag
    if (current.hourlyCountInFlight) {                  // Cleared here as there could be counts coming in while "in Flight"
      current.hourlyCount -= current.hourlyCountInFlight;  // Confirmed that count was recevied - clearing
      current.hourlyCountInFlight = 0;                  // Zero out the counts until next reporting period
      current.maxMinValue = 0;
      current.alertCount = 0;
      fram.put(FRAM::currentCountsAddr,current);
      if (Time.hour() == 0) resetEverything();                        // We have reported for the previous day - reset for the next - only needed if no sleep
    }
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;  // When in low power mode, we can nap between taps
    if (Time.hour() != Time.hour(current.lastCountTime)) state = REPORTING_STATE;  // We want to report on the hour but not after bedtime
    if ((Time.hour() > sysStatus.closeTime || Time.hour() < sysStatus.openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    detachInterrupt(intPin);                                          // Done sensing for the day
    pinSetFast(disableModule);                                        // Turn off the pressure module for the hour
    if (current.hourlyCount) {                                          // If this number is not zero then we need to send this last count
      state = REPORTING_STATE;
      break;
    }
    if (sysStatus.connectedStatus) disconnectFromParticle();                     // Disconnect cleanly from Particle
    digitalWrite(blueLED,LOW);                                        // Turn off the LED
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    System.sleep(SLEEP_MODE_DEEP,wakeInSeconds);                      // Very deep sleep till the next hour - then resets
    } break;

  case NAPPING_STATE: {  // This state puts the device in low power mode quickly
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (sensorDetect) break;                                   // Don't nap until we are done with event
    if (sysStatus.connectedStatus) {                        // If we are in connected mode
      disconnectFromParticle();                                       // Disconnect from Particle
      sysStatus.connectedStatus = false;
      fram.put(FRAM::systemStatusAddr,sysStatus);
    }
    stayAwake = sysStatus.debounce;                                             // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour                                                 
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    petWatchdog();                                                    // Reset the watchdog timer interval
    System.sleep(intPin, RISING, wakeInSeconds);                      // Sensor will wake us with an interrupt or timeout at the hour
    if (sensorDetect) {
       awokeFromNap=true;                                             // Since millis() stops when sleeping - need this to debounce
       stayAwakeTimeStamp = millis();
    }
    state = IDLE_STATE;                                               // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    } break;

  case REPORTING_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!sysStatus.connectedStatus) connectToParticle();    // Only attempt to connect if not already New process to get connected
    //sysStatus.disableUpdates();                                          // Don't want an update while we are reporting
    if (Particle.connected()) {
      if (Time.hour() == sysStatus.closeTime) dailyCleanup();                   // Once a day, clean house
      takeMeasurements();                                             // Update Temp, Battery and Signal Strength values
      sendEvent();                                                    // Send data to Ubidots
      state = RESP_WAIT_STATE;                                        // Wait for Response
    }
    else state = ERROR_STATE;
    break;

  case RESP_WAIT_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                // Response received back to IDLE state
    {
      state = IDLE_STATE;
      stayAwake = stayAwakeLong;                                      // Keeps Electron awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      waitUntil(meterParticlePublish);
      Particle.publish("spark/device/session/end", "", PRIVATE);      // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {
      if (sysStatus.resetCount <= 3) {                                          // First try simple reset
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - sysStatus.lastHookResponse > 7200L) { //It has been more than two hours since a sucessful hook response
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                  // Zero the ResetCount
        fram.put(FRAM::systemStatusAddr,sysStatus);
        digitalWrite(deepSleepPin,HIGH);                              // This will cut all power to the Boron AND everything it powers
        rtc.setAlarm(10);
      }
      else {                                                          // If we have had 3 resets - time to do something more
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                  // Zero the ResetCount
        fram.put(FRAM::systemStatusAddr,sysStatus);
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
  rtc.loop();                                                         // keeps the clock up to date
}

void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
  static byte currentMinutePeriod;                                    // Current minute

  pinSetFast(blueLED);                                                // Turn on the blue LED

  if (millis() - current.lastCountTime >= sysStatus.debounce || awokeFromNap) {          // If this event is outside the debounce time, proceed
    current.lastCountTime = millis();
    awokeFromNap = false;                                             // Reset the awoke flag

    if (currentMinutePeriod != Time.minute()) {                       // Done counting for the last minute
      currentMinutePeriod = Time.minute();                            // Reset period
      current.maxMinValue = 1;                                         // Reset for the new minute
    }
    current.maxMinValue++;

    current.hourlyCount++;                                                // Increment the PersonCount
    current.dailyCount++;                                                 // Increment the PersonCount
    if (sysStatus.verboseMode && Particle.connected()) {
      char data[256];                                                    // Store the date in this character array - not global
      snprintf(data, sizeof(data), "Count, hourly: %i, daily: %i",current.hourlyCount,current.dailyCount);
      waitUntil(meterParticlePublish);
      Particle.publish("Count",data, PRIVATE);                                   // Helpful for monitoring and calibration
    }
  }
  else if(sysStatus.verboseMode && Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("Event","Debounced", PRIVATE);
  }

  if (!digitalRead(userSwitch)) {                                     // A low value means someone is pushing this button - will trigger a send to Ubidots and take out of low power mode
    connectToParticle();                                              // Get connected to Particle
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Mode","Normal Operations", PRIVATE);
    sysStatus.connectedStatus = true;                              // Make sure we are connected
    sysStatus.lowPowerMode = false;
    fram.put(FRAM::systemStatusAddr,sysStatus);                    // Write the System Status structure to FRAM
  }

  fram.put(FRAM::currentCountsAddr, current);                   // Write updated values to FRAM
  pinResetFast(blueLED);                                              // Turn off the blue LED
  sensorDetect = false;                                               // Reset the flag
}


void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i, \"resets\":%i, \"alerts\":%i, \"maxmin\":%i}",current.hourlyCount, current.dailyCount, sysStatus.stateOfCharge, current.temperature, sysStatus.resetCount, current.alertCount, current.maxMinValue);
  Particle.publish("Ubidots-Car-Hook", data, PRIVATE);
  webhookTimeStamp = millis();
  if(Time.hour() == 00) current.hourlyCount++;                        // Ensures we don't have a zero here at midnigtt
  current.hourlyCountInFlight = current.hourlyCount;                  // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  dataInFlight = true;                                                // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response                                                                   
  char dataCopy[strlen(data)+1];                                      // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back
  strncpy(dataCopy, data, sizeof(dataCopy));                          // data needs to be copied since if (Particle.connected()) Particle.publish() will clear it
  if (!strlen(dataCopy)) {                                            // Copy - overflow safe
    waitUntil(meterParticlePublish);                                  // First check to see if there is any data
    if (Particle.connected()) Particle.publish("Ubidots Hook", "No Data", PRIVATE);
    return;
  }
  int responseCode = atoi(dataCopy);                                  // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("State","Response Received", PRIVATE);
    current.lastCountTime = Time.now();                     // Record the last successful Webhook Response
    dataInFlight = false;                                             // Data has been received
  }
  else if (Particle.connected()) Particle.publish("Ubidots Hook", dataCopy, PRIVATE);                    // Publish the response code
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready
  getTemperature();                                                   // Get Temperature at startup as well
  sysStatus.stateOfCharge = int(batteryMonitor.getSoC());                // Percentage of full charge
  fram.put(FRAM::systemStatusAddr,sysStatus);
}


void getSignalStrength()
{
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  return current.temperature;
  fram.put(FRAM::currentCountsAddr,current);
}

// Here are the various hardware and timer interrupt service routines
void sensorISR()
{
  sensorDetect = true;                              // sets the sensor flag for the main loop
}

void watchdogISR()
{
  watchdogFlag = true;
}

void petWatchdog()
{
  digitalWriteFast(donePin, HIGH);                                        // Pet the watchdog
  digitalWriteFast(donePin, LOW);
  watchdogFlag = false;
}

// Power Management function
void PMICreset() {
  power.begin();                                                      // Settings for Solar powered power management
  power.disableWatchdog();
  if (sysStatus.solarPowerMode) {
    power.setInputVoltageLimit(4840);                                 // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                                  // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                              // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                                     // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    power.setInputVoltageLimit(4208);                                 // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                                 // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                              // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                                     // default is 4.112V termination voltage
  }
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    if(sensorDetect) recordCount(); // service the interrupt every 10 seconds
    Particle.process();
  }
  if (Particle.connected()) {
    sysStatus.connectedStatus = 1;
    systemStatusWriteNeeded = true;
    return 1;                               // Were able to connect successfully
  }
  else {
    return 0;                                                    // Failed to connect
  }
}

bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
{
  Particle.disconnect();
  waitFor(notConnected, 15000);                                   // make sure before turning off the cellular modem
  Cellular.off();
  delay(2000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                             // Companion function for disconnectFromParticle
    return !Particle.connected();
}

int resetFRAM(String command)                                         // Will reset the local counts
{
  if (command == "1")
  {
    fram.erase();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)                                       // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    current.dailyCount = 0;                                           // Reset Daily Count in memory
    current.hourlyCount = 0;                                          // Reset Hourly Count in memory
    sysStatus.resetCount = 0;                                            // If so, store incremented number - watchdog must have done This
    current.alertCount = 0;                                           // Reset count variables   
    current.hourlyCountInFlight = 0;                                  // In the off-chance there is data in flight
    dataInFlight = false;
    currentCountsWriteNeeded = true;                                  // Make sure we write to FRAM back in the main loop
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    Particle.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
    digitalWrite(deepSleepPin,HIGH);                              // This will cut all power to the Boron AND everything it powers
    rtc.setAlarm(10);
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int setDebounce(String command)                                       // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  float inputDebounce = strtof(command,&pEND);                        // Looks for the first float and interprets it
  if ((inputDebounce < 0.0) | (inputDebounce > 5.0)) return 0;        // Make sure it falls in a valid range or send a "fail" result
  sysStatus.debounce = int(inputDebounce*1000);                          // debounce is how long we must space events to prevent overcounting
  systemStatusWriteNeeded = true;
  snprintf(debounceStr,sizeof(debounceStr),"%2.1f sec",inputDebounce);
  if (sysStatus.verboseMode && Particle.connected()) {                                                  // Publish result if feeling verbose
    waitUntil(meterParticlePublish);
    Particle.publish("Debounce",debounceStr, PRIVATE);
  }
  return 1;                                                           // Returns 1 to let the user know if was reset
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

void resetEverything() {                                            // The device is waking up in a new day or is a new install
  current.dailyCount = 0;                              // Reset the counts in FRAM as well
  current.hourlyCount = 0;
  current.hourlyCountInFlight = 0;                    
  current.lastCountTime = Time.now();                      // Set the time context to the new day
  sysStatus.resetCount = current.alertCount = 0;           // Reset everything for the day
  fram.put(FRAM::currentCountsAddr,current);
  fram.put(FRAM::systemStatusAddr,sysStatus);
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    PMICreset();                                               // Change the power management Settings
    fram.put(FRAM::systemStatusAddr,sysStatus);
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    fram.put(FRAM::systemStatusAddr,sysStatus);
    PMICreset();                                                // Change the power management settings
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setverboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.timezone = tempTimeZoneOffset;
  Time.zone((float)sysStatus.timezone);
  systemStatusWriteNeeded = true;                                             // Need to store to FRAM back in the main loop
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Time",Time.timeStr(t), PRIVATE);
  return 1;
}

int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                                    // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                            // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  systemStatusWriteNeeded = true;                                            // Need to store to FRAM back in the main loop
  snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}

int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (sysStatus.verboseMode && Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode","Low Power", PRIVATE);
    }
    if (sysStatus.connectedStatus) {                                     // If we are in connected mode
      Particle.disconnect();                                          // Otherwise Electron will attempt to reconnect on wake
      sysStatus.connectedStatus = false;
      Cellular.off();
      delay(1000);                                                    // Bummer but only should happen once an hour
    }
    sysStatus.lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (sysStatus.verboseMode && Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode","Normal Operations", PRIVATE);
    }
    if (!sysStatus.connectedStatus) {
      sysStatus.connectedStatus = true;
      Particle.connect();
      waitFor(Particle.connected,60000);                                // Give us 60 seconds to connect
      Particle.process();
    }
    sysStatus.lowPowerMode = false;
  }
  systemStatusWriteNeeded = true;
  return 1;
}

bool meterParticlePublish(void)
{
  static unsigned long lastPublish=0;                                   // Initialize and store value here
  if(millis() - lastPublish >= 1000) {                                  // Particle rate limits at 1 publish per second
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("State Transition",stateTransitionString, PRIVATE);
  }
  Serial.println(stateTransitionString);
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=15\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

void dailyCleanup() {                                                 // Function to clean house at the end of the day
  waitUntil(meterParticlePublish);
  Particle.publish("Daily Cleanup","Running", PRIVATE);               // Make sure this is being run

  sysStatus.verboseMode = false;
 
  Particle.syncTime();                                                // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                               // Wait for up to 30 seconds for the SyncTime to complete

  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 70) {                        // If the battery is being discharged
    sysStatus.lowPowerMode = true;
    sysStatus.connectedStatus = false;
  }
  fram.put(FRAM::systemStatusAddr,sysStatus);
}
