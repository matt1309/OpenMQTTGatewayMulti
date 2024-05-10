# 1 "C:\\Users\\MATTJA~1\\AppData\\Local\\Temp\\tmpkk4ubmsr"
#include <Arduino.h>
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
# 28 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
#include <Arduino.h>
#include <SD.h>
#include <math.h>

#include "User_config.h"
# 44 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
#if defined(ZgatewayRF) || defined(ZgatewayIR) || defined(ZgatewaySRFB) || defined(ZgatewayWeatherStation) || defined(ZgatewayRTL_433)

struct ReceivedSignal {
  SIGNAL_SIZE_UL_ULL value;
  uint32_t time;
};
# if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
ReceivedSignal receivedSignal[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
# else
ReceivedSignal receivedSignal[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
# endif
#define struct_size (sizeof(receivedSignal) / sizeof(ReceivedSignal))
#endif

#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

unsigned long timer_sys_measures = 0;


unsigned long timer_sys_checks = 0;

#define ARDUINOJSON_USE_LONG_LONG 1
#define ARDUINOJSON_ENABLE_STD_STRING 1

# include <queue>
int queueLength = 0;
unsigned long queueLengthSum = 0;
unsigned long blockedMessages = 0;
int maxQueueLength = 0;
# ifndef QueueSize
#define QueueSize 18
# endif

#endif





#if defined(DEEP_SLEEP_IN_US) || defined(ESP32_EXT0_WAKE_PIN)
bool ready_to_sleep = false;
#endif

#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <PubSubClient.h>

#include <string>

#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
struct JsonBundle {
  StaticJsonDocument<JSON_MSG_BUFFER> doc;
};

std::queue<JsonBundle> jsonQueue;

# ifdef ESP32

SemaphoreHandle_t xQueueMutex;
# endif

#endif

StaticJsonDocument<JSON_MSG_BUFFER> modulesBuffer;
StaticJsonDocument<JSON_MSG_BUFFER> jsonSYSCONFIGBuffer;
JsonArray modules = modulesBuffer.to<JsonArray>();
bool ethConnected = false;

#ifndef ZgatewayGFSunInverter



struct GfSun2000Data {};
#endif


#if defined(ZwebUI) && defined(ESP32)
# include "config_WebUI.h"
#endif
#if defined(ZgatewayRF) || defined(ZgatewayRF2) || defined(ZgatewayPilight) || defined(ZactuatorSomfy) || defined(ZgatewayRTL_433)
# include "config_RF.h"
#endif
#ifdef ZgatewayWeatherStation
# include "config_WeatherStation.h"
#endif
#ifdef ZgatewayGFSunInverter
# include "config_GFSunInverter.h"
#endif
#ifdef ZgatewayLORA
# include "config_LORA.h"
#endif
#ifdef ZgatewaySRFB
# include "config_SRFB.h"
#endif
#ifdef ZgatewayBT
# include "config_BT.h"
#endif
#ifdef ZgatewayIR
# include "config_IR.h"
#endif
#ifdef Zgateway2G
# include "config_2G.h"
#endif
#ifdef ZactuatorONOFF
# include "config_ONOFF.h"
#endif
#ifdef ZsensorINA226
# include "config_INA226.h"
#endif
#ifdef ZsensorHCSR501
# include "config_HCSR501.h"
#endif
#ifdef ZsensorADC
# include "config_ADC.h"
#endif
#ifdef ZsensorBH1750
# include "config_BH1750.h"
#endif
#ifdef ZsensorMQ2
# include "config_MQ2.h"
#endif
#ifdef ZsensorTEMT6000
# include "config_TEMT6000.h"
#endif
#ifdef ZsensorTSL2561
# include "config_TSL2561.h"
#endif
#ifdef ZsensorBME280
# include "config_BME280.h"
#endif
#ifdef ZsensorHTU21
# include "config_HTU21.h"
#endif
#ifdef ZsensorLM75
# include "config_LM75.h"
#endif
#ifdef ZsensorAHTx0
# include "config_AHTx0.h"
#endif
#ifdef ZsensorRN8209
# include "config_RN8209.h"
#endif
#ifdef ZsensorHCSR04
# include "config_HCSR04.h"
#endif
#ifdef ZsensorC37_YL83_HMRD
# include "config_C37_YL83_HMRD.h"
#endif
#ifdef ZsensorDHT
# include "config_DHT.h"
#endif
#ifdef ZsensorSHTC3
# include "config_SHTC3.h"
#endif
#ifdef ZsensorDS1820
# include "config_DS1820.h"
#endif
#ifdef ZgatewayRFM69
# include "config_RFM69.h"
#endif
#ifdef ZsensorGPIOInput
# include "config_GPIOInput.h"
#endif
#ifdef ZsensorGPIOKeyCode
# include "config_GPIOKeyCode.h"
#endif
#ifdef ZsensorTouch
# include "config_Touch.h"
#endif
#ifdef ZmqttDiscovery
# include "config_mqttDiscovery.h"
#endif
#ifdef ZactuatorFASTLED
# include "config_FASTLED.h"
#endif
#ifdef ZactuatorPWM
# include "config_PWM.h"
#endif
#ifdef ZactuatorSomfy
# include "config_Somfy.h"
#endif
#if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5STACK) || defined(ZboardM5TOUGH)
# include "config_M5.h"
#endif
#if defined(ZdisplaySSD1306)
# include "config_SSD1306.h"
#endif
#if defined(ZgatewayRS232)
# include "config_RS232.h"
#endif


void setupTLS(bool self_signed = false, uint8_t index = 0);
const char* getValueFromKeys(const JsonVariant& root, const String& keys);
double parseNumber(const String& expression, size_t& index);
double evaluateFunction(double x, const String& functionName);
double evaluatePrimary(double x, const String& expression, size_t& index);
double evaluateFactor(double x, const String& expression, size_t& index);
double evaluateExpression(double x, const String& expression, size_t& index);
double evaluateExpression(double x, const String& expression);
void revert_hex_data(const char* in, char* out, int l);
double round2(double value);
bool _hexToRaw(const char* in, byte* out, int rawSize);
bool _rawToHex(byte* in, char* out, int rawSize);
char* ip2CharArray(IPAddress ip);
bool to_bool(String const& s);
void pubMainCore(JsonObject& data);
void buildTopicFromId(JsonObject& Jsondata, const char* origin);
void emptyQueue();
void emptyQueue();
void pub(const char* topicori, const char* payload, bool retainFlag);
bool arrayContains(JsonArray array, const char* value);
bool filterMessage(JsonObject message);
void pub(const char* topicori, JsonObject& data);
void pub(const char* topicori, const char* payload);
void pub_custom_topic(const char* topic, JsonObject& data, boolean retain);
void pubMQTT(const char* topic, const char* payload);
void pubMQTT(const char* topic, const char* payload, bool retainFlag);
void pubMQTT(String topic, const char* payload);
void pubMQTT(const char* topic, unsigned long payload);
void pubMQTT(const char* topic, unsigned long long payload);
void pubMQTT(const char* topic, String payload);
void pubMQTT(String topic, String payload);
void pubMQTT(String topic, int payload);
void pubMQTT(String topic, unsigned long long payload);
void pubMQTT(String topic, float payload);
void pubMQTT(const char* topic, float payload);
void pubMQTT(const char* topic, int payload);
void pubMQTT(const char* topic, unsigned int payload);
void pubMQTT(const char* topic, long payload);
void pubMQTT(const char* topic, double payload);
void pubMQTT(String topic, unsigned long payload);
bool cmpToMainTopic(const char* topicOri, const char* toAdd);
void delayWithOTA(long waitMillis);
void SYSConfig_init();
void SYSConfig_fromJson(JsonObject& SYSdata);
void connectMQTT();
void setESPWifiProtocolTxPower();
void setESPWifiProtocolTxPower();
void setup();
bool wifi_reconnect_bypass();
void setOTA();
void setupTLS(bool self_signed, uint8_t index);
bool wifi_reconnect_bypass();
void ESPRestart(byte reason);
void ESPRestart(byte reason);
void setup_wifi();
void saveConfigCallback();
void blockingWaitForReset();
void checkButton();
void checkButton();
void saveConfig();
bool loadConfigFromFlash();
void setup_wifimanager(bool reset_settings);
void setup_ethernet_esp32();
void setup_ethernet();
void connectMQTTmdns();
void loop();
unsigned long uptime();
float intTemperatureRead();
void syncNTP();
int unixtimestamp();
String UTCtimestamp();
void eraseAndRestart();
String stateMeasures();
void storeSignalValue(SIGNAL_SIZE_UL_ULL MQTTvalue);
int getMin();
bool isAduplicateSignal(SIGNAL_SIZE_UL_ULL value);
void receivingMQTT(char* topicOri, char* datacallback);
bool checkForUpdates();
bool checkForUpdates();
void MQTTHttpsFWUpdate(char* topicOri, JsonObject& HttpsFwUpdateData);
bool SYSConfig_load();
void MQTTtoSYS(char* topicOri, JsonObject& SYSdata);
String toString(uint64_t input);
String toString(uint32_t input);
void setupFASTLED();
int animation_step(int duration, int steps);
int animation_step_count(int duration, int steps);
void FASTLEDLoop();
boolean FASTLEDtoMQTT();
void MQTTtoFASTLED(char* topicOri, JsonObject& jsonData);
void MQTTtoFASTLED(char* topicOri, char* datacallback);
void Fire2012WithPalette();
void ONOFFConfig_init();
void ONOFFConfig_fromJson(JsonObject& ONOFFdata);
void ONOFFConfig_load();
void ONOFFConfig_init();
void ONOFFConfig_fromJson(JsonObject& ONOFFdata);
void ONOFFConfig_load();
void updatePowerIndicator();
void setupONOFF();
void MQTTtoONOFF(char* topicOri, JsonObject& ONOFFdata);
void MQTTtoONOFF(char* topicOri, char* datacallback);
void overLimitTemp(void* pvParameters);
void overLimitCurrent(float RN8209current);
void overLimitCurrent(float RN8209current);
void ActuatorTrigger();
void stateONOFFMeasures();
void setupPWM();
static float perceptualToLinear(float perceptual, int channelIdx);
void PWMLoop();
boolean PWMtoMQTT();
void MQTTtoPWM(char* topicOri, JsonObject& jsonData);
void setupSomfy();
void MQTTtoSomfy(char* topicOri, JsonObject& jsonData);
void logToLCD(bool display);
void setBrightness(int brightness);
void setupM5();
void sleepScreen();
void wakeScreen(int brightness);
void loopM5();
void MQTTtoM5(char* topicOri, JsonObject& M5data);
void displayIntro(int i, int X, int Y);
void drawLogo(int logoSize, int circle1X, int circle1Y, bool circle1, bool circle2, bool circle3, bool line1, bool line2, bool name);
void M5Print(char* line1, char* line2, char* line3);
void initCC1101();
void setupCommonRF();
bool validFrequency(float mhz);
bool validReceiver(int receiver);
void disableCurrentReceiver();
void enableActiveReceiver();
String stateRFMeasures();
void RFConfig_fromJson(JsonObject& RFdata);
void RFConfig_init();
void RFConfig_load();
void MQTTtoRFset(char* topicOri, JsonObject& RFdata);
void RFConfig_init();
void RFConfig_load();
void MQTTtoRFset(char* topicOri, JsonObject& RFdata);
void enableActiveReceiver();
void logToOLED(bool display);
void setupSSD1306();
void loopSSD1306();
void MQTTtoSSD1306(char* topicOri, JsonObject& SSD1306data);
void SSD1306Config_save();
void SSD1306Config_init();
bool SSD1306Config_load();
void ssd1306Print(char* line1, char* line2, char* line3);
void ssd1306Print(char* line1, char* line2);
void ssd1306Print(char* line1);
String stateSSD1306Display();
void setup2G();
void setupGSM(bool deleteSMS);
void signalStrengthAnalysis();
bool _2GtoMQTT();
void MQTTto2G(char* topicOri, char* datacallback);
void MQTTto2G(char* topicOri, JsonObject& SMSdata);
void BTConfig_init();
String stateBTMeasures(bool start);
void BTConfig_load();
bool updateWorB(JsonObject& BTdata, bool isWhite);
void createOrUpdateDevice(const char* mac, uint8_t flags, int model, int mac_type);
void updateDevicesStatus();
void dumpDevices();
void strupp(char* beg);
void DT24Discovery(const char* mac, const char* sensorModel_id);
void BM2Discovery(const char* mac, const char* sensorModel_id);
void LYWSD03MMCDiscovery(const char* mac, const char* sensorModel);
void MHO_C401Discovery(const char* mac, const char* sensorModel);
void HHCCJCY01HHCCDiscovery(const char* mac, const char* sensorModel);
void XMWSDJ04MMCDiscovery(const char* mac, const char* sensorModel);
void LYWSD03MMCDiscovery(const char* mac, const char* sensorModel);
void MHO_C401Discovery(const char* mac, const char* sensorModel);
void HHCCJCY01HHCCDiscovery(const char* mac, const char* sensorModel);
void DT24Discovery(const char* mac, const char* sensorModel_id);
void BM2Discovery(const char* mac, const char* sensorModel_id);
void XMWSDJ04MMCDiscovery(const char* mac, const char* sensorModel_id);
bool checkIfIsTracker(char ch);
void procBLETask(void* pvParameters);
void BLEscan();
void BLEconnect();
void stopProcessing();
void coreTask(void* pvParameters);
void lowPowerESP32();
void deepSleep(uint64_t time_in_us);
void lowPowerESP32();
void changelowpowermode(int newLowPowerMode);
void setupBTTasksAndBLE();
void setupBT();
bool BTtoMQTT();
boolean valid_service_data(const char* data, int size);
void launchBTDiscovery(bool overrideDiscovery);
void process_bledata(JsonObject& BLEdata);
void hass_presence(JsonObject& HomePresence);
void BTforceScan();
void immediateBTAction(void* pvParameters);
void startBTActionTask();
void MQTTtoBTAction(JsonObject& BTdata);
void MQTTtoBT(char* topicOri, JsonObject& BTdata);
void GFSunInverterDataHandler(GfSun2000Data data);
void GFSunInverterErrorHandler(int errorId, char* errorMessage);
void setupGFSunInverter();
void ZgatewayGFSunInverterMQTT();
uint64_t getUInt64fromHex(char const* str);
void setupIR();
void IRtoMQTT();
void MQTTtoIR(char* topicOri, JsonObject& IRdata);
void dumpLORADevices();
void createOrUpdateDeviceLORA(const char* id, const char* model, uint8_t flags);
void launchLORADiscovery(bool overrideDiscovery);
void storeLORADiscovery(JsonObject& RFLORA_ESPdata, const char* model, const char* uniqueid);
uint8_t _determineDevice(byte* packet, int packetSize);
uint8_t _determineDevice(JsonObject& LORAdata);
boolean _WiPhoneToMQTT(byte* packet, JsonObject& LORAdata);
boolean _MQTTtoWiPhone(JsonObject& LORAdata);
void LORAConfig_init();
void LORAConfig_load();
byte hexStringToByte(const String& hexString);
void LORAConfig_fromJson(JsonObject& LORAdata);
void setupLORA();
void LORAtoMQTT();
void MQTTtoLORA(char* topicOri, JsonObject& LORAdata);
void MQTTtoLORA(char* topicOri, char* LORAarray);
String stateLORAMeasures();
void pilightCallback(const String& protocol, const String& message, int status,
                     size_t repeats, const String& deviceID);
void pilightRawCallback(const uint16_t* pulses, size_t length);
void savePilightConfig();
void loadPilightConfig();
void PilighttoMQTT();
void MQTTtoPilight(char* topicOri, JsonObject& Pilightdata);
extern void disablePilightReceive();
extern void enablePilightReceive();
static char* dec2binWzerofill(unsigned long Dec, unsigned int bitLength);
void RFtoMQTTdiscovery(SIGNAL_SIZE_UL_ULL MQTTvalue);
void RFtoMQTT();
void MQTTtoRF(char* topicOri, char* datacallback);
void MQTTtoRF(char* topicOri, JsonObject& RFdata);
void disableRFReceive();
void enableRFReceive();
void RF2toMQTTdiscovery(JsonObject& data);
void RF2toMQTT();
void rf2Callback(unsigned int period, unsigned long address, unsigned long groupBit, unsigned long unit, unsigned long switchType);
void MQTTtoRF2(char* topicOri, char* datacallback);
void MQTTtoRF2(char* topicOri, JsonObject& RF2data);
void disableRF2Receive();
void enableRF2Receive();
uint32_t gc_checksum();
void eeprom_setup();
void setupRFM69(void);
bool RFM69toMQTT(void);
void MQTTtoRFM69(char* topicOri, char* datacallback);
void MQTTtoRFM69(char* topicOri, JsonObject& RFM69data);
void setupRS232();
void RS232toMQTT();
void RS232toMQTT();
void sendMQTTfromNestedJson(JsonVariant obj, char* topic, int level, int maxLevel);
void MQTTtoRS232(char* topicOri, JsonObject& RS232data);
void dumpRTL_433Devices();
void createOrUpdateDeviceRTL_433(const char* id, const char* model, uint8_t flags);
void launchRTL_433Discovery(bool overrideDiscovery);
void storeRTL_433Discovery(JsonObject& RFrtl_433_ESPdata, const char* model, const char* uniqueid);
void rtl_433_Callback(char* message);
void setupRTL_433();
void RTL_433Loop();
extern void enableRTLreceive();
extern void disableRTLreceive();
extern int getRTLrssiThreshold();
extern int getRTLAverageRSSI();
extern int getRTLCurrentRSSI();
extern int getRTLMessageCount();
extern int getOOKThresh();
void setupSRFB();
void _rfbSend(byte* message);
void _rfbSend(byte* message, int times);
bool SRFBtoMQTT();
void _rfbDecode();
void _rfbAck();
void MQTTtoSRFB(char* topicOri, char* datacallback);
void MQTTtoSRFB(char* topicOri, JsonObject& SRFBdata);
void PairedDeviceAdded(byte newID);
void setupWeatherStation();
void sendWindSpeedData(byte id, float wind_speed, byte battery_status);
void sendRainData(byte id, float rain_volume, byte battery_status);
void sendWindData(byte id, int wind_direction, float wind_gust, byte battery_status);
void sendTemperatureData(byte id, float temperature, int humidity, byte battery_status);
void ZgatewayWeatherStationtoMQTT();
String getMacAddress();
String getUniqueId(String name, String sufix);
void createDiscoveryFromList(const char* mac,
                             const char* sensorList[][9],
                             int sensorCount,
                             const char* device_name,
                             const char* device_manufacturer,
                             const char* device_model);
void announceDeviceTrigger(bool use_gateway_info, char* topic, char* type, char* subtype, char* unique_id, char* device_name, char* device_manufacturer, char* device_model, char* device_id);
void createDiscovery(const char* sensor_type,
                     const char* st_topic, const char* s_name, const char* unique_id,
                     const char* availability_topic, const char* device_class, const char* value_template,
                     const char* payload_on, const char* payload_off, const char* unit_of_meas,
                     int off_delay,
                     const char* payload_available, const char* payload_not_available, bool gateway_entity, const char* cmd_topic,
                     const char* device_name, const char* device_manufacturer, const char* device_model, const char* device_id, bool retainCmd,
                     const char* state_class, const char* state_off, const char* state_on, const char* enum_options);
void eraseTopic(const char* sensor_type, const char* unique_id);
void btPresenceParametersDiscovery();
void btScanParametersDiscovery();
void pubMqttDiscovery();
void setupADC();
void MeasureADC();
void setupZsensorAHTx0();
void MeasureAHTTempHum();
void setupZsensorBH1750();
void MeasureLightIntensity();
void setupZsensorBME280();
void MeasureTempHumAndPressure();
void setupZsensorC37_YL83_HMRD();
void MeasureC37_YL83_HMRDWater();
void setupDHT();
void MeasureTempAndHum();
void setupZsensorDS1820();
void pubOneWire_HADiscovery();
void MeasureDS1820Temp();
void setupGPIOInput();
void MeasureGPIOInput();
void setupGPIOKeyCode();
void MeasureGPIOKeyCode();
void setupHCSR04();
void MeasureDistance();
void setupHCSR501();
void MeasureHCSR501();
void setupZsensorHTU21();
void MeasureTempHum();
void setupINA226();
void MeasureINA226();
static void writeRegister(byte reg, word value);
static word readRegister(byte reg);
void setupZsensorLM75();
void MeasureTemp();
void setupZsensorMQ2();
void MeasureGasMQ2();
void rn8209_loop(void* mode);
void setupRN8209();
void setupSHTC3();
void MeasureTempAndHum();
void setupZsensorTEMT6000();
void MeasureLightIntensityTEMT6000();
void displaySensorDetails(void);
void setupZsensorTSL2561();
void MeasureLightIntensityTSL2561();
void setupTouch();
void MeasureTouch();
size_t strchrspn(const char* str1, int character);
int WifiGetRssiAsQuality(int rssi);
char* GetTextIndexed(char* destination, size_t destination_size, uint32_t index, const char* haystack);
String HtmlEscape(const String unescaped);
bool NeedLogRefresh(uint32_t req_loglevel, uint32_t index);
bool GetLog(uint32_t req_loglevel, uint32_t* index_p, char** entry_pp, size_t* len_p);
String formatBytes(size_t bytes);
bool exists(String path);
void handleRoot();
void handleCN();
void handleWU();
void handleWI();
void handleMQ();
void handleCG();
void handleLO();
void handleLA();
bool isValidReceiver(int receiverId);
String generateActiveReceiverOptions(int currentSelection);
void handleRF();
void handleRT();
void handleCL();
void handleTK();
void handleIN();
void handleFavicon();
void handleUP();
void sendRestartPage();
void handleCS();
void notFound();
void WebUISetup();
void WebUILoop();
void MQTTtoWebUI(char* topicOri, JsonObject& WebUIdata);
String stateWebUIStatus();
bool WebUIConfig_save();
void WebUIConfig_init();
bool WebUIConfig_load();
void webUIPubPrint(const char* topicori, JsonObject& data);
void addLog(const uint8_t* buffer, size_t size);
#line 238 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
const char* getValueFromKeys(const JsonVariant& root, const String& keys) {
  return root[keys].as<const char*>();
}




double parseNumber(const String& expression, size_t& index) {
  String number;
  while (index < expression.length() && (isdigit(expression[index]) || expression[index] == '.')) {
    number += expression[index];
    index++;
  }
  return number.toDouble();
}

double evaluateFunction(double x, const String& functionName) {
  if (functionName == "sin") {
    return sin(x);
  } else if (functionName == "cos") {
    return cos(x);
  } else if (functionName == "tan") {
    return tan(x);
  } else if (functionName == "asin") {
    return asin(x);
  } else if (functionName == "acos") {
    return acos(x);
  } else if (functionName == "atan") {
    return atan(x);
  } else if (functionName == "sqrt") {
    return sqrt(x);
  }

  return 0.0;
}

double evaluatePrimary(double x, const String& expression, size_t& index) {
  if (expression[index] == '(') {
    index++;
    double result = evaluateExpression(x, expression, index);
    index++;
    return result;
  } else if (isdigit(expression[index]) || expression[index] == '.') {
    return parseNumber(expression, index);
  } else if (isalpha(expression[index])) {
    String functionName;
    while (index < expression.length() && isalpha(expression[index])) {
      functionName += expression[index];
      index++;
    }

    if (expression[index] == '(') {
      index++;
      double argument = evaluateExpression(x, expression, index);
      index++;
      return evaluateFunction(argument, functionName);
    } else {
      return evaluateFunction(x, functionName);
    }
  } else if (expression[index] == 'x') {
    index++;
    return x;
  } else {

    return 0.0;
  }
}

double evaluateFactor(double x, const String& expression, size_t& index) {
  double result = evaluatePrimary(x, expression, index);

  while (index < expression.length() && (expression[index] == '*' || expression[index] == '/')) {
    char op = expression[index];
    index++;

    double nextFactor = evaluatePrimary(x, expression, index);

    if (op == '*') {
      result *= nextFactor;
    } else if (op == '/') {
      if (nextFactor != 0) {
        result /= nextFactor;
      } else {

        return 0.0;
      }
    }
  }

  return result;
}

double evaluateExpression(double x, const String& expression, size_t& index) {
  double result = evaluateFactor(x, expression, index);

  while (index < expression.length() && (expression[index] == '+' || expression[index] == '-')) {
    char op = expression[index];
    index++;

    double nextTerm = evaluateFactor(x, expression, index);

    if (op == '+') {
      result += nextTerm;
    } else if (op == '-') {
      result -= nextTerm;
    }
  }

  return result;
}

double evaluateExpression(double x, const String& expression) {
  size_t index = 0;
  return evaluateExpression(x, expression, index);
}




void callback(char* topic, byte* payload, unsigned int length);

char mqtt_user[parameters_size] = MQTT_USER;
char mqtt_pass[parameters_size] = MQTT_PASS;
char mqtt_server[parameters_size] = MQTT_SERVER;
char mqtt_port[6] = MQTT_PORT;
char ota_pass[parameters_size] = gw_password;
#ifdef USE_MAC_AS_GATEWAY_NAME
#undef WifiManager_ssid
#undef ota_hostname
#define MAC_NAME_MAX_LEN 30
char WifiManager_ssid[MAC_NAME_MAX_LEN];
char ota_hostname[MAC_NAME_MAX_LEN];
#endif
bool connectedOnce = false;
bool connected = false;
int failure_number_ntwk = 0;
int failure_number_mqtt = 0;

unsigned long timer_led_measures = 0;
static void* eClient = nullptr;
static unsigned long last_ota_activity_millis = 0;
#if defined(ESP8266) || defined(ESP32)

SYSConfig_s SYSConfig;

bool failSafeMode = false;
static bool mqtt_secure = MQTT_SECURE_DEFAULT;
static bool mqtt_cert_validate = MQTT_CERT_VALIDATE_DEFAULT;
static uint8_t mqtt_ss_index = MQTT_SECURE_SELF_SIGNED_INDEX_DEFAULT;
static String mqtt_cert = "";
static String ota_server_cert = "";
#endif

#ifdef ESP32
# include <ArduinoOTA.h>
# include <FS.h>
# include <SPIFFS.h>
# include <esp_task_wdt.h>
# include <nvs.h>
# include <nvs_flash.h>

bool BTProcessLock = true;
bool ProcessLock = false;

# if !defined(NO_INT_TEMP_READING)

# include <stdio.h>

# include "rom/ets_sys.h"
# include "soc/rtc_cntl_reg.h"
# include "soc/sens_reg.h"
# endif
# ifdef ESP32_ETHERNET
# include <ETH.h>
void WiFiEvent(WiFiEvent_t event);
# endif

# include <WiFiClientSecure.h>
# include <WiFiMulti.h>
WiFiMulti wifiMulti;
# include <WiFiManager.h>
# ifdef MDNS_SD
# include <ESPmDNS.h>
# endif

#elif defined(ESP8266)
# include <ArduinoOTA.h>
# include <DNSServer.h>
# include <ESP8266WebServer.h>
# include <ESP8266WiFi.h>
# include <ESP8266WiFiMulti.h>
# include <FS.h>
# include <WiFiManager.h>
X509List caCert;
# if MQTT_SECURE_SELF_SIGNED_CLIENT
X509List* pClCert = nullptr;
PrivateKey* pClKey = nullptr;
# endif
ESP8266WiFiMulti wifiMulti;
# ifdef MDNS_SD
# include <ESP8266mDNS.h>
# endif

#else
# include <Ethernet.h>
#endif


PubSubClient client;

template <typename T>
void Config_update(JsonObject& data, const char* key, T& var);
template <typename T>
void Config_update(JsonObject& data, const char* key, T& var) {
  if (data.containsKey(key)) {
    if (var != data[key].as<T>()) {
      var = data[key].as<T>();
      Log.notice(F("Config %s changed: %T" CR), key, data[key].as<T>());
    } else {
      Log.notice(F("Config %s unchanged: %T" CR), key, data[key].as<T>());
    }
  }
}

void revert_hex_data(const char* in, char* out, int l) {

  int i = l - 2, j = 0;
  while (i != -2) {
    if (i % 2 == 0)
      out[j] = in[i + 1];
    else
      out[j] = in[i - 1];
    j++;
    i--;
  }
  out[l - 1] = '\0';
}





long value_from_hex_data(const char* service_data, int offset, int data_length, bool reverse, bool canBeNegative = true) {
  char data[data_length + 1];
  memcpy(data, &service_data[offset], data_length);
  data[data_length] = '\0';
  long value;
  if (reverse) {

    char rev_data[data_length + 1];
    revert_hex_data(data, rev_data, data_length + 1);
    value = strtol(rev_data, NULL, 16);
  } else {
    value = strtol(data, NULL, 16);
  }
  if (value > 65000 && data_length <= 4 && canBeNegative)
    value = value - 65535;
  Log.trace(F("value %D" CR), value);
  return value;
}





double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}




bool _hexToRaw(const char* in, byte* out, int rawSize) {
  if (strlen(in) != rawSize * 2)
    return false;
  char tmp[3] = {0};
  for (unsigned char p = 0; p < rawSize; p++) {
    memcpy(tmp, &in[p * 2], 2);
    out[p] = strtol(tmp, NULL, 16);
  }
  return true;
}




bool _rawToHex(byte* in, char* out, int rawSize) {
  for (unsigned char p = 0; p < rawSize; p++) {
    sprintf_P(&out[p * 2], PSTR("%02X" CR), in[p]);
  }
  return true;
}

char* ip2CharArray(IPAddress ip) {
  static char a[16];
  sprintf(a, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  return a;
}

bool to_bool(String const& s) {
  return s != "0";
}





void pubMainCore(JsonObject& data) {
  if (data.containsKey("origin")) {
    pub((char*)data["origin"].as<const char*>(), data);
#ifdef ZgatewayBT
    if (data.containsKey("distance")) {
      String topic = String(mqtt_topic) + BTConfig.presenceTopic + String(gateway_name);
      Log.trace(F("Pub HA Presence %s" CR), topic.c_str());
      pub_custom_topic((char*)topic.c_str(), data, false);
    }
#endif
  } else {
    Log.error(F("No origin in JSON filtered" CR));
  }
}


void enqueueJsonObject(const StaticJsonDocument<JSON_MSG_BUFFER>& jsonDoc) {
  if (jsonDoc.size() == 0) {
    Log.error(F("Empty JSON, skipping" CR));
    return;
  }
#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  if (queueLength >= QueueSize) {
    Log.warning(F("%d Doc(s) in queue, doc blocked" CR), queueLength);
    blockedMessages++;
    return;
  }
  Log.trace(F("Enqueue JSON" CR));
  JsonBundle bundle;
  bundle.doc = jsonDoc;
  jsonQueue.push(bundle);
  Log.trace(F("Queue length: %d" CR), jsonQueue.size());
#else

  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  pubMainCore(jsonObj);
#endif
}

#ifdef ESP32

bool handleJsonEnqueue(const StaticJsonDocument<JSON_MSG_BUFFER>& jsonDoc, int timeout) {
  if (xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(timeout))) {
    enqueueJsonObject(jsonDoc);
    xSemaphoreGive(xQueueMutex);
    return true;
  } else {
    Log.error(F("xQueueMutex not taken" CR));
    blockedMessages++;
    return false;
  }
}
#else
bool handleJsonEnqueue(const StaticJsonDocument<JSON_MSG_BUFFER>& jsonDoc, int timeout) {
  enqueueJsonObject(jsonDoc);
  return true;
}
#endif


bool handleJsonEnqueue(const StaticJsonDocument<JSON_MSG_BUFFER>& jsonDoc) {
  return handleJsonEnqueue(jsonDoc, QueueSemaphoreTimeOutLoop);
}

#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)





void buildTopicFromId(JsonObject& Jsondata, const char* origin) {
  if (!Jsondata.containsKey("id")) {
    Log.error(F("No id in Jsondata" CR));
    return;
  }

  std::string topic = Jsondata["id"].as<std::string>();


  size_t pos = topic.find(":");
  while (pos != std::string::npos) {
    topic.erase(pos, 1);
    pos = topic.find(":", pos);
  }
# ifdef ZgatewayBT
  if (BTConfig.pubBeaconUuidForTopic && !BTConfig.extDecoderEnable && Jsondata.containsKey("model_id") && Jsondata["model_id"].as<std::string>() == "IBEACON") {
    if (Jsondata.containsKey("uuid")) {
      topic = Jsondata["uuid"].as<std::string>();
    } else {
      Log.error(F("No uuid in Jsondata" CR));
    }
  }

  if (BTConfig.extDecoderEnable && !Jsondata.containsKey("model"))
    topic = BTConfig.extDecoderTopic.c_str();
# endif
  std::string subjectStr(origin);
  topic = subjectStr + "/" + topic;

  Jsondata["origin"] = topic;

  Log.trace(F("Origin: %s" CR), Jsondata["origin"].as<const char*>());
}


void emptyQueue() {
  queueLength = jsonQueue.size();
  if (queueLength > maxQueueLength) {
    maxQueueLength = queueLength;
  }
  if (queueLength == 0) {
    return;
  }
  JsonBundle bundle;
  Log.trace(F("Dequeue JSON" CR));
  bundle = jsonQueue.front();
  jsonQueue.pop();
  JsonObject obj = bundle.doc.as<JsonObject>();
  pubMainCore(obj);
  queueLengthSum++;
}
#else
void emptyQueue() {}
#endif
# 677 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
void pub(const char* topicori, const char* payload, bool retainFlag) {
  String topic = String(mqtt_topic) + String(gateway_name) + String(topicori);
  pubMQTT(topic.c_str(), payload, retainFlag);
}



bool arrayContains(JsonArray array, const char* value) {
  for (JsonVariant v : array) {
    if (strcmp(v.as<const char*>(), value) == 0) {
      return true;
    }
  }
  return false;
}

bool matchesPattern(const std::string& value, const std::string& pattern) {

  auto it = std::search(
      value.begin(), value.end(),
      pattern.begin(), pattern.end(),
      [](char ch1, char ch2) {
        return ch2 == '*' || ch1 == ch2;
      });

  return it != value.end();
}

bool filterMessage(JsonObject message) {
  JsonObject filters = jsonSYSCONFIGBuffer.as<JsonObject>();
  String filtersAsString = "";
  serializeJson(filters, filtersAsString);
  Log.trace(F("Filters %s" CR), filtersAsString.c_str());

  JsonObject whitelist = filters["whitelist"];
  for (JsonPair kv : whitelist) {
    const char* key = kv.key().c_str();
    if (!message.containsKey(key)) {
      return false;
    } else {
      bool foundMatch = false;
      for (JsonVariant value : whitelist[key].as<JsonArray>()) {
        if (matchesPattern(message[key].as<std::string>(), value.as<std::string>())) {
          foundMatch = true;
          break;
        }
      }
      if (!foundMatch) {
        return false;
      }
    }
  }

  JsonObject blacklist = filters["blacklist"];
  for (JsonPair kv : blacklist) {
    const char* key = kv.key().c_str();
    if (message.containsKey(key)) {
      for (JsonVariant value : blacklist[key].as<JsonArray>()) {
        if (matchesPattern(message[key].as<std::string>(), value.as<std::string>())) {
          return false;
        }
      }
    }
  }

  return true;
}







void pub(const char* topicori, JsonObject& data) {
  String dataAsString = "";
  serializeJson(data, dataAsString);
  if (!filterMessage(data)) {
    Log.notice(F("Message filtered %s" CR), dataAsString.c_str());
    return;
  }

  bool ret = sensor_Retain;

  if (true) {
    JsonObject sysConfig2 = jsonSYSCONFIGBuffer.as<JsonObject>();
    String sysConfigString;
    serializeJson(sysConfig2, sysConfigString);
    Log.warning(F("SysConfig for debugging %s" CR), sysConfigString.c_str());

    if (sysConfig2.containsKey("transformations")) {
# 782 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
      for (const JsonVariant& setting : sysConfig2["transformations"].as<JsonArray>()) {



        if (data[setting["identification_element"].as<const char*>()] == setting["identification_match"]) {
          int y = 0;
          for (const JsonVariant& dataKey : setting["dataExtractKeys"].as<JsonArray>()) {


            String var = getValueFromKeys(data, dataKey.as<String>());

            String formula;
            String mqttTopic;
            bool mattError = false;

            if (setting.containsKey("formula") && setting["formula"].is<JsonArray>() && setting.containsKey("mqttTopic") && setting["mqttTopic"].is<JsonArray>()) {
              JsonArray formulaArray = setting["formula"].as<JsonArray>();
              JsonArray mqttTopicArray = setting["mqttTopic"].as<JsonArray>();
              if (formulaArray.size() > y && mqttTopicArray.size() > y) {
                formula = formulaArray[y].as<String>();
                mqttTopic = mqttTopicArray[y].as<String>();
              } else {
                Serial.println("Not enough formula/MQTT elements");
                Log.warning(F("Not enough formula/MQTT elements" CR));
                mattError = true;
              }
            } else {
              Serial.println("Error or missing formula/mqttTopic from settings.json");
              Log.warning(F("Error or missing formula/mqttTopic from settings.json" CR));
              mattError = true;
            }


            if (!mattError) {
              if (formula == "1" || formula == "x") {
                const char* mqttTopicInput = mqttTopic.c_str();

                pubMQTT(mqttTopicInput, var.c_str(), ret);

              } else {
                double varDouble = var.toDouble();
                double result = evaluateExpression(varDouble, formula.c_str());
                String stringResult = String(result);
                const char* mqttTopicInput = mqttTopic.c_str();

                pubMQTT(mqttTopicInput, stringResult.c_str(), ret);
              }
            }
            y++;
          }
        }
      }

    } else {
      Serial.println("Failed to open settings file");
      Log.warning(F("Failed to open settings file" CR));
    }
  } else {
    Serial.println("Settings file not found");
    Log.warning(F("Settings file not found" CR));
  }

#if defined(ESP8266) || defined(ESP32)
# if message_UTCtimestamp == true
  data["UTCtime"] = UTCtimestamp();
# endif
# if message_unixtimestamp == true
  data["unixtime"] = unixtimestamp();
# endif
#endif
  if (data.containsKey("retain") && data["retain"].is<bool>()) {
    ret = data["retain"];
    data.remove("retain");
  }
  if (data.containsKey("origin")) {
    data.remove("origin");
  }
  if (data.size() == 0) {
    Log.error(F("Empty JSON, not published" CR));
    return;
  }
  serializeJson(data, dataAsString);
  Log.notice(F("Send on %s msg %s" CR), topicori, dataAsString.c_str());
  String topic = String(mqtt_topic) + String(gateway_name) + String(topicori);
#if valueAsATopic
# ifdef ZgatewayPilight
  String value = data["value"];
  String protocol = data["protocol"];
  if (value != "null" && value != 0) {
    topic = topic + "/" + protocol + "/" + value;
  }
# else
  SIGNAL_SIZE_UL_ULL value = data["value"];
  if (value != 0) {
    topic = topic + "/" + toString(value);
  }
# endif
#endif

#if jsonPublishing
  Log.trace(F("jsonPubl - ON" CR));
  pubMQTT(topic.c_str(), dataAsString.c_str(), ret);
  pubWebUI(topicori, data);
#endif

#if simplePublishing
  Log.trace(F("simplePub - ON" CR));

  for (JsonPair p : data) {
# if defined(ESP8266)
    yield();
# endif
    if (p.value().is<SIGNAL_SIZE_UL_ULL>() && strcmp(p.key().c_str(), "rssi") != 0) {
      if (strcmp(p.key().c_str(), "value") == 0) {
        pubMQTT(topic, p.value().as<SIGNAL_SIZE_UL_ULL>());
      } else {
        pubMQTT(topic + "/" + String(p.key().c_str()), p.value().as<SIGNAL_SIZE_UL_ULL>());
      }
    } else if (p.value().is<int>()) {
      pubMQTT(topic + "/" + String(p.key().c_str()), p.value().as<int>());
    } else if (p.value().is<float>()) {
      pubMQTT(topic + "/" + String(p.key().c_str()), p.value().as<float>());
    } else if (p.value().is<char*>()) {
      pubMQTT(topic + "/" + String(p.key().c_str()), p.value().as<const char*>());
    }
  }
#endif
}







void pub(const char* topicori, const char* payload) {
  String topic = String(mqtt_topic) + String(gateway_name) + String(topicori);
  pubMQTT(topic, payload);
}
# 929 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
void pub_custom_topic(const char* topic, JsonObject& data, boolean retain) {
  Serial.println("Here");
  Log.warning(F("SysConfig for debugging" CR));

  String buffer = "";
  serializeJson(data, buffer);
  pubMQTT(topic, buffer.c_str(), retain);







}







void pubMQTT(const char* topic, const char* payload) {
  pubMQTT(topic, payload, sensor_Retain);
}
# 962 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
void pubMQTT(const char* topic, const char* payload, bool retainFlag) {
  if (client.connected()) {
    SendReceiveIndicatorON();
    Log.trace(F("[ OMG->MQTT ] topic: %s msg: %s " CR), topic, payload);
    client.publish(topic, payload, retainFlag);
  } else {
    Log.warning(F("Client not connected, aborting the publication" CR));
  }
}

void pubMQTT(String topic, const char* payload) {
  pubMQTT(topic.c_str(), payload);
}

void pubMQTT(const char* topic, unsigned long payload) {
  char val[11];
  sprintf(val, "%lu", payload);
  pubMQTT(topic, val);
}

void pubMQTT(const char* topic, unsigned long long payload) {
  char val[21];
  sprintf(val, "%llu", payload);
  pubMQTT(topic, val);
}

void pubMQTT(const char* topic, String payload) {
  pubMQTT(topic, payload.c_str());
}

void pubMQTT(String topic, String payload) {
  pubMQTT(topic.c_str(), payload.c_str());
}

void pubMQTT(String topic, int payload) {
  char val[12];
  sprintf(val, "%d", payload);
  pubMQTT(topic.c_str(), val);
}

void pubMQTT(String topic, unsigned long long payload) {
  char val[21];
  sprintf(val, "%llu", payload);
  pubMQTT(topic.c_str(), val);
}

void pubMQTT(String topic, float payload) {
  char val[12];
  dtostrf(payload, 3, 1, val);
  pubMQTT(topic.c_str(), val);
}

void pubMQTT(const char* topic, float payload) {
  char val[12];
  dtostrf(payload, 3, 1, val);
  pubMQTT(topic, val);
}

void pubMQTT(const char* topic, int payload) {
  char val[12];
  sprintf(val, "%d", payload);
  pubMQTT(topic, val);
}

void pubMQTT(const char* topic, unsigned int payload) {
  char val[12];
  sprintf(val, "%u", payload);
  pubMQTT(topic, val);
}

void pubMQTT(const char* topic, long payload) {
  char val[11];
  sprintf(val, "%ld", payload);
  pubMQTT(topic, val);
}

void pubMQTT(const char* topic, double payload) {
  char val[16];
  sprintf(val, "%f", payload);
  pubMQTT(topic, val);
}

void pubMQTT(String topic, unsigned long payload) {
  char val[11];
  sprintf(val, "%lu", payload);
  pubMQTT(topic.c_str(), val);
}

bool cmpToMainTopic(const char* topicOri, const char* toAdd) {


  if (strncmp(topicOri, mqtt_topic, strlen(mqtt_topic)) != 0)
    return false;

  topicOri += strlen(mqtt_topic);

  if (strncmp(topicOri, gateway_name, strlen(gateway_name)) != 0)
    return false;
  topicOri += strlen(gateway_name);
  if (strncmp(topicOri, toAdd, strlen(toAdd)) != 0)
    return false;
  return true;
}

void delayWithOTA(long waitMillis) {
#if defined(ESP8266) || defined(ESP32)
  long waitStep = 100;
  for (long waitedMillis = 0; waitedMillis < waitMillis; waitedMillis += waitStep) {
# ifndef ESPWifiManualSetup
# if defined(ESP8266) || defined(ESP32)
    checkButton();
# endif
# endif
    ArduinoOTA.handle();
# if defined(ZwebUI) && defined(ESP32)
    WebUILoop();
# endif
# ifdef ESP32

# endif
    delay(waitStep);
  }
#else
  delay(waitMillis);
#endif
}

void SYSConfig_init() {
#ifdef ZmqttDiscovery
  SYSConfig.discovery = true;
  SYSConfig.ohdiscovery = OpenHABDiscovery;
#endif
#ifdef RGB_INDICATORS
  SYSConfig.rgbbrightness = DEFAULT_ADJ_BRIGHTNESS;
#endif
}

void SYSConfig_fromJson(JsonObject& SYSdata) {
#ifdef ZmqttDiscovery
  Config_update(SYSdata, "disc", SYSConfig.discovery);
  Config_update(SYSdata, "ohdisc", SYSConfig.ohdiscovery);
#endif
#ifdef RGB_INDICATORS
  Config_update(SYSdata, "rgbb", SYSConfig.rgbbrightness);
#endif
}
# 1139 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
void connectMQTT() {
#ifndef ESPWifiManualSetup
# if defined(ESP8266) || defined(ESP32)
  checkButton();
# endif
#endif

  Log.warning(F("MQTT connection..." CR));
#ifdef ESP32


#endif
  char topic[mqtt_topic_max_size];
  strcpy(topic, mqtt_topic);
  strcat(topic, gateway_name);
  strcat(topic, will_Topic);
  client.setBufferSize(mqtt_max_packet_size);
  client.setSocketTimeout(GeneralTimeOut - 1);
#if AWS_IOT
  if (client.connect(gateway_name, mqtt_user, mqtt_pass)) {
#else

  bool conn = false;

  if ((mqtt_user == "" && mqtt_pass == "") || ((mqtt_user[0] == '\0' && mqtt_pass[0] == '\0') || (strlen(mqtt_user) == 0 && strlen(mqtt_pass) == 0))) {
    conn = client.connect(gateway_name, NULL, NULL, topic, will_QoS, will_Retain, will_Message);
    Log.warning(F("NULL case" CR));
  } else {
    conn = client.connect(gateway_name, mqtt_user, mqtt_pass, topic, will_QoS, will_Retain, will_Message);
    Log.warning(F("default case" CR));
    Log.warning(F("mqttUser %s, mqttPass %s" CR), mqtt_user, mqtt_user);
  }

  if (conn) {
#endif

    displayPrint("MQTT connected");
    Log.notice(F("Connected to broker" CR));
    failure_number_mqtt = 0;

    pub(will_Topic, Gateway_AnnouncementMsg, will_Retain);

    char topic2[mqtt_topic_max_size];
    strcpy(topic2, mqtt_topic);
    strcat(topic2, gateway_name);
    strcat(topic2, subjectMQTTtoX);
    if (client.subscribe(topic2)) {
#ifdef ZgatewayRF
      client.subscribe(subjectMultiGTWRF);
#endif
#ifdef ZgatewayIR
      client.subscribe(subjectMultiGTWIR);
#endif
      Log.trace(F("Subscription OK to the subjects %s" CR), topic2);
    }
  } else {
    failure_number_mqtt++;
    Log.warning(F("failure_number_mqtt: %d" CR), failure_number_mqtt);
    Log.warning(F("failed, rc=%d" CR), client.state());
#if defined(ESP32)
    if (mqtt_secure)
      Log.warning(F("failed, ssl error code=%d" CR), ((WiFiClientSecure*)eClient)->lastError(nullptr, 0));
#elif defined(ESP8266)
    if (mqtt_secure)
      Log.warning(F("failed, ssl error code=%d" CR), ((WiFiClientSecure*)eClient)->getLastSSLError());
#endif
    ErrorIndicatorON();
    delayWithOTA(5000);
    ErrorIndicatorOFF();
    delayWithOTA(5000);
    if (failure_number_mqtt > maxRetryWatchDog) {
      unsigned long millis_since_last_ota;
      while (


          (last_ota_activity_millis != 0)

          && ((millis_since_last_ota = millis() - last_ota_activity_millis) < ota_timeout_millis)) {


        Log.warning(F("OTA might be still active (activity %d ms ago)" CR), millis_since_last_ota);
#if defined(ESP8266) || defined(ESP32)
        ArduinoOTA.handle();
#endif
        delay(100);
      }
      ESPRestart(1);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {



  Log.trace(F("Hey I got a callback %s" CR), topic);

  byte* p = (byte*)malloc(length + 1);

  memcpy(p, payload, length);

  p[length] = '\0';

  if ((strstr(topic, subjectMultiGTWKey) != NULL) ||
      (strstr(topic, subjectGTWSendKey) != NULL) ||
      (strstr(topic, subjectMQTTtoSYSupdate) != NULL))
    receivingMQTT(topic, (char*)p);


  free(p);
}

#if defined(ESP32) && (defined(WifiGMode) || defined(WifiPower))
void setESPWifiProtocolTxPower() {



# if WifiGMode == true
  if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G) != ESP_OK) {
    Log.error(F("Failed to change WifiMode." CR));
  }
# endif

# if WifiGMode == false
  if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N) != ESP_OK) {
    Log.error(F("Failed to change WifiMode." CR));
  }
# endif

  uint8_t getprotocol;
  esp_err_t err;
  err = esp_wifi_get_protocol(WIFI_IF_STA, &getprotocol);

  if (err != ESP_OK) {
    Log.notice(F("Could not get protocol!" CR));
  }
  if (getprotocol & WIFI_PROTOCOL_11N) {
    Log.notice(F("WiFi_Protocol_11n" CR));
  }
  if (getprotocol & WIFI_PROTOCOL_11G) {
    Log.notice(F("WiFi_Protocol_11g" CR));
  }
  if (getprotocol & WIFI_PROTOCOL_11B) {
    Log.notice(F("WiFi_Protocol_11b" CR));
  }

# ifdef WifiPower
  Log.notice(F("Requested WiFi power level: %i" CR), WifiPower);
  WiFi.setTxPower(WifiPower);
# endif
  Log.notice(F("Operating WiFi power level: %i" CR), WiFi.getTxPower());
}
#endif

#if defined(ESP8266) && (defined(WifiGMode) || defined(WifiPower))
void setESPWifiProtocolTxPower() {
# if WifiGMode == true
  if (!wifi_set_phy_mode(PHY_MODE_11G)) {
    Log.error(F("Failed to change WifiMode." CR));
  }
# endif

# if WifiGMode == false
  if (!wifi_set_phy_mode(PHY_MODE_11N)) {
    Log.error(F("Failed to change WifiMode." CR));
  }
# endif

  phy_mode_t getprotocol = wifi_get_phy_mode();
  if (getprotocol == PHY_MODE_11N) {
    Log.notice(F("WiFi_Protocol_11n" CR));
  }
  if (getprotocol == PHY_MODE_11G) {
    Log.notice(F("WiFi_Protocol_11g" CR));
  }
  if (getprotocol == PHY_MODE_11B) {
    Log.notice(F("WiFi_Protocol_11b" CR));
  }

# ifdef WifiPower
  Log.notice(F("Requested WiFi power level: %i dBm" CR), WifiPower);

  int i_dBm = int(WifiPower * 4.0f);


  if (i_dBm > 82) {
    i_dBm = 82;
  } else if (i_dBm < 0) {
    i_dBm = 0;
  }

  system_phy_set_max_tpw((uint8_t)i_dBm);
# endif
}
#endif

void setup() {

  Serial.begin(SERIAL_BAUD);
  Log.begin(LOG_LEVEL, &Serial);
  Log.notice(F(CR "************* WELCOME TO OpenMQTTGateway **************" CR));
#if defined(TRIGGER_GPIO) && !defined(ESPWifiManualSetup)
  pinMode(TRIGGER_GPIO, INPUT_PULLUP);
  checkButton();
#endif

  delay(1000);

  SYSConfig_init();
  SYSConfig_load();


  SetupIndicatorError();
  SetupIndicatorSendReceive();
  SetupIndicatorInfo();
  SetupIndicators();

#if defined(ESP8266) || defined(ESP32)
  SYSConfig_load();
# ifdef ESP8266
# ifndef ZgatewaySRFB
  Serial.end();
  Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_TX_ONLY);
# endif
# elif ESP32
  xQueueMutex = xSemaphoreCreateMutex();
# if DEFAULT_LOW_POWER_MODE != -1
  preferences.begin(Gateway_Short_Name, false);
  if (preferences.isKey("lowpowermode")) {
    lowpowermode = preferences.getUInt("lowpowermode", DEFAULT_LOW_POWER_MODE);
  } else {
    Log.notice(F("No lowpowermode config to load" CR));
  }
  preferences.end();
# endif
# if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5STACK) || defined(ZboardM5TOUGH)
  setupM5();
# endif
# if defined(ZdisplaySSD1306)
  setupSSD1306();
  modules.add(ZdisplaySSD1306);
# endif
# endif

  Log.notice(F("OpenMQTTGateway Version: " OMG_VERSION CR));





# if defined(DEEP_SLEEP_IN_US) || defined(ESP32_EXT0_WAKE_PIN)
  ready_to_sleep = false;
# endif

# ifdef DEEP_SLEEP_IN_US
# ifdef ESP8266
  Log.notice(F("Setting wake pin for deep sleep." CR));
  pinMode(ESP8266_DEEP_SLEEP_WAKE_PIN, WAKEUP_PULLUP);
# endif
# ifdef ESP32
  Log.notice(F("Setting duration for deep sleep." CR));
  if (esp_sleep_enable_timer_wakeup(DEEP_SLEEP_IN_US) != ESP_OK) {
    Log.error(F("Failed to set deep sleep duration." CR));
  }
# endif
# endif

# ifdef ESP32_EXT0_WAKE_PIN
  Log.notice(F("Setting EXT0 Wakeup for deep sleep." CR));
  if (esp_sleep_enable_ext0_wakeup(ESP32_EXT0_WAKE_PIN, ESP32_EXT0_WAKE_PIN_STATE) != ESP_OK) {
    Log.error(F("Failed to set deep sleep EXT0 Wakeup." CR));
  }
# endif
# ifdef ESP32

# endif




# ifdef ZsensorRN8209
  setupRN8209();
  modules.add(ZsensorRN8209);
# endif
# ifdef ZactuatorONOFF
  setupONOFF();
  modules.add(ZactuatorONOFF);
# endif

# if defined(ESPWifiManualSetup)
  setup_wifi();
# else
  if (loadConfigFromFlash()) {
    Log.notice(F("Config loaded from flash" CR));
# ifdef ESP32_ETHERNET
    setup_ethernet_esp32();
# endif
    if (!failSafeMode && !ethConnected) setup_wifimanager(false);
  } else {
# ifdef ESP32_ETHERNET
    setup_ethernet_esp32();
# endif
    Log.notice(F("No config in flash, launching wifi manager" CR));

    if (!failSafeMode) setup_wifimanager(false);
  }

# endif
  Log.trace(F("OpenMQTTGateway mac: %s" CR), WiFi.macAddress().c_str());
  Log.trace(F("OpenMQTTGateway ip: %s" CR), WiFi.localIP().toString().c_str());

  setOTA();
#else


  Serial.begin(SERIAL_BAUD);

  setup_ethernet();
#endif

#if defined(ZwebUI) && defined(ESP32)
  WebUISetup();
  modules.add(ZwebUI);
#endif

#if defined(ESP8266) || defined(ESP32)
  if (mqtt_secure) {
    eClient = new WiFiClientSecure;
    if (mqtt_cert_validate) {
      setupTLS(MQTT_SECURE_SELF_SIGNED, mqtt_ss_index);
    } else {
      WiFiClientSecure* sClient = (WiFiClientSecure*)eClient;
      sClient->setInsecure();
    }
  } else {
    eClient = new WiFiClient;
  }
#else
  eClient = new EthernetClient;
#endif
  client.setClient(*(Client*)eClient);

#if defined(MDNS_SD) && (defined(ESP8266) || defined(ESP32))
  Log.trace(F("Connecting to MQTT by mDNS without MQTT hostname" CR));
  connectMQTTmdns();
#else
  uint16_t port = strtol(mqtt_port, NULL, 10);
  Log.trace(F("Port: %l" CR), port);
  Log.trace(F("Mqtt server: %s" CR), mqtt_server);
  client.setServer(mqtt_server, port);
#endif

  client.setCallback(callback);

  delay(1500);
#if defined(ZgatewayRF) || defined(ZgatewayPilight) || defined(ZgatewayRTL_433) || defined(ZgatewayRF2) || defined(ZactuatorSomfy)
# ifndef ARDUINO_AVR_UNO
  setupCommonRF();
# endif
#endif
#ifdef ZsensorBME280
  setupZsensorBME280();
  modules.add(ZsensorBME280);
#endif
#ifdef ZsensorHTU21
  setupZsensorHTU21();
  modules.add(ZsensorHTU21);
#endif
#ifdef ZsensorLM75
  setupZsensorLM75();
  modules.add(ZsensorLM75);
#endif
#ifdef ZsensorAHTx0
  setupZsensorAHTx0();
  modules.add(ZsensorAHTx0);
#endif
#ifdef ZsensorBH1750
  setupZsensorBH1750();
  modules.add(ZsensorBH1750);
#endif
#ifdef ZsensorMQ2
  setupZsensorMQ2();
  modules.add(ZsensorMQ2);
#endif
#ifdef ZsensorTEMT6000
  setupZsensorTEMT6000();
  modules.add(ZsensorTEMT6000);
#endif
#ifdef ZsensorTSL2561
  setupZsensorTSL2561();
  modules.add(ZsensorTSL2561);
#endif
#ifdef Zgateway2G
  setup2G();
  modules.add(Zgateway2G);
#endif
#ifdef ZgatewayIR
  setupIR();
  modules.add(ZgatewayIR);
#endif
#ifdef ZgatewayLORA
  setupLORA();
  modules.add(ZgatewayLORA);
#endif
#ifdef ZgatewayRF
  modules.add(ZgatewayRF);
#define ACTIVE_RECEIVER ACTIVE_RF
#endif
#ifdef ZgatewayRF2
  modules.add(ZgatewayRF2);
# ifdef ACTIVE_RECEIVER
#undef ACTIVE_RECEIVER
# endif
#define ACTIVE_RECEIVER ACTIVE_RF2
#endif
#ifdef ZgatewayPilight
  modules.add(ZgatewayPilight);
# ifdef ACTIVE_RECEIVER
#undef ACTIVE_RECEIVER
# endif
#define ACTIVE_RECEIVER ACTIVE_PILIGHT
#endif
#ifdef ZgatewayWeatherStation
  setupWeatherStation();
  modules.add(ZgatewayWeatherStation);
#endif
#ifdef ZgatewayGFSunInverter
  setupGFSunInverter();
  modules.add(ZgatewayGFSunInverter);
#endif
#ifdef ZgatewaySRFB
  setupSRFB();
  modules.add(ZgatewaySRFB);
#endif
#ifdef ZgatewayBT
  setupBT();
  modules.add(ZgatewayBT);
#endif
#ifdef ZgatewayRFM69
  setupRFM69();
  modules.add(ZgatewayRFM69);
#endif
#ifdef ZsensorINA226
  setupINA226();
  modules.add(ZsensorINA226);
#endif
#ifdef ZsensorHCSR501
  setupHCSR501();
  modules.add(ZsensorHCSR501);
#endif
#ifdef ZsensorHCSR04
  setupHCSR04();
  modules.add(ZsensorHCSR04);
#endif
#ifdef ZsensorGPIOInput
  setupGPIOInput();
  modules.add(ZsensorGPIOInput);
#endif
#ifdef ZsensorGPIOKeyCode
  setupGPIOKeyCode();
  modules.add(ZsensorGPIOKeyCode);
#endif
#ifdef ZactuatorFASTLED
  setupFASTLED();
  modules.add(ZactuatorFASTLED);
#endif
#ifdef ZactuatorPWM
  setupPWM();
  modules.add(ZactuatorPWM);
#endif
#ifdef ZactuatorSomfy
# ifdef ACTIVE_RECEIVER
#undef ACTIVE_RECEIVER
# endif
#define ACTIVE_RECEIVER ACTIVE_NONE
  setupSomfy();
  modules.add(ZactuatorSomfy);
#endif
#ifdef ZsensorDS1820
  setupZsensorDS1820();
  modules.add(ZsensorDS1820);
#endif
#ifdef ZsensorADC
  setupADC();
  modules.add(ZsensorADC);
#endif
#ifdef ZsensorTouch
  setupTouch();
  modules.add(ZsensorTouch);
#endif
#ifdef ZsensorC37_YL83_HMRD
  setupZsensorC37_YL83_HMRD();
  modules.add(ZsensorC37_YL83_HMRD);
#endif
#ifdef ZsensorDHT
  setupDHT();
  modules.add(ZsensorDHT);
#endif
#ifdef ZgatewayRS232
  setupRS232();
  modules.add(ZgatewayRS232);
#endif
#ifdef ZsensorSHTC3
  setupSHTC3();
#endif
#ifdef ZgatewayRTL_433
# ifdef ACTIVE_RECEIVER
#undef ACTIVE_RECEIVER
# endif
#define ACTIVE_RECEIVER ACTIVE_RTL
  setupRTL_433();
  modules.add(ZgatewayRTL_433);
#endif
  Log.trace(F("mqtt_max_packet_size: %d" CR), mqtt_max_packet_size);

#ifndef ARDUINO_AVR_UNO
  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);
  Log.notice(F("OpenMQTTGateway modules: %s" CR), jsonChar);
#endif
  Log.notice(F("************** Setup OpenMQTTGateway end **************" CR));
}

#if defined(ESP8266) || defined(ESP32)

bool wifi_reconnect_bypass() {
  uint8_t wifi_autoreconnect_cnt = 0;
# ifdef ESP32
  while (WiFi.status() != WL_CONNECTED && wifi_autoreconnect_cnt < maxConnectionRetryNetwork) {
# else
  while (WiFi.waitForConnectResult() != WL_CONNECTED && wifi_autoreconnect_cnt < maxConnectionRetryNetwork) {
# endif
    Log.notice(F("Attempting Wifi connection with saved AP: %d" CR), wifi_autoreconnect_cnt);

    WiFi.begin();
# if (defined(ESP8266) || defined(ESP32)) && (defined(WifiGMode) || defined(WifiPower))
    setESPWifiProtocolTxPower();
# endif
    delay(1000);
    wifi_autoreconnect_cnt++;
  }
  if (wifi_autoreconnect_cnt < maxConnectionRetryNetwork) {
    return true;
  } else {
    return false;
  }
}

void setOTA() {

  ArduinoOTA.setPort(ota_port);


  ArduinoOTA.setHostname(ota_hostname);


  ArduinoOTA.setPassword(ota_pass);

  ArduinoOTA.onStart([]() {
    Log.trace(F("Start OTA, lock other functions" CR));
    ErrorIndicatorON();
    SendReceiveIndicatorON();
    last_ota_activity_millis = millis();
# ifdef ESP32
    ProcessLock = true;
# ifdef ZgatewayBT
    stopProcessing();
# endif
# endif
    lpDisplayPrint("OTA in progress");
  });
  ArduinoOTA.onEnd([]() {
    Log.trace(F("\nOTA done" CR));
    last_ota_activity_millis = 0;
    ErrorIndicatorOFF();
    SendReceiveIndicatorOFF();
    lpDisplayPrint("OTA done");
    ESPRestart(6);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Log.trace(F("Progress: %u%%\r" CR), (progress / (total / 100)));
# ifdef ESP32

# endif
    last_ota_activity_millis = millis();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    last_ota_activity_millis = millis();
    ErrorIndicatorOFF();
    SendReceiveIndicatorOFF();
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Log.error(F("Auth Failed" CR));
    else if (error == OTA_BEGIN_ERROR)
      Log.error(F("Begin Failed" CR));
    else if (error == OTA_CONNECT_ERROR)
      Log.error(F("Connect Failed" CR));
    else if (error == OTA_RECEIVE_ERROR)
      Log.error(F("Receive Failed" CR));
    else if (error == OTA_END_ERROR)
      Log.error(F("End Failed" CR));
    ESPRestart(6);
  });
  ArduinoOTA.begin();
}

void setupTLS(bool self_signed, uint8_t index) {
  configTime(0, 0, NTP_SERVER);
  WiFiClientSecure* sClient = (WiFiClientSecure*)eClient;
# if MQTT_SECURE_SELF_SIGNED
  if (self_signed) {
    Log.notice(F("Using self signed cert index %u" CR), index);
# if defined(ESP32)
    sClient->setCACert(certs_array[index].server_cert);
# if AWS_IOT
    if (strcmp(mqtt_port, "443") == 0) {
      sClient->setAlpnProtocols(alpnProtocols);
    }
# endif
# if MQTT_SECURE_SELF_SIGNED_CLIENT
    sClient->setCertificate(certs_array[index].client_cert);
    sClient->setPrivateKey(certs_array[index].client_key);
# endif
# elif defined(ESP8266)
    caCert.append(certs_array[index].server_cert);
    sClient->setTrustAnchors(&caCert);
    sClient->setBufferSizes(512, 512);
# if MQTT_SECURE_SELF_SIGNED_CLIENT
    if (pClCert != nullptr) {
      delete pClCert;
    }
    if (pClKey != nullptr) {
      delete pClKey;
    }
    pClCert = new X509List(certs_array[index].client_cert);
    pClKey = new PrivateKey(certs_array[index].client_key);
    sClient->setClientRSACert(pClCert, pClKey);
# endif
# endif
  } else
# endif
  {
    if (mqtt_cert.length() > 0) {
# if defined(ESP32)
      sClient->setCACert(mqtt_cert.c_str());
    } else {
      sClient->setCACert(certificate);
    }
# elif defined(ESP8266)
      caCert.append(mqtt_cert.c_str());
    } else {
      caCert.append(certificate);
    }
    sClient->setTrustAnchors(&caCert);
    sClient->setBufferSizes(512, 512);
# endif
  }
}
#else
bool wifi_reconnect_bypass() {
  return true;
}
#endif
# 1815 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/main.ino"
#if defined(ESP8266) || defined(ESP32)
void ESPRestart(byte reason) {
  delay(1000);
  StaticJsonDocument<128> jsonBuffer;
  JsonObject jsondata = jsonBuffer.to<JsonObject>();
  jsondata["reason"] = reason;
  jsondata["retain"] = true;
  jsondata["uptime"] = uptime();
  pub(subjectLOGtoMQTT, jsondata);
  Log.warning(F("Rebooting for reason code %d" CR), reason);
# if defined(ESP32)
  ESP.restart();
# elif defined(ESP8266)
  ESP.reset();
# endif
}
#else
void ESPRestart(byte reason) {}
#endif

#if defined(ESPWifiManualSetup)
void setup_wifi() {
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(wifi_ssid, wifi_password);
  Log.trace(F("Connecting to %s" CR), wifi_ssid);
# ifdef wifi_ssid1
  wifiMulti.addAP(wifi_ssid1, wifi_password1);
  Log.trace(F("Connecting to %s" CR), wifi_ssid1);
# endif
  delay(10);



# ifdef NetworkAdvancedSetup
  IPAddress ip_adress;
  IPAddress gateway_adress;
  IPAddress subnet_adress;
  IPAddress dns_adress;
  ip_adress.fromString(NET_IP);
  gateway_adress.fromString(NET_GW);
  subnet_adress.fromString(NET_MASK);
  dns_adress.fromString(NET_DNS);

  if (!WiFi.config(ip_adress, gateway_adress, subnet_adress, dns_adress)) {
    Log.error(F("Wifi STA Failed to configure" CR));
  }

# endif

  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Log.trace(F("." CR));
    failure_number_ntwk++;
# if defined(ESP32) && defined(ZgatewayBT)
    if (lowpowermode) {
      if (failure_number_ntwk > maxConnectionRetryNetwork) {
        lowPowerESP32();
      }
    } else {
      if (failure_number_ntwk > maxRetryWatchDog) {
        ESPRestart(2);
      }
    }
# else
    if (failure_number_ntwk > maxRetryWatchDog) {
      ESPRestart(2);
    }
# endif
  }
  Log.notice(F("WiFi ok with manual config credentials" CR));
  displayPrint("Wifi connected");
}

#elif defined(ESP8266) || defined(ESP32)

WiFiManager wifiManager;


bool shouldSaveConfig = false;



void saveConfigCallback() {
  Log.trace(F("Should save config" CR));
  shouldSaveConfig = true;
}

# ifdef TRIGGER_GPIO



void blockingWaitForReset() {
  if (digitalRead(TRIGGER_GPIO) == LOW) {
    delay(50);
    if (digitalRead(TRIGGER_GPIO) == LOW) {
      Log.trace(F("Trigger button Pressed" CR));
      delay(3000);
      if (digitalRead(TRIGGER_GPIO) == LOW) {
        Log.trace(F("Button Held" CR));

# ifdef ZactuatorONOFF
        uint8_t level = digitalRead(ACTUATOR_ONOFF_GPIO);
        if (level == ACTUATOR_ON) {
          ActuatorTrigger();
        }
# endif
# ifdef ESP32

# endif
        InfoIndicatorOFF();
        SendReceiveIndicatorOFF();


        if (SPIFFS.begin()) {
          Log.trace(F("mounted file system" CR));
          if (SPIFFS.exists("/config.json")) {
            Log.notice(F("Erasing ESP Config, restarting" CR));
            setup_wifimanager(true);
          }
        }
        delay(30000);
        if (digitalRead(TRIGGER_GPIO) == LOW) {
          Log.notice(F("Going into failsafe mode without peripherals" CR));

          failSafeMode = true;
          setup_wifimanager(false);
        }
      }
    }
  }
}




void checkButton() {
  unsigned long timeFromStart = millis();

  if (timeFromStart < TimeToResetAtStart) {
    blockingWaitForReset();
  } else {
# if defined(INPUT_GPIO) && defined(ZsensorGPIOInput) && INPUT_GPIO == TRIGGER_GPIO
    MeasureGPIOInput();
# else
    blockingWaitForReset();
# endif
  }
}
# else
void checkButton() {}
# endif

void saveConfig() {
  Log.trace(F("saving config" CR));
  DynamicJsonDocument json(512 + ota_server_cert.length() + mqtt_cert.length());
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_pass"] = mqtt_pass;
  json["mqtt_topic"] = mqtt_topic;
  json["gateway_name"] = gateway_name;
  json["mqtt_broker_secure"] = mqtt_secure;
  json["mqtt_broker_cert"] = mqtt_cert;
  json["mqtt_ss_index"] = mqtt_ss_index;
  json["ota_server_cert"] = ota_server_cert;
  json["ota_pass"] = ota_pass;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Log.error(F("failed to open config file for writing" CR));
  }

  serializeJson(json, configFile);
  configFile.close();
}

bool loadConfigFromFlash() {
  Log.trace(F("mounting FS..." CR));
  bool result = false;

  if (SPIFFS.begin()) {
    Log.trace(F("mounted file system" CR));
  } else {
    Log.warning(F("failed to mount FS -> formating" CR));
    SPIFFS.format();
    if (SPIFFS.begin())
      Log.trace(F("mounted file system after formating" CR));
  }
  if (SPIFFS.exists("/config.json")) {

    Log.trace(F("reading config file" CR));
    File configFile = SPIFFS.open("/config.json", "r");
    if (configFile) {
      Log.trace(F("opened config file" CR));
      DynamicJsonDocument json(configFile.size() * 2);
      auto error = deserializeJson(json, configFile);
      if (error) {
        Log.error(F("deserialize config failed: %s, buffer capacity: %u" CR), error.c_str(), json.capacity());
      }
      if (!json.isNull()) {
        Log.trace(F("\nparsed json, size: %u" CR), json.memoryUsage());
        if (json.containsKey("mqtt_server"))
          strcpy(mqtt_server, json["mqtt_server"]);
        if (json.containsKey("mqtt_port"))
          strcpy(mqtt_port, json["mqtt_port"]);
        if (json.containsKey("mqtt_user"))
          strcpy(mqtt_user, json["mqtt_user"]);
        if (json.containsKey("mqtt_pass"))
          strcpy(mqtt_pass, json["mqtt_pass"]);
        if (json.containsKey("mqtt_topic"))
          strcpy(mqtt_topic, json["mqtt_topic"]);
        if (json.containsKey("mqtt_broker_secure"))
          mqtt_secure = json["mqtt_broker_secure"].as<bool>();
        if (json.containsKey("mqtt_broker_cert"))
          mqtt_cert = json["mqtt_broker_cert"].as<const char*>();
        if (json.containsKey("mqtt_ss_index"))
          mqtt_ss_index = json["mqtt_ss_index"].as<uint8_t>();
        if (json.containsKey("gateway_name"))
          strcpy(gateway_name, json["gateway_name"]);
        if (json.containsKey("ota_pass")) {
          strcpy(ota_pass, json["ota_pass"]);
# ifdef WM_PWD_FROM_MAC


          if (strcmp(ota_pass, "OTAPASSWORD") == 0) {
            String s = WiFi.macAddress();
            sprintf(ota_pass, "%.2s%.2s%.2s%.2s",
                    s.c_str() + 6, s.c_str() + 9, s.c_str() + 12, s.c_str() + 15);
          }
# endif
        }
        if (json.containsKey("ota_server_cert"))
          ota_server_cert = json["ota_server_cert"].as<const char*>();
        result = true;
      } else {
        Log.warning(F("failed to load json config" CR));
      }
      configFile.close();
    }
  } else {
    Log.notice(F("no config file found defining default values" CR));
# ifdef USE_MAC_AS_GATEWAY_NAME
    String s = WiFi.macAddress();
    sprintf(gateway_name, "%.2s%.2s%.2s%.2s%.2s%.2s",
            s.c_str(), s.c_str() + 3, s.c_str() + 6, s.c_str() + 9, s.c_str() + 12, s.c_str() + 15);
# endif
# ifdef WM_PWD_FROM_MAC
    sprintf(ota_pass, "%.2s%.2s%.2s%.2s",
            s.c_str() + 6, s.c_str() + 9, s.c_str() + 12, s.c_str() + 15);
# endif
  }

  return result;
}

void setup_wifimanager(bool reset_settings) {
  delay(10);
  WiFi.mode(WIFI_STA);

  if (reset_settings)
    eraseAndRestart();

# ifdef USE_MAC_AS_GATEWAY_NAME
  String s = WiFi.macAddress();
  snprintf(WifiManager_ssid, MAC_NAME_MAX_LEN, "%s_%.2s%.2s", Gateway_Short_Name, s.c_str(), s.c_str() + 3);
  strcpy(ota_hostname, WifiManager_ssid);
  Log.notice(F("OTA Hostname: %s.local" CR), ota_hostname);
# endif

  wifiManager.setDebugOutput(WM_DEBUG);




# ifndef WIFIMNG_HIDE_MQTT_CONFIG
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, parameters_size, " minlength='1' maxlength='64' required");
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6, " minlength='1' maxlength='5' required");
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, parameters_size, " maxlength='64'");
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", MQTT_PASS, parameters_size, " input type='password' maxlength='64'");
  WiFiManagerParameter custom_mqtt_topic("topic", "mqtt base topic", mqtt_topic, mqtt_topic_max_size, " minlength='1' maxlength='64' required");
  WiFiManagerParameter custom_mqtt_secure("secure", "mqtt secure", "1", 2, mqtt_secure ? "type=\"checkbox\" checked" : "type=\"checkbox\"");
  WiFiManagerParameter custom_mqtt_cert("cert", "<br/>mqtt broker cert", mqtt_cert.c_str(), 4096);
  WiFiManagerParameter custom_gateway_name("name", "gateway name", gateway_name, parameters_size, " minlength='1' maxlength='64' required");
  WiFiManagerParameter custom_ota_pass("ota", "gateway password", ota_pass, parameters_size, " input type='password' minlength='8' maxlength='64' required");
# endif



  wifiManager.setConnectTimeout(WiFi_TimeOut);

  wifiManager.setConfigPortalTimeout(WifiManager_ConfigPortalTimeOut);


  wifiManager.setSaveConfigCallback(saveConfigCallback);


# ifdef NetworkAdvancedSetup
  Log.trace(F("Adv wifi cfg" CR));
  IPAddress ip_adress;
  IPAddress gateway_adress;
  IPAddress subnet_adress;
  IPAddress dns_adress;
  ip_adress.fromString(NET_IP);
  gateway_adress.fromString(NET_GW);
  subnet_adress.fromString(NET_MASK);
  dns_adress.fromString(NET_DNS);
  wifiManager.setSTAStaticIPConfig(ip_adress, gateway_adress, subnet_adress, dns_adress);
# endif

# ifndef WIFIMNG_HIDE_MQTT_CONFIG

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_secure);
  wifiManager.addParameter(&custom_mqtt_cert);
  wifiManager.addParameter(&custom_gateway_name);
  wifiManager.addParameter(&custom_mqtt_topic);
  wifiManager.addParameter(&custom_ota_pass);
# endif

  wifiManager.setMinimumSignalQuality(MinimumWifiSignalQuality);

  if (SPIFFS.begin()) {
    Log.trace(F("mounted file system" CR));



    if (SPIFFS.exists("/config.json")) {
      wifiManager.setEnableConfigPortal(false);
    }
  }

# ifdef ESP32_ETHERNET
  wifiManager.setBreakAfterConfig(true);
# endif

  if (!wifi_reconnect_bypass())
  {
# ifdef ESP32
    if (lowpowermode < 2) {
      displayPrint("Connect your phone to WIFI AP:", WifiManager_ssid, ota_pass);
    } else {
# ifdef ZgatewayBT
      lowPowerESP32();
# endif
    }
# endif

    InfoIndicatorON();
    ErrorIndicatorON();
    Log.notice(F("Connect your phone to WIFI AP: %s with PWD: %s" CR), WifiManager_ssid, ota_pass);




    if (!wifiManager.autoConnect(WifiManager_ssid, ota_pass)) {
      if (!ethConnected) {
        Log.warning(F("failed to connect and hit timeout" CR));
        delay(3000);

# ifdef ESP32

        esp_wifi_set_mode(WIFI_MODE_AP);
        esp_wifi_start();
        wifi_config_t conf;
        esp_wifi_get_config(WIFI_IF_AP, &conf);
        conf.ap.ssid_hidden = 1;
        esp_wifi_set_config(WIFI_IF_AP, &conf);
# endif


        ESPRestart(3);
      }
    }
    InfoIndicatorOFF();
    ErrorIndicatorOFF();
  }

  displayPrint("Wifi connected");

  if (shouldSaveConfig) {

# ifndef WIFIMNG_HIDE_MQTT_CONFIG
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_user, custom_mqtt_user.getValue());

    if (strcmp(custom_mqtt_pass.getValue(), MQTT_PASS) != 0) {

      strcpy(mqtt_pass, custom_mqtt_pass.getValue());
    }
    strcpy(mqtt_topic, custom_mqtt_topic.getValue());
    if (mqtt_topic[strlen(mqtt_topic) - 1] != '/' && strlen(mqtt_topic) < parameters_size) {
      strcat(mqtt_topic, "/");
    }

    strcpy(gateway_name, custom_gateway_name.getValue());
    strcpy(ota_pass, custom_ota_pass.getValue());
    mqtt_secure = *custom_mqtt_secure.getValue();

    int cert_len = strlen(custom_mqtt_cert.getValue());
    if (cert_len) {
      char* cert_in = (char*)custom_mqtt_cert.getValue();
      while (*cert_in == ' ' || *cert_in == '\t') {
        cert_in++;
      }

      char* cert_begin = cert_in;
      while (*cert_in != NULL) {
        if (*cert_in == ' ' && (strncmp((cert_in + 1), "CERTIFICATE", 11) != 0)) {
          *cert_in = '\n';
        }
        cert_in++;
      }

      mqtt_cert = cert_begin;
    }
# endif

    saveConfig();
  }
}
# ifdef ESP32_ETHERNET
void setup_ethernet_esp32() {
  bool ethBeginSuccess = false;
  WiFi.onEvent(WiFiEvent);
# ifdef NetworkAdvancedSetup
  IPAddress ip_adress;
  IPAddress gateway_adress;
  IPAddress subnet_adress;
  IPAddress dns_adress;
  ip.fromString(NET_IP);
  gateway.fromString(NET_GW);
  subnet.fromString(NET_MASK);
  Dns.fromString(NET_DNS);

  Log.trace(F("Adv eth cfg" CR));
  ETH.config(ip, gateway, subnet, Dns);
  ethBeginSuccess = ETH.begin();
# else
  Log.trace(F("Spl eth cfg" CR));
  ethBeginSuccess = ETH.begin();
# endif
  Log.trace(F("Connecting to Ethernet" CR));
  while (!ethConnected && failure_number_ntwk <= maxConnectionRetryNetwork) {
    delay(500);
    Log.trace(F("." CR));
    failure_number_ntwk++;
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Log.trace(F("Ethernet Started" CR));
      ETH.setHostname(gateway_name);
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Log.notice(F("Ethernet Connected" CR));
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Log.trace(F("OpenMQTTGateway MAC: %s" CR), ETH.macAddress().c_str());
      Log.trace(F("OpenMQTTGateway IP: %s" CR), ETH.localIP().toString().c_str());
      Log.trace(F("OpenMQTTGateway link speed: %d Mbps" CR), ETH.linkSpeed());
      ethConnected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Log.error(F("Ethernet Disconnected" CR));
      ethConnected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Log.error(F("Ethernet Stopped" CR));
      ethConnected = false;
      break;
    default:
      break;
  }
}
# endif
#else
void setup_ethernet() {
# ifdef NetworkAdvancedSetup
  IPAddress ip_adress;
  IPAddress gateway_adress;
  IPAddress subnet_adress;
  IPAddress dns_adress;
  ip.fromString(NET_IP);
  gateway.fromString(NET_GW);
  subnet.fromString(NET_MASK);
  Dns.fromString(NET_DNS);

  Log.trace(F("Adv eth cfg" CR));
  Ethernet.begin(mac, ip, Dns, gateway, subnet);
# else
  Log.trace(F("Spl eth cfg" CR));
  Ethernet.begin(mac, ip);
# endif
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Log.error(F("Ethernet shield was not found." CR));
  } else {
    Log.trace(F("ip: %s " CR), Ethernet.localIP());
  }
}
#endif

#if defined(MDNS_SD) && (defined(ESP8266) || defined(ESP32))
void connectMQTTmdns() {
  Log.trace(F("Browsing for MQTT service" CR));
  int n = MDNS.queryService("mqtt", "tcp");
  if (n == 0) {
    Log.error(F("no services found" CR));
  } else {
    Log.trace(F("%d service(s) found" CR), n);
    for (int i = 0; i < n; ++i) {
      Log.trace(F("Service %d %s found" CR), i, MDNS.hostname(i).c_str());
      Log.trace(F("IP %s Port %d" CR), MDNS.IP(i).toString().c_str(), MDNS.port(i));
    }
    if (n == 1) {
      Log.trace(F("One MQTT server found setting parameters" CR));
      client.setServer(MDNS.IP(0), int(MDNS.port(0)));
    } else {
      Log.error(F("Several MQTT servers found, please deactivate mDNS and set your default server" CR));
    }
  }
}
#endif

void loop() {
#ifndef ESPWifiManualSetup
# if defined(ESP8266) || defined(ESP32)
  checkButton();
# endif
#endif

#ifdef ESP32

#endif

  unsigned long now = millis();


  if (now > (timer_led_measures + (TimeLedON * 1000))) {
    timer_led_measures = millis();
    InfoIndicatorOFF();
    SendReceiveIndicatorOFF();
  }

#if defined(ESP8266) || defined(ESP32)
  if (ethConnected || WiFi.status() == WL_CONNECTED) {
    if (ethConnected && WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect();
    }
    ArduinoOTA.handle();
#else
  if ((Ethernet.hardwareStatus() != EthernetW5100 && Ethernet.linkStatus() == LinkON) || (Ethernet.hardwareStatus() == EthernetW5100)) {
#endif
    failure_number_ntwk = 0;
#if defined(ZwebUI) && defined(ESP32)
    WebUILoop();
#endif
    if (client.loop()) {
      InfoIndicatorON();
      failure_number_ntwk = 0;

      bool justReconnected = !connected;
#ifdef ZmqttDiscovery

      if (SYSConfig.discovery && (now > lastDiscovery + DiscoveryAutoOffTimer))
        SYSConfig.discovery = false;


      bool publishDiscovery = SYSConfig.discovery && (!connectedOnce || (discovery_republish_on_reconnect && justReconnected));
      if (publishDiscovery) {
        pubMqttDiscovery();
      }
#endif
      connectedOnce = true;
      connected = true;
#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      if (now > (timer_sys_measures + (TimeBetweenReadingSYS * 1000)) || !timer_sys_measures) {
        timer_sys_measures = millis();
        stateMeasures();
# ifdef ZgatewayBT
        stateBTMeasures(false);
# endif
# ifdef ZactuatorONOFF
        stateONOFFMeasures();
# endif
# ifdef ZdisplaySSD1306
        stateSSD1306Display();
# endif
# ifdef ZgatewayLORA
        stateLORAMeasures();
# endif
# if defined(ZgatewayRTL_433) || defined(ZgatewayPilight) || defined(ZgatewayRF) || defined(ZgatewayRF2) || defined(ZactuatorSomfy)
        stateRFMeasures();
# endif
# if defined(ZwebUI) && defined(ESP32)
        stateWebUIStatus();
# endif
      }
      if (now > (timer_sys_checks + (TimeBetweenCheckingSYS * 1000)) || !timer_sys_checks) {
# if defined(ESP8266) || defined(ESP32)
# if message_UTCtimestamp || message_unixtimestamp
        syncNTP();
# endif
# endif
        if (!timer_sys_checks) {
# if defined(ESP32) && defined(MQTT_HTTPS_FW_UPDATE)
          checkForUpdates();
# endif
# ifdef ZgatewayBT
          BTProcessLock = !BTConfig.enabled;
# endif
        }

        timer_sys_checks = millis();
      }
#endif
      emptyQueue();
#ifdef ZsensorBME280
      MeasureTempHumAndPressure();
#endif
#ifdef ZsensorHTU21
      MeasureTempHum();
#endif
#ifdef ZsensorLM75
      MeasureTemp();
#endif
#ifdef ZsensorAHTx0
      MeasureAHTTempHum();
#endif
#ifdef ZsensorHCSR04
      MeasureDistance();
#endif
#ifdef ZsensorBH1750
      MeasureLightIntensity();
#endif
#ifdef ZsensorMQ2
      MeasureGasMQ2();
#endif
#ifdef ZsensorTEMT6000
      MeasureLightIntensityTEMT6000();
#endif
#ifdef ZsensorTSL2561
      MeasureLightIntensityTSL2561();
#endif
#ifdef ZsensorC37_YL83_HMRD
      MeasureC37_YL83_HMRDWater();
#endif
#ifdef ZsensorDHT
      MeasureTempAndHum();
#endif
#ifdef ZsensorSHTC3
      MeasureTempAndHum();
#endif
#ifdef ZsensorDS1820
      MeasureDS1820Temp();
#endif
#ifdef ZsensorINA226
      MeasureINA226();
#endif
#ifdef ZsensorHCSR501
      MeasureHCSR501();
#endif
#ifdef ZsensorGPIOInput
      MeasureGPIOInput();
#endif
#ifdef ZsensorGPIOKeyCode
      MeasureGPIOKeyCode();
#endif
#ifdef ZsensorADC
      MeasureADC();
#endif
#ifdef ZsensorTouch
      MeasureTouch();
#endif
#ifdef ZgatewayLORA
      LORAtoMQTT();
# ifdef ZmqttDiscovery
      if (SYSConfig.discovery)
        launchLORADiscovery(publishDiscovery);
# endif
#endif
#ifdef ZgatewayRF
      RFtoMQTT();
#endif
#ifdef ZgatewayRF2
      RF2toMQTT();
#endif
#ifdef ZgatewayWeatherStation
      ZgatewayWeatherStationtoMQTT();
#endif
#ifdef ZgatewayGFSunInverter
      ZgatewayGFSunInverterMQTT();
#endif
#ifdef ZgatewayPilight
      PilighttoMQTT();
#endif
#ifdef ZgatewayBT
# ifdef ZmqttDiscovery
      if (SYSConfig.discovery)
        launchBTDiscovery(publishDiscovery);
# endif
#endif
#ifdef ZgatewaySRFB
      SRFBtoMQTT();
#endif
#ifdef ZgatewayIR
      IRtoMQTT();
#endif
#ifdef Zgateway2G
      if (_2GtoMQTT())
        Log.trace(F("2GtoMQTT OK" CR));
#endif
#ifdef ZgatewayRFM69
      if (RFM69toMQTT())
        Log.trace(F("RFM69toMQTT OK" CR));
#endif
#ifdef ZgatewayRS232
      RS232toMQTT();
#endif
#ifdef ZactuatorFASTLED
      FASTLEDLoop();
#endif
#ifdef ZactuatorPWM
      PWMLoop();
#endif
#ifdef ZgatewayRTL_433
      RTL_433Loop();
# ifdef ZmqttDiscovery
      if (SYSConfig.discovery)
        launchRTL_433Discovery(publishDiscovery);
# endif
#endif

    } else {

      connected = false;
      connectMQTT();
    }
  } else {
#ifdef ESP32

#endif
    connected = false;
    Log.warning(F("Network disconnected" CR));
    ErrorIndicatorON();
    delay(2000);
#ifdef ESP32

#endif
    ErrorIndicatorOFF();
    delay(2000);
    wifi_reconnect_bypass();
  }

#if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5STACK) || defined(ZboardM5TOUGH)
  loopM5();
#endif
#if defined(ZdisplaySSD1306)
  loopSSD1306();
#endif





#if defined(DEEP_SLEEP_IN_US) || defined(ESP32_EXT0_WAKE_PIN)
  if (ready_to_sleep) {
    delay(250);
# ifdef DEEP_SLEEP_IN_US
    Log.notice(F("Entering deep sleep for %l us." CR), DEEP_SLEEP_IN_US);
# endif
# ifdef ESP32_EXT0_WAKE_PIN
    Log.notice(F("Entering deep sleep, EXT0 Wakeup by pin : %l." CR), ESP32_EXT0_WAKE_PIN);
# endif
# ifdef ESP8266
    ESP.deepSleep(DEEP_SLEEP_IN_US);
# endif
# ifdef ESP32
    esp_deep_sleep_start();
# endif
  }
#endif
}




unsigned long uptime() {
  static unsigned long lastUptime = 0;
  static unsigned long uptimeAdd = 0;
  unsigned long uptime = millis() / 1000 + uptimeAdd;
  if (uptime < lastUptime) {
    uptime += 4294967;
    uptimeAdd += 4294967;
  }
  lastUptime = uptime;
  return uptime;
}




#if defined(ESP32) && !defined(NO_INT_TEMP_READING)
float intTemperatureRead() {
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3, SENS_FORCE_XPD_SAR_S);
  SET_PERI_REG_BITS(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, 10, SENS_TSENS_CLK_DIV_S);
  CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
  CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
  ets_delay_us(100);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
  ets_delay_us(5);
  float temp_f = (float)GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR3_REG, SENS_TSENS_OUT, SENS_TSENS_OUT_S);
  float temp_c = (temp_f - 32) / 1.8;
  return temp_c;
}
#endif

#if defined(ESP8266) || defined(ESP32)
void syncNTP() {
  configTime(0, 0, NTP_SERVER);
  time_t now = time(nullptr);
  uint8_t count = 0;
  Log.trace(F("Waiting for NTP time sync" CR));
  while ((now < 8 * 3600 * 2) && count++ < 60) {
    delay(500);
    now = time(nullptr);
  }

  if (count >= 60) {
    Log.error(F("Unable to update - invalid time" CR));
    return;
  }
}

int unixtimestamp() {
  return time(nullptr);
}

String UTCtimestamp() {
  time_t now;
  time(&now);
  char buffer[sizeof "yyyy-MM-ddThh:mm:ssZ"];
  strftime(buffer, sizeof buffer, "%FT%TZ", gmtime(&now));
  return buffer;
}




void eraseAndRestart() {
  Log.trace(F("Formatting requested, result: %d" CR), SPIFFS.format());

# if defined(ESP8266)
  WiFi.disconnect(true);
# ifndef ESPWifiManualSetup
  wifiManager.resetSettings();
# endif
  delay(5000);
  ESP.reset();
# else

  nvs_flash_erase();
  ESP.restart();
# endif
}

#endif

#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
String stateMeasures() {
  StaticJsonDocument<JSON_MSG_BUFFER> SYSdata;

  SYSdata["uptime"] = uptime();

  SYSdata["version"] = OMG_VERSION;
# ifdef RGB_INDICATORS
  SYSdata["rgbb"] = SYSConfig.rgbbrightness;
# endif
# ifdef ZmqttDiscovery
  SYSdata["disc"] = SYSConfig.discovery;
  SYSdata["ohdisc"] = SYSConfig.ohdiscovery;
# endif
# if defined(ESP8266) || defined(ESP32)
  SYSdata["env"] = ENV_NAME;
  uint32_t freeMem;
  uint32_t minFreeMem;
  freeMem = ESP.getFreeHeap();
# ifdef ZgatewayRTL_433

  if (freeMem < MinimumMemory) {
    Log.error(F("Not enough memory %d, restarting" CR), freeMem);
    ESPRestart(8);
  }
# endif
  SYSdata["freemem"] = freeMem;
  SYSdata["mqttp"] = mqtt_port;
  SYSdata["mqtts"] = mqtt_secure;
  SYSdata["msgprc"] = queueLengthSum;
  SYSdata["msgblck"] = blockedMessages;
  SYSdata["maxq"] = maxQueueLength;
# ifdef ESP32
  minFreeMem = ESP.getMinFreeHeap();
  SYSdata["minmem"] = minFreeMem;
# ifndef NO_INT_TEMP_READING
  SYSdata["tempc"] = round2(intTemperatureRead());
# endif
  SYSdata["freestck"] = uxTaskGetStackHighWaterMark(NULL);
# endif

  SYSdata["eth"] = ethConnected;
  if (ethConnected) {
# ifdef ESP32_ETHERNET
    SYSdata["mac"] = (char*)ETH.macAddress().c_str();
    SYSdata["ip"] = ip2CharArray(ETH.localIP());
    ETH.fullDuplex() ? SYSdata["fd"] = (bool)"true" : SYSdata["fd"] = (bool)"false";
    SYSdata["linkspeed"] = (int)ETH.linkSpeed();
# endif
  } else {
    SYSdata["rssi"] = (long)WiFi.RSSI();
    SYSdata["SSID"] = (char*)WiFi.SSID().c_str();
    SYSdata["BSSID"] = (char*)WiFi.BSSIDstr().c_str();
    SYSdata["ip"] = ip2CharArray(WiFi.localIP());
    SYSdata["mac"] = (char*)WiFi.macAddress().c_str();
  }

# endif
# ifdef ZgatewayBT
# ifdef ESP32
  SYSdata["lowpowermode"] = (int)lowpowermode;
# endif
# endif
# ifdef ZboardM5STACK
  M5.Power.begin();
  SYSdata["m5battlevel"] = (int8_t)M5.Power.getBatteryLevel();
  SYSdata["m5ischarging"] = (bool)M5.Power.isCharging();
  SYSdata["m5ischargefull"] = (bool)M5.Power.isChargeFull();
# endif
# if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5TOUGH)
  M5.Axp.EnableCoulombcounter();
  SYSdata["m5batvoltage"] = (float)M5.Axp.GetBatVoltage();
  SYSdata["m5batcurrent"] = (float)M5.Axp.GetBatCurrent();
  SYSdata["m5vinvoltage"] = (float)M5.Axp.GetVinVoltage();
  SYSdata["m5vincurrent"] = (float)M5.Axp.GetVinCurrent();
  SYSdata["m5vbusvoltage"] = (float)M5.Axp.GetVBusVoltage();
  SYSdata["m5vbuscurrent"] = (float)M5.Axp.GetVBusCurrent();
  SYSdata["m5tempaxp"] = (float)M5.Axp.GetTempInAXP192();
  SYSdata["m5batpower"] = (float)M5.Axp.GetBatPower();
  SYSdata["m5batchargecurrent"] = (float)M5.Axp.GetBatChargeCurrent();
  SYSdata["m5apsvoltage"] = (float)M5.Axp.GetAPSVoltage();
# endif
  SYSdata["modules"] = modules;

  SYSdata["origin"] = subjectSYStoMQTT;
  handleJsonEnqueue(SYSdata);
  pubOled(subjectSYStoMQTT, SYSdata);

  char jsonChar[100];
  serializeJson(modules, jsonChar, 99);

  String _modules = jsonChar;

  _modules.replace(",", ", ");
  _modules.replace("[", "");
  _modules.replace("]", "");
  _modules.replace("\"", "'");

  SYSdata["modules"] = _modules.c_str();

  String output;
  serializeJson(SYSdata, output);
  return output;
}
#endif

#if defined(ZgatewayRF) || defined(ZgatewayIR) || defined(ZgatewaySRFB) || defined(ZgatewayWeatherStation) || defined(ZgatewayRTL_433)



void storeSignalValue(SIGNAL_SIZE_UL_ULL MQTTvalue) {
  unsigned long now = millis();

  int o = getMin();
  Log.trace(F("Min ind: %d" CR), o);

  receivedSignal[o].value = MQTTvalue;
  receivedSignal[o].time = now;


  Log.trace(F("store code : %u / %u" CR), (unsigned long)receivedSignal[o].value, receivedSignal[o].time);
  Log.trace(F("Col: val/timestamp" CR));
  for (int i = 0; i < struct_size; i++) {
    Log.trace(F("mem code : %u / %u" CR), (unsigned long)receivedSignal[i].value, receivedSignal[i].time);
  }
}




int getMin() {
  unsigned int minimum = receivedSignal[0].time;
  int minindex = 0;
  for (int i = 1; i < struct_size; i++) {
    if (receivedSignal[i].time < minimum) {
      minimum = receivedSignal[i].time;
      minindex = i;
    }
  }
  return minindex;
}




bool isAduplicateSignal(SIGNAL_SIZE_UL_ULL value) {
  Log.trace(F("isAdupl?" CR));
  for (int i = 0; i < struct_size; i++) {
    if (receivedSignal[i].value == value) {
      unsigned long now = millis();
      if (now - receivedSignal[i].time < time_avoid_duplicate) {
        Log.trace(F("no pub. dupl" CR));
        return true;
      }
    }
  }
  return false;
}
#endif

void receivingMQTT(char* topicOri, char* datacallback) {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject jsondata = jsonBuffer.to<JsonObject>();
  auto error = deserializeJson(jsonBuffer, datacallback);
  if (error) {
    Log.error(F("deserialize MQTT data failed: %s" CR), error.c_str());
    return;
  }

#if defined(ZgatewayRF) || defined(ZgatewayIR) || defined(ZgatewaySRFB) || defined(ZgatewayWeatherStation)
  if (strstr(topicOri, subjectMultiGTWKey) != NULL) {
    SIGNAL_SIZE_UL_ULL data = jsondata.isNull() ? STRTO_UL_ULL(datacallback, NULL, 10) : jsondata["value"];
    if (data != 0 && !isAduplicateSignal(data)) {
      storeSignalValue(data);
    }
  }
#endif

  if (!jsondata.isNull()) {

    String buffer = "";
    serializeJson(jsondata, buffer);
    Log.notice(F("[ MQTT->OMG ]: %s" CR), buffer.c_str());

#ifdef ZgatewayPilight
    MQTTtoPilight(topicOri, jsondata);
#endif
#if defined(ZgatewayRTL_433) || defined(ZgatewayPilight) || defined(ZgatewayRF) || defined(ZgatewayRF2) || defined(ZactuatorSomfy)
    MQTTtoRFset(topicOri, jsondata);
#endif
#if jsonReceiving
# ifdef ZgatewayLORA
    MQTTtoLORA(topicOri, jsondata);
# endif
# ifdef ZgatewayRF
    MQTTtoRF(topicOri, jsondata);
# endif
# ifdef ZgatewayRF2
    MQTTtoRF2(topicOri, jsondata);
# endif
# ifdef Zgateway2G
    MQTTto2G(topicOri, jsondata);
# endif
# ifdef ZgatewaySRFB
    MQTTtoSRFB(topicOri, jsondata);
# endif
# ifdef ZgatewayIR
    MQTTtoIR(topicOri, jsondata);
# endif
# ifdef ZgatewayRFM69
    MQTTtoRFM69(topicOri, jsondata);
# endif
# ifdef ZgatewayBT
    MQTTtoBT(topicOri, jsondata);
# endif
# ifdef ZactuatorFASTLED
    MQTTtoFASTLED(topicOri, jsondata);
# endif
# ifdef ZactuatorPWM
    MQTTtoPWM(topicOri, jsondata);
# endif
# if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5STACK) || defined(ZboardM5TOUGH)
    MQTTtoM5(topicOri, jsondata);
# endif
# if defined(ZdisplaySSD1306)
    MQTTtoSSD1306(topicOri, jsondata);
# endif
# ifdef ZactuatorONOFF
    MQTTtoONOFF(topicOri, jsondata);
# endif
# ifdef ZactuatorSomfy
    MQTTtoSomfy(topicOri, jsondata);
# endif
# ifdef ZgatewayRS232
    MQTTtoRS232(topicOri, jsondata);
# endif
# ifdef MQTT_HTTPS_FW_UPDATE
    MQTTHttpsFWUpdate(topicOri, jsondata);
# endif
# if defined(ZwebUI) && defined(ESP32)
    MQTTtoWebUI(topicOri, jsondata);
# endif
#endif
    SendReceiveIndicatorON();

    MQTTtoSYS(topicOri, jsondata);
  } else {
#if simpleReceiving
# ifdef ZgatewayLORA
    MQTTtoLORA(topicOri, datacallback);
# endif
# ifdef ZgatewayRF
    MQTTtoRF(topicOri, datacallback);
# endif
# ifdef ZgatewayRF315
    MQTTtoRF315(topicOri, datacallback);
# endif
# ifdef ZgatewayRF2
    MQTTtoRF2(topicOri, datacallback);
# endif
# ifdef Zgateway2G
    MQTTto2G(topicOri, datacallback);
# endif
# ifdef ZgatewaySRFB
    MQTTtoSRFB(topicOri, datacallback);
# endif
# ifdef ZgatewayRFM69
    MQTTtoRFM69(topicOri, datacallback);
# endif
# ifdef ZactuatorFASTLED
    MQTTtoFASTLED(topicOri, datacallback);
# endif
#endif
#ifdef ZactuatorONOFF
    MQTTtoONOFF(topicOri, datacallback);
#endif
  }
}

#ifdef MQTT_HTTPS_FW_UPDATE
String latestVersion;
# ifdef ESP32
# include <HTTPClient.h>

# include "zzHTTPUpdate.h"

# if CHECK_OTA_UPDATE





bool checkForUpdates() {
  Log.notice(F("Update check, free heap: %d"), ESP.getFreeHeap());
  HTTPClient http;
  http.setTimeout((GeneralTimeOut - 1) * 1000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(OTA_JSON_URL, OTAserver_cert);
  int httpCode = http.GET();
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject jsondata = jsonBuffer.to<JsonObject>();

  if (httpCode > 0) {
    String payload = http.getString();
    auto error = deserializeJson(jsonBuffer, payload);
    if (error) {
      Log.error(F("Deserialize MQTT data failed: %s" CR), error.c_str());
    }
    Log.trace(F("HttpCode %d" CR), httpCode);
    Log.trace(F("Payload %s" CR), payload.c_str());
  } else {
    Log.error(F("Error on HTTP request"));
  }
  http.end();
  Log.notice(F("Update check done, free heap: %d"), ESP.getFreeHeap());
  if (jsondata.containsKey("latest_version")) {
    jsondata["installed_version"] = OMG_VERSION;
    jsondata["entity_picture"] = ENTITY_PICTURE;
    if (!jsondata.containsKey("release_summary"))
      jsondata["release_summary"] = "";
    latestVersion = jsondata["latest_version"].as<String>();
    jsondata["origin"] = subjectRLStoMQTT;
    jsondata["retain"] = true;
    handleJsonEnqueue(jsondata);

    Log.trace(F("Update file found on server" CR));
    return true;
  } else {
    Log.trace(F("No update file found on server" CR));
    return false;
  }
}

# else
bool checkForUpdates() {
  return false;
}
# endif
# elif ESP8266
# include <ESP8266httpUpdate.h>
# endif

void MQTTHttpsFWUpdate(char* topicOri, JsonObject& HttpsFwUpdateData) {
  if (strstr(topicOri, subjectMQTTtoSYSupdate) != NULL) {
    const char* version = HttpsFwUpdateData["version"] | "latest";
    if (version && ((strlen(version) != strlen(OMG_VERSION)) || strcmp(version, OMG_VERSION) != 0)) {
      const char* url = HttpsFwUpdateData["url"];
      String systemUrl;
      if (url) {
        if (!strstr((url + (strlen(url) - 5)), ".bin")) {
          Log.error(F("Invalid firmware extension" CR));
          return;
        }
# if MQTT_HTTPS_FW_UPDATE_USE_PASSWORD > 0
        const char* pwd = HttpsFwUpdateData["password"];
        if (pwd) {
          if (strcmp(pwd, ota_pass) != 0) {
            Log.error(F("Invalid OTA password" CR));
            return;
          }
        } else {
          Log.error(F("No password sent" CR));
          return;
        }
# endif
# ifdef ESP32
      } else if (strcmp(version, "latest") == 0) {
        systemUrl = RELEASE_LINK + latestVersion + "/" + ENV_NAME + "-firmware.bin";
        url = systemUrl.c_str();
        Log.notice(F("Using system OTA url with latest version %s" CR), url);
      } else if (strcmp(version, "dev") == 0) {
        systemUrl = String(RELEASE_LINK_DEV) + ENV_NAME + "-firmware.bin";
        url = systemUrl.c_str();
        Log.notice(F("Using system OTA url with dev version %s" CR), url);
      } else if (version[0] == 'v') {
        systemUrl = String(RELEASE_LINK) + version + "/" + ENV_NAME + "-firmware.bin";
        url = systemUrl.c_str();
        Log.notice(F("Using system OTA url with defined version %s" CR), url);
# endif
      } else {
        Log.error(F("Invalid URL" CR));
        return;
      }
# ifdef ESP32
      ProcessLock = true;

# ifdef ZgatewayBT
      stopProcessing();
# endif
# endif
      Log.warning(F("Starting firmware update" CR));

      SendReceiveIndicatorON();
      ErrorIndicatorON();
      StaticJsonDocument<JSON_MSG_BUFFER> jsondata;
      jsondata["release_summary"] = "Update in progress ...";
      jsondata["origin"] = subjectRLStoMQTT;
      handleJsonEnqueue(jsondata);

      const char* ota_cert = HttpsFwUpdateData["server_cert"];
      if (!ota_cert && !strstr(url, "http:")) {
        if (ota_server_cert.length() > 0) {
          Log.notice(F("Using stored cert" CR));
          ota_cert = ota_server_cert.c_str();
        } else {
          Log.notice(F("Using config cert" CR));
          ota_cert = OTAserver_cert;
        }
      }

      t_httpUpdate_return result = HTTP_UPDATE_FAILED;
      if (strstr(url, "http:")) {
        Log.notice(F("Http update" CR));
        WiFiClient update_client;
# ifdef ESP32
        httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        result = httpUpdate.update(update_client, url);
# elif ESP8266
        ESPhttpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        result = ESPhttpUpdate.update(update_client, url);
# endif

      } else {
        WiFiClientSecure update_client;
        if (mqtt_secure) {
          client.disconnect();
          update_client = *(WiFiClientSecure*)eClient;
        } else {
          syncNTP();
        }

# ifdef ESP32
        update_client.setCACert(ota_cert);
        update_client.setTimeout(12);
        httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        httpUpdate.rebootOnUpdate(false);
        result = httpUpdate.update(update_client, url);
# elif ESP8266
        caCert.append(ota_cert);
        update_client.setTrustAnchors(&caCert);
        update_client.setTimeout(12000);
        ESPhttpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        ESPhttpUpdate.rebootOnUpdate(false);
        result = ESPhttpUpdate.update(update_client, url);
# endif
      }

      switch (result) {
        case HTTP_UPDATE_FAILED:
# ifdef ESP32
          Log.error(F("HTTP_UPDATE_FAILED Error (%d): %s\n" CR), httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
# elif ESP8266
          Log.error(F("HTTP_UPDATE_FAILED Error (%d): %s\n" CR), ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
# endif
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Log.notice(F("HTTP_UPDATE_NO_UPDATES" CR));
          break;

        case HTTP_UPDATE_OK:
          Log.notice(F("HTTP_UPDATE_OK" CR));
          jsondata["release_summary"] = "Update success !";
          jsondata["installed_version"] = latestVersion;
          jsondata["origin"] = subjectRLStoMQTT;
          handleJsonEnqueue(jsondata);
          ota_server_cert = ota_cert;
# ifndef ESPWifiManualSetup
          saveConfig();
# endif
          ESPRestart(6);
          break;
      }

      SendReceiveIndicatorOFF();
      ErrorIndicatorOFF();

      ESPRestart(6);
    }
  }
}
#endif



#if defined(ESP8266) || defined(ESP32)
bool SYSConfig_load() {
  Log.warning(F("Gateway_ShortName %s" CR), Gateway_Short_Name);
  preferences.begin(Gateway_Short_Name, true);
  if (preferences.isKey("SYSConfig")) {
    auto error = deserializeJson(jsonSYSCONFIGBuffer, preferences.getString("SYSConfig", "{}"));
    preferences.end();
    if (error) {
      Log.error(F("SYS config deserialization failed: %s, buffer capacity: %u" CR), error.c_str(), jsonSYSCONFIGBuffer.capacity());
      return false;
    }
    if (jsonSYSCONFIGBuffer.isNull()) {
      Log.warning(F("SYS config is null" CR));
      return false;
    }
    Log.notice(F("Saved SYS config loaded" CR));
    return true;
  } else {
    preferences.end();
    Log.notice(F("No SYS config to load" CR));
    return false;
  }
}

void SYSConfig_save(std::string jsonSYSConfigString) {
  preferences.begin(Gateway_Short_Name, false);
  int result = preferences.putString("SYSConfig", jsonSYSConfigString.c_str());
  preferences.end();
  Log.notice(F("SYS Config_save: %s, result: %d" CR), jsonSYSConfigString.c_str(), result);
}
#endif

void MQTTtoSYS(char* topicOri, JsonObject& SYSdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoSYSset)) {
    bool restartESP = false;
    Log.trace(F("MQTTtoSYS json" CR));
#if defined(ESP8266) || defined(ESP32)
    if (SYSdata.containsKey("cmd")) {
      const char* cmd = SYSdata["cmd"];
      Log.notice(F("Command: %s" CR), cmd);
      if (strstr(cmd, restartCmd) != NULL) {
        ESPRestart(5);
      } else if (strstr(cmd, eraseCmd) != NULL) {
# ifndef ESPWifiManualSetup
        setup_wifimanager(true);
# endif
      } else if (strstr(cmd, statusCmd) != NULL) {
        stateMeasures();
      }
    }

    if ((SYSdata.containsKey("whitelist") || SYSdata.containsKey("blacklist") || SYSdata.containsKey("transformations"))) {
      std::string SYSfilter;
      serializeJson(SYSdata, SYSfilter);
      SYSConfig_save(SYSfilter);
      SYSConfig_load();
    }

# ifdef RGB_INDICATORS
    if (SYSdata.containsKey("rgbb") && SYSdata["rgbb"].is<float>()) {
      if (SYSdata["rgbb"] >= 0 && SYSdata["rgbb"] <= 255) {
        SYSConfig.rgbbrightness = round2(SYSdata["rgbb"]);
        leds.setBrightness(SYSConfig.rgbbrightness);
        leds.show();
# ifdef ZactuatorONOFF
        updatePowerIndicator();
# endif
        Log.notice(F("RGB brightness: %d" CR), SYSConfig.rgbbrightness);
        stateMeasures();
      } else {
        Log.error(F("RGB brightness value invalid - ignoring command" CR));
      }
    }
# endif
# ifdef ZmqttDiscovery
    if (SYSdata.containsKey("ohdisc") && SYSdata["ohdisc"].is<bool>()) {
      SYSConfig.ohdiscovery = SYSdata["ohdisc"];
      Log.notice(F("OpenHAB discovery: %T" CR), SYSConfig.ohdiscovery);
      stateMeasures();
    }
# endif
    if (SYSdata.containsKey("wifi_ssid") && SYSdata.containsKey("wifi_pass")) {
# ifdef ESP32
      ProcessLock = true;
# ifdef ZgatewayBT
      stopProcessing();
# endif
# endif
      String prev_ssid = WiFi.SSID();
      String prev_pass = WiFi.psk();
      client.disconnect();
      WiFi.disconnect(true);

      Log.warning(F("Attempting connection to new AP %s" CR), (const char*)SYSdata["wifi_ssid"]);
      WiFi.begin((const char*)SYSdata["wifi_ssid"], (const char*)SYSdata["wifi_pass"]);
# if (defined(ESP8266) || defined(ESP32)) && (defined(WifiGMode) || defined(WifiPower))
      setESPWifiProtocolTxPower();
# endif
      WiFi.waitForConnectResult(WiFi_TimeOut * 1000);

      if (WiFi.status() != WL_CONNECTED) {
        Log.error(F("Failed to connect to new AP; falling back" CR));
        WiFi.disconnect(true);
        WiFi.begin(prev_ssid.c_str(), prev_pass.c_str());
# if (defined(ESP8266) || defined(ESP32)) && (defined(WifiGMode) || defined(WifiPower))
        setESPWifiProtocolTxPower();
# endif
      }
      restartESP = true;
    }

    bool disconnectClient = false;

    if (SYSdata.containsKey("mqtt_topic") || SYSdata.containsKey("gateway_name") || SYSdata.containsKey("gw_pass")) {
      if (SYSdata.containsKey("mqtt_topic")) {
        strncpy(mqtt_topic, SYSdata["mqtt_topic"], parameters_size);
      }
      if (SYSdata.containsKey("gateway_name")) {
        strncpy(gateway_name, SYSdata["gateway_name"], parameters_size);
      }
      if (SYSdata.containsKey("gw_pass")) {
        strncpy(ota_pass, SYSdata["gw_pass"], parameters_size);
        restartESP = true;
      }
# ifndef ESPWifiManualSetup
      saveConfig();
# endif
      disconnectClient = true;
    }

# ifdef MQTTsetMQTT
    if (SYSdata.containsKey("mqtt_user") && SYSdata.containsKey("mqtt_pass")) {
      bool update_server = false;
      bool secure_connect = SYSdata["mqtt_secure"].as<bool>();
      void* prev_client = nullptr;
      bool use_ss_cert = SYSdata.containsKey("mqtt_cert_index");
      uint8_t cert_index = mqtt_ss_index;

      if (SYSdata.containsKey("mqtt_server") && SYSdata.containsKey("mqtt_port")) {
        if (!SYSdata.containsKey("mqtt_secure")) {
          Log.error(F("mqtt_server provided without mqtt_secure defined - ignoring command" CR));
          return;
        }
# if MQTT_SECURE_SELF_SIGNED
        if (use_ss_cert) {
          cert_index = SYSdata["mqtt_cert_index"].as<uint8_t>();
          if (cert_index >= sizeof(certs_array) / sizeof(ss_certs)) {
            Log.error(F("mqtt_cert_index invalid - ignoring command" CR));
            return;
          }
        }
# endif

# ifdef ESP32
        ProcessLock = true;
# ifdef ZgatewayBT
        stopProcessing();
# endif
# endif
        disconnectClient = false;
        client.disconnect();
        update_server = true;
        if (secure_connect != mqtt_secure) {
          prev_client = eClient;
          if (!mqtt_secure) {
            eClient = new WiFiClientSecure;
          } else {
            Log.warning(F("Switching to unsecure MQTT broker" CR));
            eClient = new WiFiClient;
          }

          client.setClient(*(Client*)eClient);
        }

        if (secure_connect) {
          setupTLS(use_ss_cert, cert_index);
        }

        client.setServer(SYSdata["mqtt_server"].as<const char*>(), SYSdata["mqtt_port"].as<unsigned int>());
      } else {
# ifdef ESP32
        ProcessLock = true;
# ifdef ZgatewayBT
        stopProcessing();
# endif
# endif
        disconnectClient = false;
        client.disconnect();
      }

      String prev_user = mqtt_user;
      String prev_pass = mqtt_pass;
      strcpy(mqtt_user, SYSdata["mqtt_user"]);
      strcpy(mqtt_pass, SYSdata["mqtt_pass"]);

      connectMQTT();

      if (client.connected()) {
        if (update_server) {
          strcpy(mqtt_server, SYSdata["mqtt_server"]);
          strcpy(mqtt_port, SYSdata["mqtt_port"]);
          mqtt_ss_index = cert_index;
          if (prev_client != nullptr) {
            mqtt_secure = !mqtt_secure;
            delete prev_client;
          }
        }
# ifndef ESPWifiManualSetup
        saveConfig();
# endif
      } else {
        if (update_server) {
          if (prev_client != nullptr) {
            delete eClient;
            eClient = prev_client;
            client.setClient(*(Client*)eClient);
          }
          uint16_t port = strtol(mqtt_port, NULL, 10);
          client.setServer(mqtt_server, port);
        }
        strcpy(mqtt_user, prev_user.c_str());
        strcpy(mqtt_pass, prev_pass.c_str());
        if (mqtt_secure) {
          setupTLS(MQTT_SECURE_SELF_SIGNED, mqtt_ss_index);
        }
        connectMQTT();
      }
      restartESP = true;
    }
# endif

    if (disconnectClient) {
      client.disconnect();
    }
#endif

#ifdef ZmqttDiscovery
    if (SYSdata.containsKey("disc")) {
      if (SYSdata["disc"].is<bool>()) {
        if (SYSdata["disc"] == true && SYSConfig.discovery == false)
          lastDiscovery = millis();
        SYSConfig.discovery = SYSdata["disc"];
        stateMeasures();
        if (SYSConfig.discovery)
          pubMqttDiscovery();
      } else {
        Log.error(F("Discovery command not a boolean" CR));
      }
      Log.notice(F("Discovery state: %T" CR), SYSConfig.discovery);
    }
# ifdef ESP32
    if (SYSdata.containsKey("save") && SYSdata["save"].as<bool>()) {
      StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
      JsonObject jo = jsonBuffer.to<JsonObject>();
      jo["disc"] = SYSConfig.discovery;
      jo["ohdisc"] = SYSConfig.ohdiscovery;
# ifdef RGB_INDICATORS
      jo["rgbb"] = SYSConfig.rgbbrightness;
# endif

      String conf = "";
      serializeJson(jsonBuffer, conf);
      preferences.begin(Gateway_Short_Name, false);
      int result = preferences.putString("SYSConfig", conf);
      preferences.end();
      Log.notice(F("SYS Config_save: %s, result: %d" CR), conf.c_str(), result);
    }
# endif
#endif
    if (restartESP) {
      ESPRestart(7);
    }
  }
}

#if valueAsATopic && !defined(ZgatewayPilight)
# if defined(ESP32) || defined(ESP8266) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
String toString(uint64_t input) {
  String result = "";
  uint8_t base = 10;

  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

# else

String toString(uint32_t input) {
  String result = String(input);

  return result;
}
# endif
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorFASTLED.ino"
# 23 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorFASTLED.ino"
#include "User_config.h"

#ifdef ZactuatorFASTLED

# include <FastLED.h>

enum LEDState {
  OFF,
  FIRE,
  GENERAL
};
LEDState currentLEDState;
long lastUpdate = 0;
long currentUpdate = 0;
CRGB leds[FASTLED_NUM_LEDS];
CRGB ledColorBlink[FASTLED_NUM_LEDS];
bool blinkLED[FASTLED_NUM_LEDS];
const long blinkInterval = 300;
const long fireUpdate = 10;
CRGBPalette16 gPal;

void setupFASTLED() {
  Log.notice(F("FASTLED_DATA_GPIO: %d" CR), FASTLED_DATA_GPIO);
  Log.notice(F("FASTLED_NUM_LEDS: %d" CR), FASTLED_NUM_LEDS);
  Log.trace(F("ZactuatorFASTLED setup done " CR));
  FastLED.addLeds<FASTLED_TYPE, FASTLED_DATA_GPIO>(leds, FASTLED_NUM_LEDS);
}


int animation_step(int duration, int steps) {
  int currentStep = ((currentUpdate % duration) / ((float)duration)) * steps;
  return currentStep;
}


int animation_step_count(int duration, int steps) {
  long lastAnimationNumber = lastUpdate / duration;
  long currentAnimationNumber = currentUpdate / duration;
  int lastStep = ((lastUpdate % duration) / ((float)duration)) * steps;
  int currentStep = ((currentUpdate % duration) / ((float)duration)) * steps;

  return currentStep - lastStep + (currentAnimationNumber - lastAnimationNumber) * steps;
}

void FASTLEDLoop() {
  lastUpdate = currentUpdate;
  currentUpdate = millis();

  if (currentLEDState == GENERAL) {
    for (int i = 0; i < FASTLED_NUM_LEDS; i++) {
      int count = animation_step_count(blinkInterval, 2);
      int step = animation_step(blinkInterval, 2);

      if (count > 0) {
        if (blinkLED[i]) {
          if (step == 0) {
            leds[i] = ledColorBlink[i];
          } else {
            ledColorBlink[i] = leds[i];
            leds[i] = CRGB::Black;
          }
        }
      }
    }
  } else if (currentLEDState == FIRE) {
    int count = animation_step_count(fireUpdate, 1);
    if (count > 0) {

      Fire2012WithPalette();
    }
  }
  FastLED.show();
}

boolean FASTLEDtoMQTT() {
  return false;
}
# if jsonReceiving
void MQTTtoFASTLED(char* topicOri, JsonObject& jsonData) {
  currentLEDState = GENERAL;



  if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLEDsetled)) {
    Log.trace(F("MQTTtoFASTLED JSON analysis" CR));
    int ledNr = jsonData["led"];
    Log.notice(F("Led numero: %d" CR), ledNr);
    const char* color = jsonData["hex"];
    Log.notice(F("Color hex: %s" CR), color);

    long number = (long)strtol(color, NULL, 16);
    bool blink = jsonData["blink"];
    if (ledNr <= FASTLED_NUM_LEDS) {
      Log.notice(F("Blink: %d" CR), blink);
      blinkLED[ledNr] = blink;
      leds[ledNr] = number;
    }
  }
}
# endif

# if simpleReceiving
void MQTTtoFASTLED(char* topicOri, char* datacallback) {
  Log.trace(F("MQTTtoFASTLED: " CR));
  currentLEDState = GENERAL;
  long number = 0;
  if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLED)) {
    number = (long)strtol(&datacallback[1], NULL, 16);
    Log.notice(F("Number: %l" CR), number);
    for (int i = 0; i < FASTLED_NUM_LEDS; i++) {
      leds[i] = number;
    }
    FastLED.show();
  } else if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLEDsetbrightness)) {
    number = (long)strtol(&datacallback[1], NULL, 16);
    Log.notice(F("Number: %l" CR), number);
    FastLED.setBrightness(number);
    FastLED.show();
  } else if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLEDsetanimation)) {
    String payload = datacallback;
    Log.notice(F("Datacallback: %s" CR), datacallback);
    if (strstr(datacallback, "fire") != NULL) {
      currentLEDState = FIRE;
      gPal = HeatColors_p;
    } else {
      currentLEDState = OFF;
    }
  } else {
    currentLEDState = OFF;
  }
  if (currentLEDState == OFF) {
    for (int i = 0; i < FASTLED_NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
  }
}
# endif
# 191 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorFASTLED.ino"
#define COOLING 55




#define SPARKING 120
bool gReverseDirection = false;

void Fire2012WithPalette() {

  static byte heat[FASTLED_NUM_LEDS];


  for (int i = 0; i < FASTLED_NUM_LEDS; i++) {
    heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / FASTLED_NUM_LEDS) + 2));
  }


  for (int k = FASTLED_NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }


  if (random8() < SPARKING) {
    int y = random8(7);
    heat[y] = qadd8(heat[y], random8(160, 255));
  }


  for (int j = 0; j < FASTLED_NUM_LEDS; j++) {


    byte colorindex = scale8(heat[j], 240);
    CRGB color = ColorFromPalette(gPal, colorindex);
    int pixelnumber;
    if (gReverseDirection) {
      pixelnumber = (FASTLED_NUM_LEDS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorONOFF.ino"
# 29 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorONOFF.ino"
#include "User_config.h"

#ifdef ZactuatorONOFF

# ifdef ESP32

ONOFFConfig_s ONOFFConfig;

void ONOFFConfig_init() {
  ONOFFConfig.ONOFFState = !ACTUATOR_ON;
  ONOFFConfig.useLastStateOnStart = USE_LAST_STATE_ON_RESTART;
}

void ONOFFConfig_fromJson(JsonObject& ONOFFdata) {
  Config_update(ONOFFdata, "uselaststate", ONOFFConfig.useLastStateOnStart);
  Config_update(ONOFFdata, "cmd", ONOFFConfig.ONOFFState);

  if (ONOFFdata.containsKey("erase") && ONOFFdata["erase"].as<bool>()) {

    preferences.begin(Gateway_Short_Name, false);
    if (preferences.isKey("ONOFFConfig")) {
      int result = preferences.remove("ONOFFConfig");
      Log.notice(F("ONOFF config erase result: %d" CR), result);
      preferences.end();
      return;
    } else {
      Log.notice(F("ONOFF config not found" CR));
      preferences.end();
    }
  }
  if (ONOFFdata.containsKey("save") && ONOFFdata["save"].as<bool>()) {
    StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject jo = jsonBuffer.to<JsonObject>();
    jo["uselaststate"] = ONOFFConfig.useLastStateOnStart;
    jo["cmd"] = ONOFFConfig.ONOFFState;

    String conf = "";
    serializeJson(jsonBuffer, conf);
    preferences.begin(Gateway_Short_Name, false);
    int result = preferences.putString("ONOFFConfig", conf);
    preferences.end();
    Log.notice(F("ONOFF Config_save: %s, result: %d" CR), conf.c_str(), result);
  }
}

void ONOFFConfig_load() {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  preferences.begin(Gateway_Short_Name, true);
  if (preferences.isKey("ONOFFConfig")) {
    auto error = deserializeJson(jsonBuffer, preferences.getString("ONOFFConfig", "{}"));
    preferences.end();
    if (error) {
      Log.error(F("ONOFF config deserialization failed: %s, buffer capacity: %u" CR), error.c_str(), jsonBuffer.capacity());
      return;
    }
    if (jsonBuffer.isNull()) {
      Log.warning(F("ONOFF config is null" CR));
      return;
    }
    JsonObject jo = jsonBuffer.as<JsonObject>();
    ONOFFConfig_fromJson(jo);
    Log.notice(F("ONOFF config loaded" CR));
  } else {
    preferences.end();
    Log.notice(F("ONOFF config not found" CR));
  }
}
# else
void ONOFFConfig_init(){};
void ONOFFConfig_fromJson(JsonObject& ONOFFdata){};
void ONOFFConfig_load(){};
# endif

void updatePowerIndicator() {
  if (digitalRead(ACTUATOR_ONOFF_GPIO) == ACTUATOR_ON) {
    PowerIndicatorON();
  } else {
    PowerIndicatorOFF();
  }
}

void setupONOFF() {
# ifdef MAX_TEMP_ACTUATOR
  xTaskCreate(overLimitTemp, "overLimitTemp", 4000, NULL, 10, NULL);
# endif
# ifdef ESP32
  ONOFFConfig_init();
  ONOFFConfig_load();
  Log.notice(F("Target state on restart: %T" CR), ONOFFConfig.ONOFFState);
  Log.notice(F("Use last state on restart: %T" CR), ONOFFConfig.useLastStateOnStart);
# endif
  pinMode(ACTUATOR_ONOFF_GPIO, OUTPUT);
# ifdef ACTUATOR_ONOFF_DEFAULT
  digitalWrite(ACTUATOR_ONOFF_GPIO, ACTUATOR_ONOFF_DEFAULT);
# elif defined(ESP32)
  if (ONOFFConfig.useLastStateOnStart) {
    digitalWrite(ACTUATOR_ONOFF_GPIO, ONOFFConfig.ONOFFState);
  } else {
    digitalWrite(ACTUATOR_ONOFF_GPIO, !ACTUATOR_ON);
  }
# endif
  updatePowerIndicator();
  Log.trace(F("ZactuatorONOFF setup done" CR));
}

# if jsonReceiving
void MQTTtoONOFF(char* topicOri, JsonObject& ONOFFdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoONOFF)) {
    Log.trace(F("MQTTtoONOFF json data analysis" CR));
    int boolSWITCHTYPE = ONOFFdata["cmd"] | 99;
    int gpio = ONOFFdata["gpio"] | ACTUATOR_ONOFF_GPIO;
    if (boolSWITCHTYPE != 99) {
      Log.notice(F("MQTTtoONOFF boolSWITCHTYPE ok: %d" CR), boolSWITCHTYPE);
      Log.notice(F("GPIO number: %d" CR), gpio);
      pinMode(gpio, OUTPUT);
      digitalWrite(gpio, boolSWITCHTYPE);
      if (boolSWITCHTYPE == ACTUATOR_ON) {
        PowerIndicatorON();
      } else {
        PowerIndicatorOFF();
      }
# ifdef ESP32
      if (ONOFFConfig.useLastStateOnStart) {
        ONOFFdata["save"] = true;
        ONOFFConfig_fromJson(ONOFFdata);
      }
# endif

      stateONOFFMeasures();
    } else {
      if (ONOFFdata["cmd"] == "high_pulse") {
        Log.notice(F("MQTTtoONOFF high_pulse ok" CR));
        Log.notice(F("GPIO number: %d" CR), gpio);
        int pulselength = ONOFFdata["pulse_length"] | 500;
        Log.notice(F("Pulse length: %d ms" CR), pulselength);
        pinMode(gpio, OUTPUT);
        digitalWrite(gpio, HIGH);
        delay(pulselength);
        digitalWrite(gpio, LOW);
      } else if (ONOFFdata["cmd"] == "low_pulse") {
        Log.notice(F("MQTTtoONOFF low_pulse ok" CR));
        Log.notice(F("GPIO number: %d" CR), gpio);
        int pulselength = ONOFFdata["pulse_length"] | 500;
        Log.notice(F("Pulse length: %d ms" CR), pulselength);
        pinMode(gpio, OUTPUT);
        digitalWrite(gpio, LOW);
        delay(pulselength);
        digitalWrite(gpio, HIGH);
      } else {
        Log.error(F("MQTTtoONOFF failed json read" CR));
      }
    }
  }
  if (cmpToMainTopic(topicOri, subjectMQTTtoONOFFset)) {
    Log.trace(F("MQTTtoONOFF json set" CR));






    if (ONOFFdata.containsKey("init") && ONOFFdata["init"].as<bool>()) {

      ONOFFConfig_init();
    } else if (ONOFFdata.containsKey("load") && ONOFFdata["load"].as<bool>()) {

      ONOFFConfig_load();
    }


    ONOFFConfig_fromJson(ONOFFdata);
    stateONOFFMeasures();
  }
}
# endif

# if simpleReceiving
void MQTTtoONOFF(char* topicOri, char* datacallback) {
  if ((cmpToMainTopic(topicOri, subjectMQTTtoONOFF))) {
    Log.trace(F("MQTTtoONOFF" CR));
    char* endptr = NULL;
    long gpio = strtol(datacallback, &endptr, 10);
    if (datacallback == endptr)
      gpio = ACTUATOR_ONOFF_GPIO;

    Log.notice(F("GPIO number: %d" CR), gpio);
    pinMode(gpio, OUTPUT);

    bool ON = false;
    if (strstr(topicOri, ONKey) != NULL)
      ON = true;
    if (strstr(topicOri, OFFKey) != NULL)
      ON = false;

    digitalWrite(gpio, ON);
    if (ON == ACTUATOR_ON) {
      PowerIndicatorON();
    } else {
      PowerIndicatorOFF();
    }

    char b = ON;
    pub(subjectGTWONOFFtoMQTT, &b);
  }
}
# endif


# ifdef MAX_TEMP_ACTUATOR
void overLimitTemp(void* pvParameters) {
# if defined(ESP32) && !defined(NO_INT_TEMP_READING)
  for (;;) {
    static float previousInternalTempc = 0;
    float internalTempc = intTemperatureRead();
    Log.trace(F("Internal temperature of the ESP32 %F" CR), internalTempc);

    if (internalTempc > MAX_TEMP_ACTUATOR && previousInternalTempc > MAX_TEMP_ACTUATOR) {
      if (digitalRead(ACTUATOR_ONOFF_GPIO) == ACTUATOR_ON) {
        Log.error(F("[ActuatorONOFF] OverTemperature detected ( %F > %F ) switching OFF Actuator" CR), internalTempc, MAX_TEMP_ACTUATOR);
        ActuatorTrigger();
        CriticalIndicatorON();
      }
    }
    previousInternalTempc = internalTempc;
    vTaskDelay(TimeBetweenReadingIntTemp);
  }
# endif
}
# endif


# ifdef MAX_CURRENT_ACTUATOR
void overLimitCurrent(float RN8209current) {
  static float RN8209previousCurrent = 0;
  Log.trace(F("RN8209 Current %F" CR), RN8209current);

  if (RN8209current > MAX_CURRENT_ACTUATOR && RN8209previousCurrent > MAX_CURRENT_ACTUATOR) {
    if (digitalRead(ACTUATOR_ONOFF_GPIO) == ACTUATOR_ON) {
      Log.error(F("[ActuatorONOFF] OverCurrent detected ( %F > %F ) switching OFF Actuator" CR), RN8209current, MAX_CURRENT_ACTUATOR);
      ActuatorTrigger();
      CriticalIndicatorON();
    }
  }
  RN8209previousCurrent = RN8209current;
}
# else
void overLimitCurrent(float RN8209current) {}
# endif






void ActuatorTrigger() {
  uint8_t level = !digitalRead(ACTUATOR_ONOFF_GPIO);
  Log.trace(F("Actuator triggered %d" CR), level);
  digitalWrite(ACTUATOR_ONOFF_GPIO, level);
  if (level == ACTUATOR_ON) {
    PowerIndicatorON();
  } else {
    PowerIndicatorOFF();
  }

# ifdef ESP32
  if (ONOFFConfig.useLastStateOnStart) {
    StaticJsonDocument<64> jsonBuffer;
    JsonObject ONOFFdata = jsonBuffer.to<JsonObject>();
    ONOFFdata["cmd"] = (int)level;
    ONOFFdata["save"] = true;
    ONOFFConfig_fromJson(ONOFFdata);
  }
# endif
  stateONOFFMeasures();
}

void stateONOFFMeasures() {

  StaticJsonDocument<128> jsonBuffer;
  JsonObject ONOFFdata = jsonBuffer.to<JsonObject>();
  ONOFFdata["cmd"] = (int)digitalRead(ACTUATOR_ONOFF_GPIO);
# ifdef ESP32
  ONOFFdata["uselaststate"] = ONOFFConfig.useLastStateOnStart;
# endif
  ONOFFdata["origin"] = subjectGTWONOFFtoMQTT;
  handleJsonEnqueue(ONOFFdata, QueueSemaphoreTimeOutTask);
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorPWM.ino"
# 68 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorPWM.ino"
#include "User_config.h"

#ifdef ZactuatorPWM

# include "config_PWM.h"

static long previousUpdateTime = 0;
static long currentUpdateTime = 0;

static const char* channelJsonKeys[] = PWM_CHANNEL_NAMES;
static const int channelPins[] = PWM_CHANNEL_PINS;
static const int kNumChannels = sizeof(channelPins) / sizeof(int);


static float currentValues[kNumChannels] = {};
static float fadeStartValues[kNumChannels] = {};
static float targetValues[kNumChannels] = {};

static long fadeStartUpdateTime[kNumChannels] = {};
static long fadeEndUpdateTime[kNumChannels] = {};
static bool fadeIsComplete = false;


static float calibrationMinLinear[kNumChannels];
static float calibrationMaxLinear[kNumChannels];
static float calibrationGamma[kNumChannels];

# if defined(ESP32)


static const int kNumDutyCycleBits = 16;
# elif defined(ESP8266)


static const int kNumDutyCycleBits = 10;
# else



static const int kNumDutyCycleBits = 8;
# endif

static const float kUNormToDutyCycle = (float)((1 << kNumDutyCycleBits) - 1);

void setupPWM() {
# 126 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorPWM.ino"
  for (int i = 0; i < kNumChannels; ++i) {
# if defined(ESP32)



    ledcSetup(i, 625.0, kNumDutyCycleBits);
    ledcAttachPin(channelPins[i], i);
# endif
    calibrationMinLinear[i] = 0.f;
    calibrationMaxLinear[i] = 1.f;
    calibrationGamma[i] = PWM_DEFAULT_GAMMA;
  }

  Log.trace(F("ZactuatorPWM setup done " CR));
}



static float perceptualToLinear(float perceptual, int channelIdx) {
  return pow(perceptual, calibrationGamma[channelIdx]);
}


void PWMLoop() {
  previousUpdateTime = currentUpdateTime;
  currentUpdateTime = millis();

  if (fadeIsComplete) {
    return;
  }

  fadeIsComplete = true;
  for (int i = 0; i < kNumChannels; ++i) {

    long totalFadeDuration = fadeEndUpdateTime[i] - fadeStartUpdateTime[i];
    float fadeLerpValue = 1.f;
    if (totalFadeDuration > 0) {
      fadeLerpValue = (float)(currentUpdateTime - fadeStartUpdateTime[i]) / (float)totalFadeDuration;
    }
    if (fadeLerpValue >= 1.f) {
      currentValues[i] = targetValues[i];
    } else {
      currentValues[i] = ((targetValues[i] - fadeStartValues[i]) * fadeLerpValue) + fadeStartValues[i];
      fadeIsComplete = false;
    }
  }



  for (int i = 0; i < kNumChannels; ++i) {
    float linear = perceptualToLinear(currentValues[i], i);


    if (linear > 0.f) {

      linear = (linear * (calibrationMaxLinear[i] - calibrationMinLinear[i])) + calibrationMinLinear[i];
    }

    long dutyCycle = (long)(linear * kUNormToDutyCycle);
# if defined(ESP32)
    ledcWrite(i, dutyCycle);
# else
    analogWrite(channelPins[i], dutyCycle);
# endif

  }
}

boolean PWMtoMQTT() {
  return false;
}

# if jsonReceiving
void MQTTtoPWM(char* topicOri, JsonObject& jsonData) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoPWMset)) {
    Log.trace(F("MQTTtoPWM JSON analysis" CR));

    int modifiedChannelBits = 0;
    for (int i = 0; i < kNumChannels; ++i) {
      fadeStartValues[i] = currentValues[i];
      JsonVariant value = jsonData[channelJsonKeys[i]];
      if (!value.isNull()) {
        float targetValue = value.as<float>();
        targetValue = std::min(targetValue, 1.f);
        targetValue = std::max(targetValue, 0.f);
        targetValues[i] = targetValue;


        fadeStartUpdateTime[i] = currentUpdateTime;
        fadeEndUpdateTime[i] = currentUpdateTime;

        modifiedChannelBits |= (1 << i);
      }
    }

    JsonVariant fade = jsonData["fade"];
    if (!fade.isNull()) {

      long endUpdateTime = currentUpdateTime + (long)(fade.as<float>() * (1000.f));

      for (int i = 0; i < kNumChannels; ++i) {
        if (modifiedChannelBits & (1 << i)) {
          fadeEndUpdateTime[i] = endUpdateTime;
        }
      }
    }
    fadeIsComplete = false;
  } else if (cmpToMainTopic(topicOri, subjectMQTTtoPWMcalibrate)) {

    for (int i = 0; i < kNumChannels; ++i) {
      char key[64];
      snprintf(key, sizeof(key), "gamma-%s", channelJsonKeys[i]);
      JsonVariant value = jsonData[key];
      if (!value.isNull()) {
        float gamma = value.as<float>();

        gamma = std::min(gamma, 4.f);
        gamma = std::max(gamma, 0.5f);
        calibrationGamma[i] = gamma;
      }
      snprintf(key, sizeof(key), "min-%s", channelJsonKeys[i]);
      value = jsonData[key];
      if (!value.isNull()) {
        calibrationMinLinear[i] = perceptualToLinear(value.as<float>(), i);
      }
      snprintf(key, sizeof(key), "max-%s", channelJsonKeys[i]);
      value = jsonData[key];
      if (!value.isNull()) {
        calibrationMaxLinear[i] = perceptualToLinear(value.as<float>(), i);
      }
    }
  }
}
# endif

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorSomfy.ino"
# 24 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZactuatorSomfy.ino"
#include "User_config.h"

#ifdef ZactuatorSomfy

# include <EEPROM.h>
# include <EEPROMRollingCodeStorage.h>
# include <SomfyRemote.h>

# ifdef ZradioCC1101
# include <ELECHOUSE_CC1101_SRC_DRV.h>
# endif

void setupSomfy() {
  pinMode(RF_EMITTER_GPIO, OUTPUT);
  digitalWrite(RF_EMITTER_GPIO, LOW);

# if defined(ESP32)
  if (!EEPROM.begin(max(4, SOMFY_REMOTE_NUM * 2))) {
    Log.error(F("failed to initialise EEPROM" CR));
  }
# elif defined(ESP8266)
  EEPROM.begin(max(4, SOMFY_REMOTE_NUM * 2));
# endif

  Log.trace(F("ZactuatorSomfy setup done " CR));
}

# if jsonReceiving
void MQTTtoSomfy(char* topicOri, JsonObject& jsonData) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoSomfy)) {
    Log.trace(F("MQTTtoSomfy json data analysis" CR));
    float txFrequency = jsonData["frequency"] | RFConfig.frequency;
# ifdef ZradioCC1101
    disableCurrentReceiver();
    ELECHOUSE_cc1101.SetTx(txFrequency);
    Log.notice(F("Transmit frequency: %F" CR), txFrequency);
# endif

    const int remoteIndex = jsonData["remote"];
    if (remoteIndex >= SOMFY_REMOTE_NUM) {
      Log.warning(F("ZactuatorSomfy remote does not exist" CR));
      return;
    }
    const String commandData = jsonData["command"];
    const Command command = getSomfyCommand(commandData);

    const int repeat = jsonData["repeat"] | 4;

    EEPROMRollingCodeStorage rollingCodeStorage(EEPROM_ADDRESS_START + remoteIndex * 2);
    SomfyRemote somfyRemote(RF_EMITTER_GPIO, somfyRemotes[remoteIndex], &rollingCodeStorage);
    somfyRemote.sendCommand(command, repeat);
    initCC1101();
    enableActiveReceiver();
  }
}
# endif

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZboardM5.ino"
# 29 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZboardM5.ino"
#include "User_config.h"

#if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5STACK) || defined(ZboardM5TOUGH)
# ifdef ZboardM5STICKC
# include <M5StickC.h>
# endif
# ifdef ZboardM5STICKCP
# include <M5StickCPlus.h>
# endif
# ifdef ZboardM5STACK
# include <M5Stack.h>
# endif
# ifdef ZboardM5TOUGH
# include <M5Tough.h>
# endif
void logToLCD(bool display) {
  display ? Log.begin(LOG_LEVEL_LCD, &M5.Lcd) : Log.begin(LOG_LEVEL, &Serial);
}

void setBrightness(int brightness) {
# if defined(ZboardM5STACK)
  M5.Lcd.setBrightness(brightness * 2);
# endif
# if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5TOUGH)
  (!brightness) ? M5.Axp.ScreenBreath(0) : M5.Axp.ScreenBreath(7 + (int)brightness * 0.08);
# endif
}

void setupM5() {
  Log.notice(F("Setup M5" CR));
  pinMode(SLEEP_BUTTON, INPUT);



  Log.notice(F("Low power set to: %d" CR), lowpowermode);
  switch (lowpowermode)
  {
    case 0:
      wakeScreen(NORMAL_LCD_BRIGHTNESS);
      M5.Lcd.fillScreen(WHITE);
      displayIntro(M5.Lcd.width() * 0.25, (M5.Lcd.width() / 2) + M5.Lcd.width() * 0.12, (M5.Lcd.height() / 2) + M5.Lcd.height() * 0.2);
# if LOG_TO_LCD
      Log.begin(LOG_LEVEL_LCD, &M5.Lcd);
# endif
      break;
    case 1:
      wakeScreen(SLEEP_LCD_BRIGHTNESS);
      M5.Lcd.fillScreen(WHITE);
# if LOG_TO_LCD
      Log.begin(LOG_LEVEL_LCD, &M5.Lcd);
# endif
      break;
    case 2:
      M5.begin(false, false, false);
      break;
  }
  Log.notice(F("Setup M5 end" CR));
}

void sleepScreen() {
  Log.trace(F("Screen going to sleep" CR));
# if defined(ZboardM5STACK)
  M5.begin(false, false, false);
# endif
# if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5TOUGH)
  M5.Axp.ScreenBreath(0);
  M5.Axp.SetLDO2(false);
# endif
}

void wakeScreen(int brightness) {
  Log.trace(F("Screen wake up" CR));
  M5.begin();
  M5.Lcd.setCursor(0, 0, (M5.Lcd.height() > 200) ? 4 : 2);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setRotation(1);
  setBrightness(brightness);
}

void loopM5() {
  static int previousBtnState;
  int currentBtnState = digitalRead(SLEEP_BUTTON);
  if (currentBtnState != previousBtnState && currentBtnState == 0) {
    int newlowpowermode = lowpowermode;
    (lowpowermode == 2) ? newlowpowermode = 0 : newlowpowermode = newlowpowermode + 1;
    changelowpowermode(newlowpowermode);
  }
  previousBtnState = currentBtnState;
  static int previousLogLevel;
  int currentLogLevel = Log.getLastMsgLevel();
  if (previousLogLevel != currentLogLevel && lowpowermode != 2) {
    switch (currentLogLevel) {
      case 1:
      case 2:
        wakeScreen(NORMAL_LCD_BRIGHTNESS);
        M5.Lcd.fillScreen(TFT_RED);
        M5.Lcd.setTextColor(TFT_BLACK, TFT_RED);
        break;
      case 3:
        wakeScreen(NORMAL_LCD_BRIGHTNESS);
        M5.Lcd.fillScreen(TFT_ORANGE);
        M5.Lcd.setTextColor(TFT_BLACK, TFT_ORANGE);
        break;
      default:
        wakeScreen(SLEEP_LCD_BRIGHTNESS);
        M5.Lcd.fillScreen(TFT_WHITE);
        drawLogo(M5.Lcd.width() * 0.1875, (M5.Lcd.width() / 2) - M5.Lcd.width() * 0.24, M5.Lcd.height() * 0.5, true, true, true, true, true, true);
        break;
    }
  }
  previousLogLevel = currentLogLevel;
}

void MQTTtoM5(char* topicOri, JsonObject& M5data) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoM5set)) {
    Log.trace(F("MQTTtoM5 json set" CR));

    if (M5data.containsKey("log-lcd")) {
      bool displayOnLCD = M5data["log-lcd"];
      Log.notice(F("Set lcd log: %T" CR), displayOnLCD);
      logToLCD(displayOnLCD);
    }
  }
}

void displayIntro(int i, int X, int Y) {
  Log.trace(F("Intro display on screen" CR));
  drawLogo(i, X, Y, false, true, false, false, false, false);
  delay(50);
  drawLogo(i, X, Y, false, false, true, false, false, false);
  delay(50);
  drawLogo(i, X, Y, false, true, false, false, false, false);
  delay(50);
  drawLogo(i, X, Y, false, true, true, false, false, false);
  delay(50);
  drawLogo(i, X, Y, false, true, true, true, false, false);
  delay(50);
  drawLogo(i, X, Y, false, true, true, false, true, false);
  delay(50);
  drawLogo(i, X, Y, false, true, true, true, true, false);
  delay(50);
  drawLogo(i, X, Y, true, true, true, true, true, false);
}

void drawLogo(int logoSize, int circle1X, int circle1Y, bool circle1, bool circle2, bool circle3, bool line1, bool line2, bool name) {
  int circle1T = logoSize / 15;
  int circle2T = logoSize / 25;
  int circle3T = logoSize / 30;

  int circle3Y = circle1Y - (logoSize * 1.2);
  int circle3X = circle1X - (logoSize * 0.13);
  int circle2X = circle1X - (logoSize * 1.05);
  int circle2Y = circle1Y - (logoSize * 0.8);

  if (line1) {
    M5.Lcd.drawLine(circle1X - 2, circle1Y, circle2X - 2, circle2Y, BLUE);
    M5.Lcd.drawLine(circle1X - 1, circle1Y, circle2X - 1, circle2Y, BLUE);
    M5.Lcd.drawLine(circle1X, circle1Y, circle2X, circle2Y, BLUE);
    M5.Lcd.drawLine(circle1X + 1, circle1Y, circle2X + 1, circle2Y, BLUE);
    M5.Lcd.drawLine(circle1X + 2, circle1Y, circle2X + 2, circle2Y, BLUE);
    M5.Lcd.fillCircle(circle3X, circle3Y, logoSize / 4 - circle3T * 2, WHITE);
  }
  if (line2) {
    M5.Lcd.drawLine(circle1X - 2, circle1Y, circle3X - 2, circle3Y, BLUE);
    M5.Lcd.drawLine(circle1X - 1, circle1Y, circle3X - 1, circle3Y, BLUE);
    M5.Lcd.drawLine(circle1X, circle1Y, circle3X, circle3Y, BLUE);
    M5.Lcd.drawLine(circle1X + 1, circle1Y, circle3X + 1, circle3Y, BLUE);
    M5.Lcd.fillCircle(circle2X, circle2Y, logoSize / 3 - circle2T * 2, WHITE);
  }
  M5.Lcd.fillCircle(circle1X, circle1Y, logoSize / 2 - circle1T * 2, WHITE);
  if (circle1) {
    M5.Lcd.fillCircle(circle1X, circle1Y, logoSize / 2, WHITE);
    M5.Lcd.fillCircle(circle1X, circle1Y, logoSize / 2 - circle1T, TFT_GREEN);
    M5.Lcd.fillCircle(circle1X, circle1Y, logoSize / 2 - circle1T * 2, WHITE);
  }
  if (circle2) {
    M5.Lcd.fillCircle(circle2X, circle2Y, logoSize / 3, WHITE);
    M5.Lcd.fillCircle(circle2X, circle2Y, logoSize / 3 - circle2T, TFT_ORANGE);
    M5.Lcd.fillCircle(circle2X, circle2Y, logoSize / 3 - circle2T * 2, WHITE);
  }
  if (circle3) {
    M5.Lcd.fillCircle(circle3X, circle3Y, logoSize / 4, WHITE);
    M5.Lcd.fillCircle(circle3X, circle3Y, logoSize / 4 - circle3T, TFT_PINK);
    M5.Lcd.fillCircle(circle3X, circle3Y, logoSize / 4 - circle3T * 2, WHITE);
  }
  if (name) {
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.drawString("penMQTTGateway", circle1X + (circle1X * 0.27), circle1Y, (M5.Lcd.height() > 200) ? 4 : 2);
  }
}

void M5Print(char* line1, char* line2, char* line3) {
  if (lowpowermode == 2) InfoIndicatorON();
  wakeScreen(NORMAL_LCD_BRIGHTNESS);
  M5.Lcd.fillScreen(TFT_WHITE);
  drawLogo(M5.Lcd.width() * 0.1875, (M5.Lcd.width() / 2) - M5.Lcd.width() * 0.24, M5.Lcd.height() * 0.5, true, true, true, true, true, true);
  M5.Lcd.setTextColor(BLUE);
  M5.Lcd.drawString(line1, 5, M5.Lcd.height() * 0.7, 1);
  M5.Lcd.drawString(line2, 5, M5.Lcd.height() * 0.8, 1);
  M5.Lcd.drawString(line3, 5, M5.Lcd.height() * 0.9, 1);
  delay(2000);
  InfoIndicatorOFF();
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZcommonRF.ino"
# 25 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZcommonRF.ino"
#include "User_config.h"

#if defined(ZgatewayRF) || defined(ZgatewayPilight) || defined(ZgatewayRTL_433) || defined(ZgatewayRF2) || defined(ZactuatorSomfy)
# ifndef ARDUINO_AVR_UNO

# ifdef ZradioCC1101
# include <ELECHOUSE_CC1101_SRC_DRV.h>
# endif

void initCC1101() {
# ifdef ZradioCC1101

  int delayMS = 16;
  int delayMaxMS = 500;
  for (int i = 0; i < 10; i++) {
    if (ELECHOUSE_cc1101.getCC1101()) {
      Log.notice(F("C1101 spi Connection OK" CR));
      ELECHOUSE_cc1101.Init();
      ELECHOUSE_cc1101.SetRx(RFConfig.frequency);
      break;
    } else {
      Log.error(F("C1101 spi Connection Error" CR));
      delay(delayMS);
    }

    delayMS = delayMS * 2;
    if (delayMS > delayMaxMS) delayMS = delayMaxMS;
  }
# endif
}

void setupCommonRF() {
  RFConfig_init();
  RFConfig_load();
}

bool validFrequency(float mhz) {

  if (mhz >= 300 && mhz <= 348)
    return true;
  if (mhz >= 387 && mhz <= 464)
    return true;
  if (mhz >= 779 && mhz <= 928)
    return true;
  return false;
}

int currentReceiver = ACTIVE_NONE;

# if !defined(ZgatewayRFM69) && !defined(ZactuatorSomfy)
# if defined(ESP8266) || defined(ESP32)

bool validReceiver(int receiver) {
  switch (receiver) {
# ifdef ZgatewayPilight
    case ACTIVE_PILIGHT:
      return true;
# endif
# ifdef ZgatewayRF
    case ACTIVE_RF:
      return true;
# endif
# ifdef ZgatewayRTL_433
    case ACTIVE_RTL:
      return true;
# endif
# ifdef ZgatewayRF2
    case ACTIVE_RF2:
      return true;
# endif
    default:
      Log.error(F("ERROR: stored receiver %d not available" CR), receiver);
  }
  return false;
}
# endif
# endif

void disableCurrentReceiver() {
  Log.trace(F("disableCurrentReceiver: %d" CR), currentReceiver);
  switch (currentReceiver) {
    case ACTIVE_NONE:
      break;
# ifdef ZgatewayPilight
    case ACTIVE_PILIGHT:
      disablePilightReceive();
      break;
# endif
# ifdef ZgatewayRF
    case ACTIVE_RF:
      disableRFReceive();
      break;
# endif
# ifdef ZgatewayRTL_433
    case ACTIVE_RTL:
      disableRTLreceive();
      break;
# endif
# ifdef ZgatewayRF2
    case ACTIVE_RF2:
      disableRF2Receive();
      break;
# endif
# ifndef ARDUINO_AVR_UNO
    default:
      Log.error(F("ERROR: unsupported receiver %d" CR), RFConfig.activeReceiver);
# endif
  }
}

void enableActiveReceiver() {
  Log.trace(F("enableActiveReceiver: %d" CR), RFConfig.activeReceiver);
  switch (RFConfig.activeReceiver) {
# ifdef ZgatewayPilight
    case ACTIVE_PILIGHT:
      initCC1101();
      enablePilightReceive();
      currentReceiver = ACTIVE_PILIGHT;
      break;
# endif
# ifdef ZgatewayRF
    case ACTIVE_RF:
      initCC1101();
      enableRFReceive();
      currentReceiver = ACTIVE_RF;
      break;
# endif
# ifdef ZgatewayRTL_433
    case ACTIVE_RTL:
      initCC1101();
      enableRTLreceive();
      currentReceiver = ACTIVE_RTL;
      break;
# endif
# ifdef ZgatewayRF2
    case ACTIVE_RF2:
      initCC1101();
      enableRF2Receive();
      currentReceiver = ACTIVE_RF2;
      break;
# endif
    case ACTIVE_RECERROR:
      Log.error(F("ERROR: no receiver selected" CR));
      break;
# ifndef ARDUINO_AVR_UNO
    default:
      Log.error(F("ERROR: unsupported receiver %d" CR), RFConfig.activeReceiver);
# endif
  }
}

String stateRFMeasures() {

  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject RFdata = jsonBuffer.to<JsonObject>();
  RFdata["active"] = RFConfig.activeReceiver;
# if defined(ZradioCC1101) || defined(ZradioSX127x)
  RFdata["frequency"] = RFConfig.frequency;
  if (RFConfig.activeReceiver == ACTIVE_RTL) {
# ifdef ZgatewayRTL_433
    RFdata["rssithreshold"] = (int)getRTLrssiThreshold();
    RFdata["rssi"] = (int)getRTLCurrentRSSI();
    RFdata["avgrssi"] = (int)getRTLAverageRSSI();
    RFdata["count"] = (int)getRTLMessageCount();
# endif
# ifdef ZradioSX127x
    RFdata["ookthreshold"] = (int)getOOKThresh();
# endif
  }
# endif
  pub(subjectcommonRFtoMQTT, RFdata);

  String output;
  serializeJson(RFdata, output);
  return output;
}

void RFConfig_fromJson(JsonObject& RFdata) {
  bool success = false;
  if (RFdata.containsKey("frequency") && validFrequency(RFdata["frequency"])) {
    Config_update(RFdata, "frequency", RFConfig.frequency);
    Log.notice(F("RF Receive mhz: %F" CR), RFConfig.frequency);
    success = true;
  }
  if (RFdata.containsKey("active")) {
    Log.notice(F("RF receiver active: %d" CR), RFConfig.activeReceiver);
    Config_update(RFdata, "active", RFConfig.activeReceiver);
    success = true;
  }
# ifdef ZgatewayRTL_433
  if (RFdata.containsKey("rssithreshold")) {
    Log.notice(F("RTL_433 RSSI Threshold : %d " CR), RFConfig.rssiThreshold);
    Config_update(RFdata, "rssithreshold", RFConfig.rssiThreshold);
    rtl_433.setRSSIThreshold(RFConfig.rssiThreshold);
    success = true;
  }
# if defined(RF_SX1276) || defined(RF_SX1278)
  if (RFdata.containsKey("ookthreshold")) {
    Config_update(RFdata, "ookthreshold", RFConfig.newOokThreshold);
    Log.notice(F("RTL_433 ookThreshold %d" CR), RFConfig.newOokThreshold);
    rtl_433.setOOKThreshold(RFConfig.newOokThreshold);
    success = true;
  }
# endif
  if (RFdata.containsKey("status")) {
    Log.notice(F("RF get status:" CR));
    rtl_433.getStatus();
    success = true;
  }
  if (!success) {
    Log.error(F("MQTTtoRF Fail json" CR));
  }
# endif
  disableCurrentReceiver();
  enableActiveReceiver();
# ifdef ESP32
  if (RFdata.containsKey("erase") && RFdata["erase"].as<bool>()) {

    preferences.begin(Gateway_Short_Name, false);
    if (preferences.isKey("RFConfig")) {
      int result = preferences.remove("RFConfig");
      Log.notice(F("RF config erase result: %d" CR), result);
      preferences.end();
      return;
    } else {
      Log.notice(F("RF config not found" CR));
      preferences.end();
    }
  }
  if (RFdata.containsKey("save") && RFdata["save"].as<bool>()) {
    StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject jo = jsonBuffer.to<JsonObject>();
    jo["frequency"] = RFConfig.frequency;
    jo["active"] = RFConfig.activeReceiver;

# ifdef ZgatewayRTL_433


# endif

    String conf = "";
    serializeJson(jsonBuffer, conf);
    preferences.begin(Gateway_Short_Name, false);
    int result = preferences.putString("RFConfig", conf);
    preferences.end();
    Log.notice(F("RF Config_save: %s, result: %d" CR), conf.c_str(), result);
  }
# endif
}

void RFConfig_init() {
  RFConfig.frequency = RF_FREQUENCY;
  RFConfig.activeReceiver = ACTIVE_RECEIVER;
  RFConfig.rssiThreshold = 0;
  RFConfig.newOokThreshold = 0;
}

void RFConfig_load() {
# ifdef ESP32
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  preferences.begin(Gateway_Short_Name, true);
  if (preferences.isKey("RFConfig")) {
    auto error = deserializeJson(jsonBuffer, preferences.getString("RFConfig", "{}"));
    preferences.end();
    if (error) {
      Log.error(F("RF Config deserialization failed: %s, buffer capacity: %u" CR), error.c_str(), jsonBuffer.capacity());
      return;
    }
    if (jsonBuffer.isNull()) {
      Log.warning(F("RF Config is null" CR));
      return;
    }
    JsonObject jo = jsonBuffer.as<JsonObject>();
    RFConfig_fromJson(jo);
    Log.notice(F("RF Config loaded" CR));
  } else {
    preferences.end();
    Log.notice(F("RF Config not found using default" CR));
    enableActiveReceiver();
  }
# else
  enableActiveReceiver();
# endif
}

void MQTTtoRFset(char* topicOri, JsonObject& RFdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoRFset)) {
    Log.trace(F("MQTTtoRF json set" CR));







    if (RFdata.containsKey("init") && RFdata["init"].as<bool>()) {

      RFConfig_init();
    } else if (RFdata.containsKey("load") && RFdata["load"].as<bool>()) {

      RFConfig_load();
    }


    RFConfig_fromJson(RFdata);
    stateRFMeasures();
  }
}
# else
void RFConfig_init() {}
void RFConfig_load() {}
void MQTTtoRFset(char* topicOri, JsonObject& RFdata) {}
void enableActiveReceiver() {}
# endif
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZdisplaySSD1306.ino"
# 34 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZdisplaySSD1306.ino"
#if defined(ZdisplaySSD1306)

# include <ArduinoJson.h>

# include "ArduinoLog.h"
# include "User_config.h"
# include "config_SSD1306.h"

SemaphoreHandle_t semaphoreOLEDOperation;

boolean logToOLEDDisplay = LOG_TO_OLED;
boolean jsonDisplay = JSON_TO_OLED;
boolean displayFlip = DISPLAY_FLIP;
boolean displayState = DISPLAY_STATE;
boolean idlelogo = DISPLAY_IDLE_LOGO;
uint8_t displayBrightness = DISPLAY_BRIGHTNESS;




void logToOLED(bool display) {
  logToOLEDDisplay = display;
  display ? Log.begin(LOG_LEVEL_OLED, &Oled) : Log.begin(LOG_LEVEL, &Serial);
}




void setupSSD1306() {
  SSD1306Config_init();
  SSD1306Config_load();
  Log.trace(F("Setup SSD1306 Display" CR));
  Log.trace(F("ZdisplaySSD1306 command topic: %s" CR), subjectMQTTtoSSD1306set);
  Log.trace(F("ZdisplaySSD1306 log-oled: %T" CR), logToOLEDDisplay);
  Log.trace(F("ZdisplaySSD1306 json-oled: %T" CR), jsonDisplay);
  Log.trace(F("ZdisplaySSD1306 DISPLAY_PAGE_INTERVAL: %d" CR), DISPLAY_PAGE_INTERVAL);
  Log.trace(F("ZdisplaySSD1306 DISPLAY_IDLE_LOGO: %T" CR), idlelogo);
  Log.trace(F("ZdisplaySSD1306 DISPLAY_FLIP: %T" CR), displayFlip);
  Oled.begin();
  Log.notice(F("Setup SSD1306 Display end" CR));

# if LOG_TO_OLED
  Log.begin(LOG_LEVEL_OLED, &Oled);
  jsonDisplay = false;
# else
  jsonDisplay = true;
# endif
}

boolean logoDisplayed = false;
unsigned long nextDisplayPage = uptime() + DISPLAY_PAGE_INTERVAL;




void loopSSD1306() {





  if (jsonDisplay && displayState) {
    if (uptime() >= nextDisplayPage && uxSemaphoreGetCount(semaphoreOLEDOperation) && currentWebUIMessage && newSSD1306Message) {
      if (!Oled.displayPage(currentWebUIMessage)) {
        Log.warning(F("[ssd1306] displayPage failed: %s" CR), currentWebUIMessage->title);
      }
      nextDisplayPage = uptime() + DISPLAY_PAGE_INTERVAL;
      logoDisplayed = false;
      newSSD1306Message = false;
    }
  }



  if (uptime() > nextDisplayPage + 1 && !logoDisplayed && idlelogo && displayState) {
    Oled.display->normalDisplay();
    Oled.fillScreen(BLACK);
    Oled.drawLogo(rand() % 13 - 5, rand() % 32 - 13);
    logoDisplayed = true;
  }
}






void MQTTtoSSD1306(char* topicOri, JsonObject& SSD1306data) {
  bool success = false;
  if (cmpToMainTopic(topicOri, subjectMQTTtoSSD1306set)) {
    Log.trace(F("MQTTtoSSD1306 json set" CR));

    if (SSD1306data.containsKey("onstate")) {
      displayState = SSD1306data["onstate"].as<bool>();
      Log.notice(F("Set display state: %T" CR), displayState);
      success = true;
    }
    if (SSD1306data.containsKey("brightness")) {
      displayBrightness = SSD1306data["brightness"].as<int>();
      Log.notice(F("Set brightness: %d" CR), displayBrightness);
      success = true;
    }
    if (SSD1306data.containsKey("log-oled")) {
      logToOLEDDisplay = SSD1306data["log-oled"].as<bool>();
      Log.notice(F("Set OLED log: %T" CR), logToOLEDDisplay);
      logToOLED(logToOLEDDisplay);
      if (logToOLEDDisplay) {
        jsonDisplay = false;
      }
      success = true;
    } else if (SSD1306data.containsKey("json-oled")) {
      jsonDisplay = SSD1306data["json-oled"].as<bool>();
      if (jsonDisplay) {
        logToOLEDDisplay = false;
        logToOLED(logToOLEDDisplay);
      }
      Log.notice(F("Set json-oled: %T" CR), jsonDisplay);
      success = true;
    }
    if (SSD1306data.containsKey("idlelogo")) {
      idlelogo = SSD1306data["idlelogo"].as<bool>();
      success = true;
    }
    if (SSD1306data.containsKey("display-flip")) {
      displayFlip = SSD1306data["display-flip"].as<bool>();
      Log.notice(F("Set display-flip: %T" CR), displayFlip);
      success = true;
    }

    if (SSD1306data.containsKey("save") && SSD1306data["save"]) {
      SSD1306Config_save();
      success = true;
    } else if (SSD1306data.containsKey("load") && SSD1306data["load"]) {
      success = SSD1306Config_load();
      if (success) {
        Log.notice(F("SSD1306 config loaded" CR));
      }
    } else if (SSD1306data.containsKey("init") && SSD1306data["init"]) {
      SSD1306Config_init();
      success = true;
      if (success) {
        Log.notice(F("SSD1306 config initialised" CR));
      }
    } else if (SSD1306data.containsKey("erase") && SSD1306data["erase"]) {

      preferences.begin(Gateway_Short_Name, false);
      if (preferences.isKey("SSD1306Config")) {
        success = preferences.remove("SSD1306Config");
      }
      preferences.end();
      if (success) {
        Log.notice(F("SSD1306 config erased" CR));
      }
    }
    if (success) {
      stateSSD1306Display();
    } else {
      Log.error(F("[ SSD1306 ] MQTTtoSSD1306 Fail json" CR), SSD1306data);
    }
  }
}

void SSD1306Config_save() {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject jo = jsonBuffer.to<JsonObject>();
  jo["onstate"] = displayState;
  jo["brightness"] = displayBrightness;
  jo["log-oled"] = logToOLEDDisplay;
  jo["json-oled"] = jsonDisplay;
  jo["idlelogo"] = idlelogo;
  jo["display-flip"] = displayFlip;

  String conf = "";
  serializeJson(jsonBuffer, conf);
  preferences.begin(Gateway_Short_Name, false);
  int result = preferences.putString("SSD1306Config", conf);
  preferences.end();
  Log.notice(F("SSD1306 Config_save: %s, result: %d" CR), conf.c_str(), result);
}

void SSD1306Config_init() {
  displayState = DISPLAY_STATE;
  displayBrightness = DISPLAY_BRIGHTNESS;
  logToOLEDDisplay = LOG_TO_OLED;
  jsonDisplay = JSON_TO_OLED;
  idlelogo = DISPLAY_IDLE_LOGO;
  displayFlip = DISPLAY_FLIP;
  Log.notice(F("SSD1306 config initialised" CR));
}

bool SSD1306Config_load() {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  preferences.begin(Gateway_Short_Name, true);
  if (preferences.isKey("SSD1306Config")) {
    auto error = deserializeJson(jsonBuffer, preferences.getString("SSD1306Config", "{}"));
    preferences.end();
    if (error) {
      Log.error(F("SSD1306 config deserialization failed: %s, buffer capacity: %u" CR), error.c_str(), jsonBuffer.capacity());
      return false;
    }
    if (jsonBuffer.isNull()) {
      Log.warning(F("SSD1306 config is null" CR));
      return false;
    }
    JsonObject jo = jsonBuffer.as<JsonObject>();
    displayState = jo["onstate"].as<bool>();
    displayBrightness = jo["brightness"].as<int>();
    logToOLEDDisplay = jo["log-oled"].as<bool>();
    jsonDisplay = jo["json-oled"].as<bool>();
    idlelogo = jo["idlelogo"].as<bool>();
    displayFlip = jo["display-flip"].as<bool>();
    Log.notice(F("Saved SSD1306 config loaded" CR));
    return true;
  } else {
    preferences.end();
    Log.notice(F("No SSD1306 config to load" CR));
    return false;
  }
}






void ssd1306Print(char* line1, char* line2, char* line3) {
  Oled.println(line1);
  Oled.println(line2);
  Oled.println(line3);
  delay(2000);
}




void ssd1306Print(char* line1, char* line2) {
  Oled.println(line1);
  Oled.println(line2);
  delay(2000);
}




void ssd1306Print(char* line1) {
  Oled.println(line1);
  delay(2000);
}



OledSerial Oled(0);
OledSerial::OledSerial(int x) {
# if defined(WIFI_Kit_32) || defined(WIFI_LoRa_32) || defined(WIFI_LoRa_32_V2)
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, LOW);
  delay(50);
  digitalWrite(RST_OLED, HIGH);
  display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64);
# elif defined(Wireless_Stick)




  display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_64_32);
# elif defined(ARDUINO_TTGO_LoRa32_v21new)




  display = new SSD1306Wire(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);
# elif defined(GenericSSD1306)
  display = new SSD1306Wire(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);
# endif
}




void OledSerial::begin() {


  semaphoreOLEDOperation = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphoreOLEDOperation);

  display->init();
  if (displayFlip) {
    display->flipScreenVertically();
  } else {
    display->resetOrientation();
  }
  display->setFont(ArialMT_Plain_10);
  display->setBrightness(round(displayBrightness * 2.55));
  drawLogo(0, 0);
  display->invertDisplay();
  display->setLogBuffer(OLED_TEXT_ROWS, OLED_TEXT_BUFFER);
  delay(1000);

  if (!displayState) {
    display->displayOff();
  }
}




int OledSerial::available(void) {
}




int OledSerial::peek(void) {
}




int OledSerial::read(void) {
}




void OledSerial::flush(void) {
}




void OledSerial::fillScreen(OLEDDISPLAY_COLOR color) {
  if (xSemaphoreTake(semaphoreOLEDOperation, pdMS_TO_TICKS(30000)) == pdTRUE) {
    display->clear();
    display->setColor(color);
    display->fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT);
    xSemaphoreGive(semaphoreOLEDOperation);
  }
}




size_t OledSerial::write(const uint8_t* buffer, size_t size) {
  if (xPortGetCoreID() == CONFIG_ARDUINO_RUNNING_CORE) {
    if (xSemaphoreTake(semaphoreOLEDOperation, pdMS_TO_TICKS(30000)) == pdTRUE) {
      nextDisplayPage = uptime() + DISPLAY_PAGE_INTERVAL;
      display->normalDisplay();
      display->clear();
      display->setColor(WHITE);
      display->setFont(ArialMT_Plain_10);
      while (size) {
        display->write((char)*buffer++);
        size--;
      }
      display->drawLogBuffer(0, 0);
      display->display();
      xSemaphoreGive(semaphoreOLEDOperation);
      return size;
    }
  }

  return Serial.write(buffer, size);
}





boolean OledSerial::displayPage(webUIQueueMessage* message) {
  if (xPortGetCoreID() == CONFIG_ARDUINO_RUNNING_CORE) {
    if (xSemaphoreTake(semaphoreOLEDOperation, pdMS_TO_TICKS(30000)) == pdTRUE) {
      display->normalDisplay();
      display->clear();
      display->setColor(WHITE);
      display->setFont(ArialMT_Plain_10);
      display->drawString(0, 0, message->title);
      display->drawLine(0, 12, OLED_WIDTH, 12);
      display->drawString(0, 13, message->line1);
      display->drawString(0, 26, message->line2);
      display->drawString(0, 39, message->line3);
      display->drawString(0, 52, message->line4);
      display->display();
      xSemaphoreGive(semaphoreOLEDOperation);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}




void OledSerial::drawLogo(int xshift, int yshift) {
  if (xSemaphoreTake(semaphoreOLEDOperation, pdMS_TO_TICKS(30000)) == pdTRUE) {
    display->setColor(WHITE);

    display->drawLine(15 + xshift, 28 + yshift, 20 + xshift, 31 + yshift);
    display->drawLine(15 + xshift, 29 + yshift, 20 + xshift, 32 + yshift);

    display->drawLine(25 + xshift, 29 + yshift, 22 + xshift, 21 + yshift);
    display->drawLine(26 + xshift, 29 + yshift, 23 + xshift, 21 + yshift);

    display->fillCircle(25 + xshift, 35 + yshift, 7);
    display->setColor(BLACK);
    display->fillCircle(25 + xshift, 35 + yshift, 5);

    display->setColor(WHITE);
    display->fillCircle(23 + xshift, 18 + yshift, 4);
    display->setColor(BLACK);
    display->fillCircle(23 + xshift, 18 + yshift, 2);

    display->setColor(WHITE);
    display->fillCircle(11 + xshift, 25 + yshift, 5);
    display->setColor(BLACK);
    display->fillCircle(11 + xshift, 25 + yshift, 3);

    display->setColor(WHITE);
    display->drawString(32 + xshift, 32 + yshift, "penMQTTGateway");

    display->display();
    xSemaphoreGive(semaphoreOLEDOperation);
  }
}

String stateSSD1306Display() {

  StaticJsonDocument<JSON_MSG_BUFFER> DISPLAYdataBuffer;
  JsonObject DISPLAYdata = DISPLAYdataBuffer.to<JsonObject>();
  DISPLAYdata["onstate"] = (bool)displayState;
  DISPLAYdata["brightness"] = (int)displayBrightness;
  DISPLAYdata["display-flip"] = (bool)displayFlip;
  DISPLAYdata["idlelogo"] = (bool)idlelogo;
  DISPLAYdata["log-oled"] = (bool)logToOLEDDisplay;
  DISPLAYdata["json-oled"] = (bool)jsonDisplay;
  DISPLAYdata["origin"] = subjectSSD1306toMQTT;
  handleJsonEnqueue(DISPLAYdata);


  Oled.display->setBrightness(round(displayBrightness * 2.55));

  if (!displayState) {
    Oled.display->displayOff();
  } else {
    Oled.display->displayOn();
  }

  if (displayFlip) {
    Oled.display->flipScreenVertically();
  } else {
    Oled.display->resetOrientation();
  }
  String output;
  serializeJson(DISPLAYdata, output);
  return output;
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/Zgateway2G.ino"
# 28 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/Zgateway2G.ino"
#include "User_config.h"

#ifdef Zgateway2G

# include <A6lib.h>
# include <ArduinoJson.h>


A6lib A6l(_2G_TX_GPIO, _2G_RX_GPIO);

int unreadSMSLocs[50] = {0};
int unreadSMSNum = 0;
SMSmessage sms;

void setup2G() {
  Log.notice(F("_2G_TX_GPIO: %d " CR), _2G_TX_GPIO);
  Log.notice(F("_2G_RX_GPIO: %d " CR), _2G_RX_GPIO);
  setupGSM(false);
  Log.trace(F("Zgateway2G setup done " CR));
}

void setupGSM(bool deleteSMS) {
  Log.trace(F("Init 2G module: %d" CR), _2G_PWR_GPIO);
  delay(1000);

  A6l.powerCycle(_2G_PWR_GPIO);
  Log.notice(F("waiting for network connection at bd: %d" CR), _2G_MODULE_BAUDRATE);
  A6l.blockUntilReady(_2G_MODULE_BAUDRATE);
  Log.notice(F("A6/A7 gsm ready" CR));
  signalStrengthAnalysis();
  delay(1000);

  if (deleteSMS) {
    if (A6l.deleteSMS(1, 4) == A6_OK) {
      Log.notice(F("delete SMS OK" CR));
    } else {
      Log.error(F("delete SMS KO" CR));
    }
  }
}

void signalStrengthAnalysis() {
  int signalStrength = 0;
  signalStrength = A6l.getSignalStrength();
  Log.trace(F("Signal strength: %d" CR), signalStrength);
  if (signalStrength < _2G_MIN_SIGNAL || signalStrength > _2G_MAX_SIGNAL) {
    Log.trace(F("Signal too low restart the module" CR));
    setupGSM(false);
  }
}

bool _2GtoMQTT() {

  unreadSMSNum = A6l.getUnreadSMSLocs(unreadSMSLocs, 512);
  Log.trace(F("Creating SMS  buffer" CR));
  StaticJsonDocument<JSON_MSG_BUFFER> SMSdataBuffer;
  JsonObject SMSdata = SMSdataBuffer.to<JsonObject>();
  for (int i = 0; i < unreadSMSNum; i++) {
    Log.notice(F("New  message at index: %d" CR), unreadSMSNum);
    sms = A6l.readSMS(unreadSMSLocs[i]);
    SMSdata["message"] = (const char*)sms.message.c_str();
    SMSdata["date"] = (const char*)sms.date.c_str();
    SMSdata["phone"] = (const char*)sms.number.c_str();
    A6l.deleteSMS(unreadSMSLocs[i]);
    Log.trace(F("Adv data 2GtoMQTT" CR));
    SMSdata["origin"] = subject2GtoMQTT;
    return handleJsonEnqueue(SMSdata);
  }
  return false;
}
# if simpleReceiving
void MQTTto2G(char* topicOri, char* datacallback) {
  String data = datacallback;
  String topic = topicOri;

  if (cmpToMainTopic(topicOri, subjectMQTTto2G)) {
    Log.trace(F("MQTTto2G data analysis" CR));

    String phone_number = "";
    int pos0 = topic.lastIndexOf(_2GPhoneKey);
    if (pos0 != -1) {
      pos0 = pos0 + strlen(_2GPhoneKey);
      phone_number = topic.substring(pos0);
      Log.notice(F("MQTTto2G phone: %s" CR), (char*)phone_number.c_str());
      Log.notice(F("MQTTto2G sms: %s" CR), (char*)data.c_str());
      if (A6l.sendSMS(phone_number, data) == A6_OK) {
        Log.notice(F("SMS OK" CR));

        pub(subjectGTW2GtoMQTT, "SMS OK");
      } else {
        Log.error(F("SMS KO" CR));

        pub(subjectGTW2GtoMQTT, "SMS KO");
      }
    } else {
      Log.error(F("MQTTto2G Fail reading phone number" CR));
    }
  }
}
# endif

# if jsonReceiving
void MQTTto2G(char* topicOri, JsonObject& SMSdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTto2G)) {
    const char* sms = SMSdata["message"];
    const char* phone = SMSdata["phone"];
    Log.trace(F("MQTTto2G json data analysis" CR));
    if (sms && phone) {
      Log.notice(F("MQTTto2G phone: %s" CR), phone);
      Log.notice(F("MQTTto2G sms: %s" CR), sms);
      if (A6l.sendSMS(String(phone), String(sms)) == A6_OK) {
        Log.notice(F("SMS OK" CR));

        pub(subjectGTW2GtoMQTT, "SMS OK");
      } else {
        Log.error(F("SMS KO" CR));
        pub(subjectGTW2GtoMQTT, "SMS KO");
      }
    } else {
      Log.error(F("MQTTto2G failed json read" CR));
    }
  }
}
# endif
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayBLEConnect.ino"
#ifdef ESP32
# include "User_config.h"
# ifdef ZgatewayBT
# include "ArduinoJson.h"
# include "ArduinoLog.h"
# include "ZgatewayBLEConnect.h"
#define convertTemp_CtoF(c) ((c * 1.8) + 32)

extern std::vector<BLEdevice*> devices;

NimBLERemoteCharacteristic* zBLEConnect::getCharacteristic(const NimBLEUUID& service,
                                                           const NimBLEUUID& characteristic) {
  BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
  if (!m_pClient) {
    Log.error(F("No BLE client" CR));
  } else if (!m_pClient->isConnected() && !m_pClient->connect()) {
    Log.error(F("Connect to: %s failed" CR), m_pClient->getPeerAddress().toString().c_str());
  } else {
    BLERemoteService* pRemoteService = m_pClient->getService(service);
    if (!pRemoteService) {
      Log.notice(F("Failed to find service UUID: %s" CR), service.toString().c_str());
    } else {
      Log.trace(F("Found service: %s" CR), service.toString().c_str());
      Log.trace(F("Client isConnected, freeHeap: %d" CR), ESP.getFreeHeap());
      pRemoteCharacteristic = pRemoteService->getCharacteristic(characteristic);
      if (!pRemoteCharacteristic) {
        Log.notice(F("Failed to find characteristic UUID: %s" CR), characteristic.toString().c_str());
      }
    }
  }

  return pRemoteCharacteristic;
}

bool zBLEConnect::writeData(BLEAction* action) {
  NimBLERemoteCharacteristic* pChar = getCharacteristic(action->service, action->characteristic);
  if (pChar && (pChar->canWrite() || pChar->canWriteNoResponse())) {
    switch (action->value_type) {
      case BLE_VAL_HEX: {
        int len = action->value.length();
        if (len % 2) {
          Log.error(F("Invalid HEX value length" CR));
          return false;
        }

        std::vector<uint8_t> buf;
        for (auto i = 0; i < len; i += 2) {
          std::string temp = action->value.substr(i, 2);
          buf.push_back((uint8_t)strtoul(temp.c_str(), nullptr, 16));
        }
        return pChar->writeValue((const uint8_t*)&buf[0], buf.size(), !pChar->canWriteNoResponse());
      }
      case BLE_VAL_INT:
        return pChar->writeValue(strtol(action->value.c_str(), nullptr, 0), !pChar->canWriteNoResponse());
      case BLE_VAL_FLOAT:
        return pChar->writeValue(strtod(action->value.c_str(), nullptr), !pChar->canWriteNoResponse());
      default:
        return pChar->writeValue(action->value, !pChar->canWriteNoResponse());
    }
  }
  return false;
}

bool zBLEConnect::readData(BLEAction* action) {
  NimBLERemoteCharacteristic* pChar = getCharacteristic(action->service, action->characteristic);

  if (pChar && pChar->canRead()) {
    action->value = pChar->readValue();
    if (action->value != "") {
      return true;
    }
  }
  return false;
}

bool zBLEConnect::processActions(std::vector<BLEAction>& actions) {
  bool result = false;
  if (actions.size() > 0) {
    for (auto& it : actions) {
      if (NimBLEAddress(it.addr) == m_pClient->getPeerAddress()) {
        DynamicJsonDocument BLEdataBuffer(JSON_MSG_BUFFER);
        JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
        BLEdata["id"] = it.addr;
        BLEdata["service"] = it.service.toString();
        BLEdata["characteristic"] = it.characteristic.toString();

        if (it.write) {
          Log.trace(F("processing BLE write" CR));
          BLEdata["write"] = it.value;
          result = writeData(&it);
        } else {
          Log.trace(F("processing BLE read" CR));
          result = readData(&it);
          if (result) {
            switch (it.value_type) {
              case BLE_VAL_HEX: {
                char* pHex = NimBLEUtils::buildHexData(nullptr, (uint8_t*)it.value.c_str(), it.value.length());
                BLEdata["read"] = pHex;
                free(pHex);
                break;
              }
              case BLE_VAL_INT: {
                int ival = *(int*)it.value.data();
                BLEdata["read"] = ival;
                break;
              }
              case BLE_VAL_FLOAT: {
                float fval = *(double*)it.value.data();
                BLEdata["read"] = fval;
                break;
              }
              default:
                BLEdata["read"] = it.value.c_str();
                break;
            }
          }
        }

        it.complete = result;
        BLEdata["success"] = result;
        if (result || it.ttl <= 1) {
          buildTopicFromId(BLEdata, subjectBTtoMQTT);
          handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
        }
      }
    }
  }

  return result;
}


void LYWSD03MMC_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (m_taskHandle == nullptr) {
    return;
  }
  if (!BTProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());

    if (length == 5) {
      Log.trace(F("Device identified creating BLE buffer" CR));
      DynamicJsonDocument BLEdataBuffer(JSON_MSG_BUFFER);
      JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
      String mac_address = m_pClient->getPeerAddress().toString().c_str();
      mac_address.toUpperCase();
      for (std::vector<BLEdevice*>::iterator it = devices.begin(); it != devices.end(); ++it) {
        BLEdevice* p = *it;
        if ((strcmp(p->macAdr, (char*)mac_address.c_str()) == 0)) {
          if (p->sensorModel_id == BLEconectable::id::LYWSD03MMC)
            BLEdata["model"] = "LYWSD03MMC";
          else if (p->sensorModel_id == BLEconectable::id::MHO_C401)
            BLEdata["model"] = "MHO-C401";
        }
      }
      BLEdata["id"] = (char*)mac_address.c_str();
      Log.trace(F("Device identified in CB: %s" CR), (char*)mac_address.c_str());
      BLEdata["tempc"] = (float)((pData[0] | (pData[1] << 8)) * 0.01);
      BLEdata["tempf"] = (float)(convertTemp_CtoF((pData[0] | (pData[1] << 8)) * 0.01));
      BLEdata["hum"] = (float)(pData[2]);
      BLEdata["volt"] = (float)(((pData[4] * 256) + pData[3]) / 1000.0);
      BLEdata["batt"] = (float)(((((pData[4] * 256) + pData[3]) / 1000.0) - 2.1) * 100);
      buildTopicFromId(BLEdata, subjectBTtoMQTT);
      handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
    } else {
      Log.notice(F("Invalid notification data" CR));
      return;
    }
  } else {
    Log.trace(F("Callback process canceled by BTProcessLock" CR));
  }

  xTaskNotifyGive(m_taskHandle);
}

void LYWSD03MMC_connect::publishData() {
  NimBLEUUID serviceUUID("ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6");
  NimBLEUUID charUUID("ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar && pChar->canNotify()) {
    Log.trace(F("Registering notification" CR));
    if (pChar->subscribe(true, std::bind(&LYWSD03MMC_connect::notifyCB, this,
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4))) {
      m_taskHandle = xTaskGetCurrentTaskHandle();
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT)) == pdFALSE) {
        m_taskHandle = nullptr;
      }
    } else {
      Log.notice(F("Failed registering notification" CR));
    }
  }
}


void DT24_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (m_taskHandle == nullptr) {
    return;
  }

  if (!BTProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());
    if (length == 20) {
      m_data.assign(pData, pData + length);
      return;
    } else if (m_data.size() == 20 && length == 16) {
      m_data.insert(m_data.end(), pData, pData + length);




      Log.trace(F("Device identified creating BLE buffer" CR));
      DynamicJsonDocument BLEdataBuffer(JSON_MSG_BUFFER);
      JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
      String mac_address = m_pClient->getPeerAddress().toString().c_str();
      mac_address.toUpperCase();
      BLEdata["model"] = "DT24";
      BLEdata["id"] = (char*)mac_address.c_str();
      Log.trace(F("Device identified in CB: %s" CR), (char*)mac_address.c_str());
      BLEdata["volt"] = (float)(((m_data[4] * 256 * 256) + (m_data[5] * 256) + m_data[6]) / 10.0);
      BLEdata["current"] = (float)(((m_data[7] * 256 * 256) + (m_data[8] * 256) + m_data[9]) / 1000.0);
      BLEdata["power"] = (float)(((m_data[10] * 256 * 256) + (m_data[11] * 256) + m_data[12]) / 10.0);
      BLEdata["energy"] = (float)(((m_data[13] * 256 * 256 * 256) + (m_data[14] * 256 * 256) + (m_data[15] * 256) + m_data[16]) / 100.0);
      BLEdata["price"] = (float)(((m_data[17] * 256 * 256) + (m_data[18] * 256) + m_data[19]) / 100.0);
      BLEdata["tempc"] = (float)(m_data[24] * 256) + m_data[25];
      BLEdata["tempf"] = (float)(convertTemp_CtoF((m_data[24] * 256) + m_data[25]));
      buildTopicFromId(BLEdata, subjectBTtoMQTT);
      handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
    } else {
      Log.notice(F("Invalid notification data" CR));
      return;
    }
  } else {
    Log.trace(F("Callback process canceled by BTProcessLock" CR));
  }

  xTaskNotifyGive(m_taskHandle);
}

void DT24_connect::publishData() {
  NimBLEUUID serviceUUID("ffe0");
  NimBLEUUID charUUID("ffe1");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar && pChar->canNotify()) {
    Log.trace(F("Registering notification" CR));
    if (pChar->subscribe(true, std::bind(&DT24_connect::notifyCB, this,
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4))) {
      m_taskHandle = xTaskGetCurrentTaskHandle();
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT)) == pdFALSE) {
        m_taskHandle = nullptr;
      }
    } else {
      Log.notice(F("Failed registering notification" CR));
    }
  }
}


void BM2_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (m_taskHandle == nullptr) {
    return;
  }

  if (!BTProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());
    if (length == 16) {
      Log.trace(F("Device identified creating BLE buffer" CR));
      DynamicJsonDocument BLEdataBuffer(JSON_MSG_BUFFER);
      JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
      String mac_address = m_pClient->getPeerAddress().toString().c_str();
      mac_address.toUpperCase();
      BLEdata["model"] = "BM2 Battery Monitor";
      BLEdata["id"] = (char*)mac_address.c_str();
      mbedtls_aes_context aes;
      mbedtls_aes_init(&aes);
      unsigned char output[16];
      unsigned char iv[16] = {};
      unsigned char key[16] = {
          108,
          101,
          97,
          103,
          101,
          110,
          100,
          255,
          254,
          49,
          56,
          56,
          50,
          52,
          54,
          54,
      };
      mbedtls_aes_setkey_dec(&aes, key, 128);
      mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, 16, iv, (uint8_t*)&pData[0], output);
      mbedtls_aes_free(&aes);
      float volt = ((output[2] | (output[1] << 8)) >> 4) / 100.0f;
      BLEdata["volt"] = volt;
      Log.trace(F("volt: %F" CR), volt);
      buildTopicFromId(BLEdata, subjectBTtoMQTT);
      handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
    } else {
      Log.notice(F("Invalid notification data" CR));
      return;
    }
  } else {
    Log.trace(F("Callback process canceled by BTProcessLock" CR));
  }

  xTaskNotifyGive(m_taskHandle);
}

void BM2_connect::publishData() {
  NimBLEUUID serviceUUID("fff0");
  NimBLEUUID charUUID("fff4");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar && pChar->canNotify()) {
    Log.trace(F("Registering notification" CR));
    if (pChar->subscribe(true, std::bind(&BM2_connect::notifyCB, this,
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4))) {
      m_taskHandle = xTaskGetCurrentTaskHandle();
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT)) == pdFALSE) {
        m_taskHandle = nullptr;
      }
    } else {
      Log.notice(F("Failed registering notification" CR));
    }
  }
}


void HHCCJCY01HHCC_connect::publishData() {
  NimBLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");
  NimBLEUUID charUUID("00001a00-0000-1000-8000-00805f9b34fb");
  NimBLEUUID charUUID2("00001a02-0000-1000-8000-00805f9b34fb");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar) {
    Log.trace(F("Read mode" CR));
    uint8_t buf[2] = {0xA0, 0x1F};
    pChar->writeValue(buf, 2, true);
    int batteryValue = -1;
    NimBLERemoteCharacteristic* pChar2 = getCharacteristic(serviceUUID, charUUID2);
    if (pChar2) {
      std::string value;
      value = pChar2->readValue();
      const char* val2 = value.c_str();
      batteryValue = val2[0];
      DynamicJsonDocument BLEdataBuffer(JSON_MSG_BUFFER);
      JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
      String mac_address = m_pClient->getPeerAddress().toString().c_str();
      mac_address.toUpperCase();
      BLEdata["model"] = "HHCCJCY01HHCC";
      BLEdata["id"] = (char*)mac_address.c_str();
      BLEdata["batt"] = (int)batteryValue;
      buildTopicFromId(BLEdata, subjectBTtoMQTT);
      handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
    } else {
      Log.notice(F("Failed getting characteristic" CR));
    }
  } else {
    Log.notice(F("Failed getting characteristic" CR));
  }
}


void XMWSDJ04MMC_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (m_taskHandle == nullptr) {
    return;
  }
  if (!BTProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());

    if (length == 6) {
      Log.trace(F("Device identified creating BLE buffer" CR));
      DynamicJsonDocument BLEdataBuffer(JSON_MSG_BUFFER);
      JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
      String mac_address = m_pClient->getPeerAddress().toString().c_str();
      mac_address.toUpperCase();
      BLEdata["model"] = "XMWSDJ04MMC";
      BLEdata["id"] = (char*)mac_address.c_str();
      Log.trace(F("Device identified in CB: %s" CR), (char*)mac_address.c_str());
      BLEdata["tempc"] = (float)((pData[0] | (pData[1] << 8)) * 0.1);
      BLEdata["tempf"] = (float)(convertTemp_CtoF((pData[0] | (pData[1] << 8)) * 0.1));
      BLEdata["hum"] = (float)((pData[2] | (pData[3] << 8)) * 0.1);
      BLEdata["volt"] = (float)((pData[4] | (pData[5] << 8)) / 1000.0);
      BLEdata["batt"] = (float)((((pData[4] | (pData[5] << 8)) / 1000.0) - 2.1) * 100);
      buildTopicFromId(BLEdata, subjectBTtoMQTT);
      handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
    } else {
      Log.notice(F("Invalid notification data" CR));
      return;
    }
  } else {
    Log.trace(F("Callback process canceled by BTProcessLock" CR));
  }

  xTaskNotifyGive(m_taskHandle);
}

void XMWSDJ04MMC_connect::publishData() {
  NimBLEUUID serviceUUID("ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6");
  NimBLEUUID charUUID("ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar && pChar->canNotify()) {
    Log.trace(F("Registering notification" CR));
    if (pChar->subscribe(true, std::bind(&XMWSDJ04MMC_connect::notifyCB, this,
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4))) {
      m_taskHandle = xTaskGetCurrentTaskHandle();
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT)) == pdFALSE) {
        m_taskHandle = nullptr;
      }
    } else {
      Log.notice(F("Failed registering notification" CR));
    }
  }
}


void SBS1_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (m_taskHandle == nullptr) {
    return;
  }
  if (!BTProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());

    if (length) {
      m_notifyVal = *pData;
    } else {
      Log.notice(F("Invalid notification data" CR));
      return;
    }
  } else {
    Log.trace(F("Callback process canceled by BTProcessLock" CR));
  }

  xTaskNotifyGive(m_taskHandle);
}

bool SBS1_connect::processActions(std::vector<BLEAction>& actions) {
  NimBLEUUID serviceUUID("cba20d00-224d-11e6-9fb8-0002a5d5c51b");
  NimBLEUUID charUUID("cba20002-224d-11e6-9fb8-0002a5d5c51b");
  NimBLEUUID notifyCharUUID("cba20003-224d-11e6-9fb8-0002a5d5c51b");
  static byte ON[] = {0x57, 0x01, 0x01};
  static byte OFF[] = {0x57, 0x01, 0x02};

  bool result = false;
  if (actions.size() > 0) {
    for (auto& it : actions) {
      if (NimBLEAddress(it.addr) == m_pClient->getPeerAddress()) {
        NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);
        NimBLERemoteCharacteristic* pNotifyChar = getCharacteristic(serviceUUID, notifyCharUUID);

        if (it.write && pChar && pNotifyChar) {
          Log.trace(F("processing Switchbot %s" CR), it.value.c_str());
          if (pNotifyChar->subscribe(true,
                                     std::bind(&SBS1_connect::notifyCB,
                                               this, std::placeholders::_1, std::placeholders::_2,
                                               std::placeholders::_3, std::placeholders::_4),
                                     true)) {
            if (it.value == "on") {
              result = pChar->writeValue(ON, 3, false);
            } else {
              result = pChar->writeValue(OFF, 3, false);
            }

            if (result) {
              m_taskHandle = xTaskGetCurrentTaskHandle();
              if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT)) == pdFALSE) {
                m_taskHandle = nullptr;
              }
              result = m_notifyVal == 0x01;
            }
          }
        }

        it.complete = result;
        if (result || it.ttl <= 1) {
          StaticJsonDocument<JSON_MSG_BUFFER> BLEdataBuffer;
          JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
          BLEdata["id"] = it.addr;
          BLEdata["state"] = result ? it.value : it.value == "on" ? "off" : "on";
          buildTopicFromId(BLEdata, subjectBTtoMQTT);
          handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
        }
      }
    }
  }

  return result;
}

# endif
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayBT.ino"
# 31 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayBT.ino"
#include "User_config.h"

#ifdef ZgatewayBT

SemaphoreHandle_t semaphoreCreateOrUpdateDevice;
SemaphoreHandle_t semaphoreBLEOperation;
QueueHandle_t BLEQueue;

# include <NimBLEAdvertisedDevice.h>
# include <NimBLEDevice.h>
# include <NimBLEScan.h>
# include <NimBLEUtils.h>
# include <decoder.h>
# include <driver/adc.h>
# include <esp_bt.h>
# include <esp_bt_main.h>
# include <esp_wifi.h>

# include <atomic>

# include "ZgatewayBLEConnect.h"
# include "soc/timer_group_reg.h"
# include "soc/timer_group_struct.h"

using namespace std;


BTConfig_s BTConfig;

TheengsDecoder decoder;

static TaskHandle_t xCoreTaskHandle;
static TaskHandle_t xProcBLETaskHandle;

struct decompose {
  int start;
  int len;
  bool reverse;
};

vector<BLEAction> BLEactions;

vector<BLEdevice*> devices;
int newDevices = 0;

static BLEdevice NO_BT_DEVICE_FOUND = {{0},
                                       0,
                                       false,
                                       false,
                                       false,
                                       false,
                                       TheengsDecoder::BLE_ID_NUM::UNKNOWN_MODEL};
static bool oneWhite = false;

void BTConfig_init() {
  BTConfig.bleConnect = AttemptBLEConnect;
  BTConfig.BLEinterval = TimeBtwRead;
  BTConfig.adaptiveScan = AdaptiveBLEScan;
  BTConfig.intervalActiveScan = TimeBtwActive;
  BTConfig.intervalConnect = TimeBtwConnect;
  BTConfig.scanDuration = Scan_duration;
  BTConfig.pubOnlySensors = PublishOnlySensors;
  BTConfig.pubRandomMACs = PublishRandomMACs;
  BTConfig.presenceEnable = HassPresence;
  BTConfig.presenceTopic = subjectHomePresence;
  BTConfig.presenceUseBeaconUuid = useBeaconUuidForPresence;
  BTConfig.minRssi = MinimumRSSI;
  BTConfig.extDecoderEnable = UseExtDecoder;
  BTConfig.extDecoderTopic = MQTTDecodeTopic;
  BTConfig.filterConnectable = BLE_FILTER_CONNECTABLE;
  BTConfig.pubAdvData = pubBLEAdvData;
  BTConfig.pubBeaconUuidForTopic = useBeaconUuidForTopic;
  BTConfig.ignoreWBlist = false;
  BTConfig.presenceAwayTimer = PresenceAwayTimer;
  BTConfig.movingTimer = MovingTimer;
  BTConfig.forcePassiveScan = false;
  BTConfig.enabled = true;
}

unsigned long timeBetweenConnect = 0;
unsigned long timeBetweenActive = 0;

String stateBTMeasures(bool start) {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject jo = jsonBuffer.to<JsonObject>();
  jo["bleconnect"] = BTConfig.bleConnect;
  jo["interval"] = BTConfig.BLEinterval;
  jo["adaptivescan"] = BTConfig.adaptiveScan;
  jo["intervalacts"] = BTConfig.intervalActiveScan;
  jo["intervalcnct"] = BTConfig.intervalConnect;
  jo["scanduration"] = BTConfig.scanDuration;
  jo["onlysensors"] = BTConfig.pubOnlySensors;
  jo["randommacs"] = BTConfig.pubRandomMACs;
  jo["hasspresence"] = BTConfig.presenceEnable;
  jo["prestopic"] = BTConfig.presenceTopic;
  jo["presuseuuid"] = BTConfig.presenceUseBeaconUuid;
  jo["minrssi"] = -abs(BTConfig.minRssi);
  jo["extDecoderEnable"] = BTConfig.extDecoderEnable;
  jo["extDecoderTopic"] = BTConfig.extDecoderTopic;
  jo["filterConnectable"] = BTConfig.filterConnectable;
  jo["pubadvdata"] = BTConfig.pubAdvData;
  jo["pubuuid4topic"] = BTConfig.pubBeaconUuidForTopic;
  jo["ignoreWBlist"] = BTConfig.ignoreWBlist;
  jo["presenceawaytimer"] = BTConfig.presenceAwayTimer;
  jo["movingtimer"] = BTConfig.movingTimer;
  jo["forcepscn"] = BTConfig.forcePassiveScan;
  jo["tskstck"] = uxTaskGetStackHighWaterMark(xProcBLETaskHandle);
  jo["crstck"] = uxTaskGetStackHighWaterMark(xCoreTaskHandle);
  jo["enabled"] = BTConfig.enabled;
  jo["scnct"] = scanCount;

  if (start) {
    Log.notice(F("BT sys: "));
    serializeJsonPretty(jsonBuffer, Serial);
    Serial.println();
    return "";
  }
  String output;
  serializeJson(jo, output);
  jo["origin"] = subjectBTtoMQTT;
  handleJsonEnqueue(jo, QueueSemaphoreTimeOutTask);
  return (output);
}

void BTConfig_fromJson(JsonObject& BTdata, bool startup = false) {

  Config_update(BTdata, "bleconnect", BTConfig.bleConnect);

  if (startup == false) {
    if (BTdata.containsKey("hasspresence") && BTdata["hasspresence"] == false && BTConfig.presenceEnable == true) {
      BTdata["adaptivescan"] = true;
# ifdef ZmqttDiscovery

      eraseTopic("number", (char*)getUniqueId("presenceawaytimer", "").c_str());
# endif
    } else if (BTdata.containsKey("hasspresence") && BTdata["hasspresence"] == true && BTConfig.presenceEnable == false) {
      BTdata["adaptivescan"] = false;
    }

    if (BTdata.containsKey("adaptivescan") && BTdata["adaptivescan"] == false && BTConfig.adaptiveScan == true) {
      BTdata["interval"] = MinTimeBtwScan;
      BTdata["intervalacts"] = MinTimeBtwScan;
      BTdata["scanduration"] = MinScanDuration;
    } else if (BTdata.containsKey("adaptivescan") && BTdata["adaptivescan"] == true && BTConfig.adaptiveScan == false) {
      BTdata["interval"] = TimeBtwRead;
      BTdata["intervalacts"] = TimeBtwActive;
      BTdata["scanduration"] = Scan_duration;
# ifdef ZmqttDiscovery

      eraseTopic("number", (char*)getUniqueId("interval", "").c_str());
      eraseTopic("number", (char*)getUniqueId("intervalacts", "").c_str());
# endif
    }

    if (BTdata.containsKey("enabled") && BTdata["enabled"] == false && BTConfig.enabled == true) {
      stopProcessing();
    } else if (BTdata.containsKey("enabled") && BTdata["enabled"] == true && BTConfig.enabled == false) {
      BTProcessLock = false;
      setupBTTasksAndBLE();
    }
  }
  Config_update(BTdata, "adaptivescan", BTConfig.adaptiveScan);

  Config_update(BTdata, "hasspresence", BTConfig.presenceEnable);
# ifdef ZmqttDiscovery

  btScanParametersDiscovery();
  btPresenceParametersDiscovery();
# endif


  if (BTdata.containsKey("interval") && BTdata["interval"] != 0)
    Config_update(BTdata, "interval", BTConfig.BLEinterval);

  Config_update(BTdata, "intervalacts", BTConfig.intervalActiveScan);

  Config_update(BTdata, "intervalcnct", BTConfig.intervalConnect);

  Config_update(BTdata, "scanduration", BTConfig.scanDuration);

  Config_update(BTdata, "onlysensors", BTConfig.pubOnlySensors);

  Config_update(BTdata, "randommacs", BTConfig.pubRandomMACs);

  Config_update(BTdata, "prestopic", BTConfig.presenceTopic);

  Config_update(BTdata, "presuseuuid", BTConfig.presenceUseBeaconUuid);

  Config_update(BTdata, "presenceawaytimer", BTConfig.presenceAwayTimer);

  Config_update(BTdata, "movingtimer", BTConfig.movingTimer);

  Config_update(BTdata, "forcepscn", BTConfig.forcePassiveScan);

  Config_update(BTdata, "minrssi", BTConfig.minRssi);

  Config_update(BTdata, "extDecoderEnable", BTConfig.extDecoderEnable);

  Config_update(BTdata, "extDecoderTopic", BTConfig.extDecoderTopic);

  Config_update(BTdata, "filterConnectable", BTConfig.filterConnectable);

  Config_update(BTdata, "pubadvdata", BTConfig.pubAdvData);

  Config_update(BTdata, "pubuuid4topic", BTConfig.pubBeaconUuidForTopic);

  Config_update(BTdata, "ignoreWBlist", (BTConfig.ignoreWBlist));

  Config_update(BTdata, "enabled", BTConfig.enabled);

  stateBTMeasures(startup);

  if (BTdata.containsKey("erase") && BTdata["erase"].as<bool>()) {

    preferences.begin(Gateway_Short_Name, false);
    if (preferences.isKey("BTConfig")) {
      int result = preferences.remove("BTConfig");
      Log.notice(F("BT config erase result: %d" CR), result);
      preferences.end();
      return;
    } else {
      preferences.end();
      Log.notice(F("BT config not found" CR));
    }
  }

  if (BTdata.containsKey("save") && BTdata["save"].as<bool>()) {
    StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject jo = jsonBuffer.to<JsonObject>();
    jo["bleconnect"] = BTConfig.bleConnect;
    jo["interval"] = BTConfig.BLEinterval;
    jo["adaptivescan"] = BTConfig.adaptiveScan;
    jo["intervalacts"] = BTConfig.intervalActiveScan;
    jo["intervalcnct"] = BTConfig.intervalConnect;
    jo["scanduration"] = BTConfig.scanDuration;
    jo["onlysensors"] = BTConfig.pubOnlySensors;
    jo["randommacs"] = BTConfig.pubRandomMACs;
    jo["hasspresence"] = BTConfig.presenceEnable;
    jo["prestopic"] = BTConfig.presenceTopic;
    jo["presuseuuid"] = BTConfig.presenceUseBeaconUuid;
    jo["minrssi"] = -abs(BTConfig.minRssi);
    jo["extDecoderEnable"] = BTConfig.extDecoderEnable;
    jo["extDecoderTopic"] = BTConfig.extDecoderTopic;
    jo["filterConnectable"] = BTConfig.filterConnectable;
    jo["pubadvdata"] = BTConfig.pubAdvData;
    jo["pubuuid4topic"] = BTConfig.pubBeaconUuidForTopic;
    jo["ignoreWBlist"] = BTConfig.ignoreWBlist;
    jo["presenceawaytimer"] = BTConfig.presenceAwayTimer;
    jo["movingtimer"] = BTConfig.movingTimer;
    jo["forcepscn"] = BTConfig.forcePassiveScan;
    jo["enabled"] = BTConfig.enabled;

    String conf = "";
    serializeJson(jsonBuffer, conf);
    preferences.begin(Gateway_Short_Name, false);
    int result = preferences.putString("BTConfig", conf);
    preferences.end();
    Log.notice(F("BT config save: %s, result: %d" CR), conf.c_str(), result);
  }
}

void BTConfig_load() {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  preferences.begin(Gateway_Short_Name, true);
  if (preferences.isKey("BTConfig")) {
    auto error = deserializeJson(jsonBuffer, preferences.getString("BTConfig", "{}"));
    preferences.end();
    Log.notice(F("BT config loaded" CR));
    if (error) {
      Log.error(F("BT config deserialization failed: %s, buffer capacity: %u" CR), error.c_str(), jsonBuffer.capacity());
      return;
    }
    if (jsonBuffer.isNull()) {
      Log.warning(F("BT config is null" CR));
      return;
    }
    JsonObject jo = jsonBuffer.as<JsonObject>();
    BTConfig_fromJson(jo, true);
    Log.notice(F("BT config loaded" CR));
  } else {
    preferences.end();
    Log.notice(F("BT config not found" CR));
  }
}

void PublishDeviceData(JsonObject& BLEdata);

atomic_int forceBTScan;

void createOrUpdateDevice(const char* mac, uint8_t flags, int model, int mac_type = 0);

BLEdevice* getDeviceByMac(const char* mac);
BLEdevice* getDeviceByMac(const char* mac) {
  Log.trace(F("getDeviceByMac %s" CR), mac);

  for (vector<BLEdevice*>::iterator it = devices.begin(); it != devices.end(); ++it) {
    if ((strcmp((*it)->macAdr, mac) == 0)) {
      return *it;
    }
  }
  return &NO_BT_DEVICE_FOUND;
}

bool updateWorB(JsonObject& BTdata, bool isWhite) {
  Log.trace(F("update WorB" CR));
  const char* jsonKey = isWhite ? "white-list" : "black-list";

  int size = BTdata[jsonKey].size();
  if (size == 0)
    return false;

  for (int i = 0; i < size; i++) {
    const char* mac = BTdata[jsonKey][i];
    createOrUpdateDevice(mac, (isWhite ? device_flags_isWhiteL : device_flags_isBlackL),
                         TheengsDecoder::BLE_ID_NUM::UNKNOWN_MODEL);
  }

  return true;
}

void createOrUpdateDevice(const char* mac, uint8_t flags, int model, int mac_type) {
  if (xSemaphoreTake(semaphoreCreateOrUpdateDevice, pdMS_TO_TICKS(30000)) == pdFALSE) {
    Log.error(F("Semaphore NOT taken" CR));
    return;
  }

  BLEdevice* device = getDeviceByMac(mac);
  if (device == &NO_BT_DEVICE_FOUND) {
    Log.trace(F("add %s" CR), mac);

    device = new BLEdevice();
    strcpy(device->macAdr, mac);
    device->isDisc = flags & device_flags_isDisc;
    device->isWhtL = flags & device_flags_isWhiteL;
    device->isBlkL = flags & device_flags_isBlackL;
    device->connect = flags & device_flags_connect;
    device->macType = mac_type;
    device->sensorModel_id = model;
    device->lastUpdate = millis();
    devices.push_back(device);
    newDevices++;
  } else {
    Log.trace(F("update %s" CR), mac);
    device->lastUpdate = millis();
    device->macType = mac_type;

    if (flags & device_flags_isDisc) {
      device->isDisc = true;
    }

    if (flags & device_flags_connect) {
      device->connect = true;
    }

    if (model != TheengsDecoder::BLE_ID_NUM::UNKNOWN_MODEL) {
      device->sensorModel_id = model;
    }

    if (flags & device_flags_isWhiteL || flags & device_flags_isBlackL) {
      device->isWhtL = flags & device_flags_isWhiteL;
      device->isBlkL = flags & device_flags_isBlackL;
    }
  }


  oneWhite = oneWhite || device->isWhtL;

  xSemaphoreGive(semaphoreCreateOrUpdateDevice);
}

void updateDevicesStatus() {
  for (vector<BLEdevice*>::iterator it = devices.begin(); it != devices.end(); ++it) {
    BLEdevice* p = *it;
    unsigned long now = millis();

    bool isTracker = false;
    std::string tag = decoder.getTheengAttribute(p->sensorModel_id, "tag");
    if (tag.length() >= 4) {
      isTracker = checkIfIsTracker(tag[3]);
    }

    if (isTracker) {
      if ((p->lastUpdate != 0) && (p->lastUpdate < (now - BTConfig.presenceAwayTimer) && (now > BTConfig.presenceAwayTimer)) &&
          (BTConfig.ignoreWBlist || ((!oneWhite || isWhite(p)) && !isBlack(p)))) {
        StaticJsonDocument<JSON_MSG_BUFFER> BLEdataBuffer;
        JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
        BLEdata["id"] = p->macAdr;
        BLEdata["state"] = "offline";
        buildTopicFromId(BLEdata, subjectBTtoMQTT);
        handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);

        p->lastUpdate = 0;
      }
    }

    if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::BC08) {
      if ((p->lastUpdate != 0) && (p->lastUpdate < (now - BTConfig.movingTimer) && (now > BTConfig.movingTimer)) &&
          (BTConfig.ignoreWBlist || ((!oneWhite || isWhite(p)) && !isBlack(p)))) {
        StaticJsonDocument<JSON_MSG_BUFFER> BLEdataBuffer;
        JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
        BLEdata["id"] = p->macAdr;
        BLEdata["state"] = "offline";
        buildTopicFromId(BLEdata, subjectBTtoMQTT);
        handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);

        p->lastUpdate = 0;
      }
    }
  }
}

void dumpDevices() {
# if LOG_LEVEL > LOG_LEVEL_NOTICE
  for (vector<BLEdevice*>::iterator it = devices.begin(); it != devices.end(); ++it) {
    BLEdevice* p = *it;
    Log.trace(F("macAdr %s" CR), p->macAdr);
    Log.trace(F("macType %d" CR), p->macType);
    Log.trace(F("isDisc %d" CR), p->isDisc);
    Log.trace(F("isWhtL %d" CR), p->isWhtL);
    Log.trace(F("isBlkL %d" CR), p->isBlkL);
    Log.trace(F("connect %d" CR), p->connect);
    Log.trace(F("sensorModel_id %d" CR), p->sensorModel_id);
    Log.trace(F("LastUpdate %u" CR), p->lastUpdate);
  }
# endif
}

void strupp(char* beg) {
  while ((*beg = toupper(*beg)))
    ++beg;
}

# ifdef ZmqttDiscovery
void DT24Discovery(const char* mac, const char* sensorModel_id) {
#define DT24parametersCount 7
  Log.trace(F("DT24Discovery" CR));
  const char* DT24sensor[DT24parametersCount][9] = {
      {"sensor", "volt", mac, "voltage", jsonVolt, "", "", "V", stateClassMeasurement},
      {"sensor", "amp", mac, "current", jsonCurrent, "", "", "A", stateClassMeasurement},
      {"sensor", "watt", mac, "power", jsonPower, "", "", "W", stateClassMeasurement},
      {"sensor", "watt-hour", mac, "power", jsonEnergy, "", "", "kWh", stateClassMeasurement},
      {"sensor", "price", mac, "", jsonMsg, "", "", "", stateClassNone},
      {"sensor", "temp", mac, "temperature", jsonTempc, "", "", "°C", stateClassMeasurement},
      {"binary_sensor", "inUse", mac, "power", jsonInuse, "", "", "", stateClassNone}

  };

  createDiscoveryFromList(mac, DT24sensor, DT24parametersCount, "DT24", "ATorch", sensorModel_id);
}

void BM2Discovery(const char* mac, const char* sensorModel_id) {
#define BM2parametersCount 2
  Log.trace(F("BM2Discovery" CR));
  const char* BM2sensor[BM2parametersCount][9] = {
      {"sensor", "volt", mac, "voltage", jsonVoltBM2, "", "", "V", stateClassMeasurement},
      {"sensor", "batt", mac, "battery", jsonBatt, "", "", "%", stateClassMeasurement}

  };

  createDiscoveryFromList(mac, BM2sensor, BM2parametersCount, "BM2", "Generic", sensorModel_id);
}

void LYWSD03MMCDiscovery(const char* mac, const char* sensorModel) {
#define LYWSD03MMCparametersCount 4
  Log.trace(F("LYWSD03MMCDiscovery" CR));
  const char* LYWSD03MMCsensor[LYWSD03MMCparametersCount][9] = {
      {"sensor", "batt", mac, "battery", jsonBatt, "", "", "%", stateClassMeasurement},
      {"sensor", "volt", mac, "", jsonVolt, "", "", "V", stateClassMeasurement},
      {"sensor", "temp", mac, "temperature", jsonTempc, "", "", "°C", stateClassMeasurement},
      {"sensor", "hum", mac, "humidity", jsonHum, "", "", "%", stateClassMeasurement}

  };

  createDiscoveryFromList(mac, LYWSD03MMCsensor, LYWSD03MMCparametersCount, "LYWSD03MMC", "Xiaomi", sensorModel);
}

void MHO_C401Discovery(const char* mac, const char* sensorModel) {
#define MHO_C401parametersCount 4
  Log.trace(F("MHO_C401Discovery" CR));
  const char* MHO_C401sensor[MHO_C401parametersCount][9] = {
      {"sensor", "batt", mac, "battery", jsonBatt, "", "", "%", stateClassMeasurement},
      {"sensor", "volt", mac, "", jsonVolt, "", "", "V", stateClassMeasurement},
      {"sensor", "temp", mac, "temperature", jsonTempc, "", "", "°C", stateClassMeasurement},
      {"sensor", "hum", mac, "humidity", jsonHum, "", "", "%", stateClassMeasurement}

  };

  createDiscoveryFromList(mac, MHO_C401sensor, MHO_C401parametersCount, "MHO_C401", "Xiaomi", sensorModel);
}

void HHCCJCY01HHCCDiscovery(const char* mac, const char* sensorModel) {
#define HHCCJCY01HHCCparametersCount 5
  Log.trace(F("HHCCJCY01HHCCDiscovery" CR));
  const char* HHCCJCY01HHCCsensor[HHCCJCY01HHCCparametersCount][9] = {
      {"sensor", "batt", mac, "battery", jsonBatt, "", "", "%", stateClassMeasurement},
      {"sensor", "temp", mac, "temperature", jsonTempc, "", "", "°C", stateClassMeasurement},
      {"sensor", "lux", mac, "illuminance", jsonLux, "", "", "lx", stateClassMeasurement},
      {"sensor", "fer", mac, "", jsonFer, "", "", "µS/cm", stateClassMeasurement},
      {"sensor", "moi", mac, "", jsonMoi, "", "", "%", stateClassMeasurement}

  };

  createDiscoveryFromList(mac, HHCCJCY01HHCCsensor, HHCCJCY01HHCCparametersCount, "HHCCJCY01HHCC", "Xiaomi", sensorModel);
}

void XMWSDJ04MMCDiscovery(const char* mac, const char* sensorModel) {
#define XMWSDJ04MMCparametersCount 4
  Log.trace(F("XMWSDJ04MMCDiscovery" CR));
  const char* XMWSDJ04MMCsensor[XMWSDJ04MMCparametersCount][9] = {
      {"sensor", "batt", mac, "battery", jsonBatt, "", "", "%", stateClassMeasurement},
      {"sensor", "volt", mac, "", jsonVolt, "", "", "V", stateClassMeasurement},
      {"sensor", "temp", mac, "temperature", jsonTempc, "", "", "°C", stateClassMeasurement},
      {"sensor", "hum", mac, "humidity", jsonHum, "", "", "%", stateClassMeasurement}

  };

  createDiscoveryFromList(mac, XMWSDJ04MMCsensor, XMWSDJ04MMCparametersCount, "XMWSDJ04MMC", "Xiaomi", sensorModel);
}

# else
void LYWSD03MMCDiscovery(const char* mac, const char* sensorModel) {}
void MHO_C401Discovery(const char* mac, const char* sensorModel) {}
void HHCCJCY01HHCCDiscovery(const char* mac, const char* sensorModel) {}
void DT24Discovery(const char* mac, const char* sensorModel_id) {}
void BM2Discovery(const char* mac, const char* sensorModel_id) {}
void XMWSDJ04MMCDiscovery(const char* mac, const char* sensorModel_id) {}
# endif
# 566 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayBT.ino"
static int taskCore = 0;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice* advertisedDevice) {
    BLEAdvertisedDevice* ad = new BLEAdvertisedDevice(*advertisedDevice);
    if (xQueueSend(BLEQueue, &ad, 0) != pdTRUE) {
      Log.error(F("BLEQueue full" CR));
      delete (ad);
    }
  }
};

std::string convertServiceData(std::string deviceServiceData) {
  int serviceDataLength = (int)deviceServiceData.length();
  char spr[2 * serviceDataLength + 1];
  for (int i = 0; i < serviceDataLength; i++) sprintf(spr + 2 * i, "%.2x", (unsigned char)deviceServiceData[i]);
  spr[2 * serviceDataLength] = 0;
  Log.trace(F("Converted service data (%d) to %s" CR), serviceDataLength, spr);
  return spr;
}

bool checkIfIsTracker(char ch) {
  uint8_t data = 0;
  if (ch >= '0' && ch <= '9')
    data = ch - '0';
  else if (ch >= 'a' && ch <= 'f')
    data = 10 + (ch - 'a');

  if (((data >> 3) & 0x01) == 1) {
    Log.trace(F("Is Device Tracker" CR));
    return true;
  } else {
    return false;
  }
}

void procBLETask(void* pvParameters) {
  BLEAdvertisedDevice* advertisedDevice = nullptr;

  for (;;) {
    xQueueReceive(BLEQueue, &advertisedDevice, portMAX_DELAY);


    if (!BTProcessLock) {
      Log.trace(F("Creating BLE buffer" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> BLEdataBuffer;
      JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
      std::string mac_address = advertisedDevice->getAddress().toString();

      for (char& c : mac_address) {
        c = std::toupper(c);
      }

      BLEdata["id"] = mac_address;
      BLEdata["mac_type"] = advertisedDevice->getAddress().getType();
      BLEdata["adv_type"] = advertisedDevice->getAdvType();
      Log.notice(F("Device detected: %s" CR), BLEdata["id"].as<const char*>());
      BLEdevice* device = getDeviceByMac(BLEdata["id"].as<const char*>());

      if (BTConfig.filterConnectable && device->connect) {
        Log.notice(F("Filtered connectable device" CR));
        delete (advertisedDevice);
        continue;
      }

      if (BTConfig.ignoreWBlist || ((!oneWhite || isWhite(device)) && !isBlack(device))) {
        if (advertisedDevice->haveName())
          BLEdata["name"] = (char*)advertisedDevice->getName().c_str();
        if (advertisedDevice->haveManufacturerData()) {
          char* manufacturerdata = BLEUtils::buildHexData(NULL, (uint8_t*)advertisedDevice->getManufacturerData().data(), advertisedDevice->getManufacturerData().length());
          BLEdata["manufacturerdata"] = manufacturerdata;
          free(manufacturerdata);
        }
        if (advertisedDevice->haveRSSI())
          BLEdata["rssi"] = (int)advertisedDevice->getRSSI();
        if (advertisedDevice->haveTXPower())
          BLEdata["txpower"] = (int8_t)advertisedDevice->getTXPower();
        if (advertisedDevice->haveRSSI() && BTConfig.presenceEnable) {
          hass_presence(BLEdata);
        }
        if (advertisedDevice->haveServiceData()) {
          int serviceDataCount = advertisedDevice->getServiceDataCount();
          Log.trace(F("Get services data number: %d" CR), serviceDataCount);
          for (int j = 0; j < serviceDataCount; j++) {
            StaticJsonDocument<JSON_MSG_BUFFER> BLEdataBufferTemp;
            JsonObject BLEdataTemp = BLEdataBufferTemp.to<JsonObject>();
            BLEdataBufferTemp = BLEdataBuffer;
            std::string service_data = convertServiceData(advertisedDevice->getServiceData(j));
            Log.trace(F("Service data: %s" CR), service_data.c_str());
            std::string serviceDatauuid = advertisedDevice->getServiceDataUUID(j).toString();
            Log.trace(F("Service data UUID: %s" CR), (char*)serviceDatauuid.c_str());
            BLEdataTemp["servicedata"] = (char*)service_data.c_str();
            BLEdataTemp["servicedatauuid"] = (char*)serviceDatauuid.c_str();
            PublishDeviceData(BLEdataTemp);
          }
        } else {
          PublishDeviceData(BLEdata);
        }
      } else {
        Log.trace(F("Filtered MAC device" CR));
      }
      updateDevicesStatus();
    }
    delete (advertisedDevice);
  }
}




void BLEscan() {

  while (uxQueueMessagesWaiting(BLEQueue) || queueLength != 0) {
    delay(1);
  }
  Log.notice(F("Scan begin" CR));
  BLEScan* pBLEScan = BLEDevice::getScan();
  MyAdvertisedDeviceCallbacks myCallbacks;
  pBLEScan->setAdvertisedDeviceCallbacks(&myCallbacks);
  if ((millis() > (timeBetweenActive + BTConfig.intervalActiveScan) || BTConfig.intervalActiveScan == BTConfig.BLEinterval) && !BTConfig.forcePassiveScan) {
    pBLEScan->setActiveScan(true);
    timeBetweenActive = millis();
  } else {
    pBLEScan->setActiveScan(false);
  }
  pBLEScan->setInterval(BLEScanInterval);
  pBLEScan->setWindow(BLEScanWindow);
  BLEScanResults foundDevices = pBLEScan->start(BTConfig.scanDuration / 1000, false);
  if (foundDevices.getCount())
    scanCount++;
  Log.notice(F("Found %d devices, scan number %d end" CR), foundDevices.getCount(), scanCount);
  Log.trace(F("Process BLE stack free: %u" CR), uxTaskGetStackHighWaterMark(xProcBLETaskHandle));
}




void BLEconnect() {
  if (!BTProcessLock) {
    Log.notice(F("BLE Connect begin" CR));
    do {
      for (vector<BLEdevice*>::iterator it = devices.begin(); it != devices.end(); ++it) {
        BLEdevice* p = *it;
        if (p->connect) {
          Log.trace(F("Model to connect found: %s" CR), p->macAdr);
          NimBLEAddress addr((const char*)p->macAdr, p->macType);
          if (p->sensorModel_id == BLEconectable::id::LYWSD03MMC ||
              p->sensorModel_id == BLEconectable::id::MHO_C401) {
            LYWSD03MMC_connect BLEclient(addr);
            BLEclient.processActions(BLEactions);
            BLEclient.publishData();
          } else if (p->sensorModel_id == BLEconectable::id::DT24_BLE) {
            DT24_connect BLEclient(addr);
            BLEclient.processActions(BLEactions);
            BLEclient.publishData();
          } else if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::BM2) {
            BM2_connect BLEclient(addr);
            BLEclient.processActions(BLEactions);
            BLEclient.publishData();
          } else if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::HHCCJCY01HHCC) {
            HHCCJCY01HHCC_connect BLEclient(addr);
            BLEclient.processActions(BLEactions);
            BLEclient.publishData();
          } else if (p->sensorModel_id == BLEconectable::id::XMWSDJ04MMC) {
            XMWSDJ04MMC_connect BLEclient(addr);
            BLEclient.processActions(BLEactions);
            BLEclient.publishData();
          } else if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::SBS1) {
            SBS1_connect BLEclient(addr);
            BLEclient.processActions(BLEactions);
          } else {
            GENERIC_connect BLEclient(addr);
            if (BLEclient.processActions(BLEactions)) {


              p->connect = false;
            }
          }
          if (BLEactions.size() > 0) {
            std::vector<BLEAction> swap;
            for (auto& it : BLEactions) {
              if (!it.complete && --it.ttl) {
                swap.push_back(it);
              } else if (memcmp(it.addr, p->macAdr, sizeof(it.addr)) == 0) {
                if (p->sensorModel_id != BLEconectable::id::DT24_BLE &&
                    p->sensorModel_id != TheengsDecoder::BLE_ID_NUM::HHCCJCY01HHCC &&
                    p->sensorModel_id != BLEconectable::id::LYWSD03MMC &&
                    p->sensorModel_id != TheengsDecoder::BLE_ID_NUM::BM2 &&
                    p->sensorModel_id != BLEconectable::id::MHO_C401 &&
                    p->sensorModel_id != BLEconectable::id::XMWSDJ04MMC) {

                  p->connect = false;
                }
              }
            }
            std::swap(BLEactions, swap);
          }
        }
      }
    } while (BLEactions.size() > 0);
    Log.notice(F("BLE Connect end" CR));
  }
}

void stopProcessing() {
  if (BTConfig.enabled) {
    BTProcessLock = true;

    Log.notice(F("Stopping BLE scan" CR));
    BLEScan* pBLEScan = BLEDevice::getScan();
    if (pBLEScan->isScanning()) {
      pBLEScan->stop();
    }

    if (xSemaphoreTake(semaphoreBLEOperation, pdMS_TO_TICKS(5000)) == pdTRUE) {
      Log.notice(F("Stopping BLE tasks" CR));

      vTaskSuspend(xCoreTaskHandle);
      vTaskDelete(xCoreTaskHandle);
      vTaskSuspend(xProcBLETaskHandle);
      vTaskDelete(xProcBLETaskHandle);
      xSemaphoreGive(semaphoreBLEOperation);
    }
  }
  Log.notice(F("BLE gateway stopped %T, free heap: %d" CR), ESP.getFreeHeap());
}

void coreTask(void* pvParameters) {
  while (true) {
    if (!BTProcessLock) {
      int n = 0;
      while (client.state() != 0 && n <= InitialMQTTConnectionTimeout && !BTProcessLock) {
        n++;
        delay(1000);
      }
      if (!connected) {
        Log.warning(F("MQTT client or Network disconnected no BLE scan" CR));
      } else if (!BTProcessLock) {
        if (xSemaphoreTake(semaphoreBLEOperation, pdMS_TO_TICKS(30000)) == pdTRUE) {
          BLEscan();

          if (millis() > (timeBetweenConnect + BTConfig.intervalConnect) && BTConfig.bleConnect) {
            timeBetweenConnect = millis();
            BLEconnect();
          }

          Log.trace(F("CoreTask stack free: %u" CR), uxTaskGetStackHighWaterMark(xCoreTaskHandle));
          xSemaphoreGive(semaphoreBLEOperation);
        } else {
          Log.error(F("Failed to start scan - BLE busy" CR));
        }
      }
      if (lowpowermode > 0) {
        lowPowerESP32();
        int scan = atomic_exchange_explicit(&forceBTScan, 0, ::memory_order_seq_cst);
        if (scan == 1) BTforceScan();
      } else {
        for (int interval = BTConfig.BLEinterval, waitms; interval > 0; interval -= waitms) {
          int scan = atomic_exchange_explicit(&forceBTScan, 0, ::memory_order_seq_cst);
          if (scan == 1) BTforceScan();
          delay(waitms = interval > 100 ? 100 : interval);
        }
      }
    }
    delay(1);
  }
}

# if DEFAULT_LOW_POWER_MODE != -1
void lowPowerESP32() {
  Log.trace(F("Going to deep sleep for: %l s" CR), (TimeBtwRead / 1000));
  deepSleep(TimeBtwRead * 1000);
}

void deepSleep(uint64_t time_in_us) {
# if defined(ZboardM5STACK) || defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5TOUGH)
  sleepScreen();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SLEEP_BUTTON, LOW);
# endif

  Log.trace(F("Deactivating ESP32 components" CR));
  stopProcessing();
  ProcessLock = true;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  adc_power_off();
#pragma GCC diagnostic pop
  esp_wifi_stop();
  esp_deep_sleep(time_in_us);
}
# else
void lowPowerESP32() {}
# endif

void changelowpowermode(int newLowPowerMode) {
# if DEFAULT_LOW_POWER_MODE != -1
  Log.notice(F("Changing LOW POWER mode to: %d" CR), newLowPowerMode);
# if defined(ZboardM5STACK) || defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5TOUGH)
  if (lowpowermode == 2) {
# ifdef ZboardM5STACK
    M5.Lcd.wakeup();
# endif
# if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5TOUGH)
    M5.Axp.SetLDO2(true);
    M5.Lcd.begin();
# endif
  }
  char lpm[2];
  sprintf(lpm, "%d", newLowPowerMode);
  M5Print("Changing LOW POWER mode to:", lpm, "");
# endif
  lowpowermode = newLowPowerMode;
  preferences.begin(Gateway_Short_Name, false);
  int result = preferences.putUInt("lowpowermode", lowpowermode);
  Log.notice(F("BT LPM config save result: %d" CR), result);
  preferences.end();

  stateMeasures();
# endif
}

void setupBTTasksAndBLE() {
  BLEDevice::setScanDuplicateCacheSize(BLEScanDuplicateCacheSize);
  BLEDevice::init("");

  xTaskCreatePinnedToCore(
      procBLETask,
      "procBLETask",
      8500,
      NULL,
      2,
      &xProcBLETaskHandle,
      1);


  xTaskCreatePinnedToCore(
      coreTask,
      "coreTask",
      5120,
      NULL,
      1,
      &xCoreTaskHandle,
      taskCore);
}

void setupBT() {
  BTConfig_init();
  BTConfig_load();
  Log.notice(F("BLE scans interval: %d" CR), BTConfig.BLEinterval);
  Log.notice(F("BLE connects interval: %d" CR), BTConfig.intervalConnect);
  Log.notice(F("BLE scan duration: %d" CR), BTConfig.scanDuration);
  Log.notice(F("Publishing only BLE sensors: %T" CR), BTConfig.pubOnlySensors);
  Log.notice(F("Publishing random MAC devices: %T" CR), BTConfig.pubRandomMACs);
  Log.notice(F("Adaptive BLE scan: %T" CR), BTConfig.adaptiveScan);
  Log.notice(F("Active BLE scan interval: %d" CR), BTConfig.intervalActiveScan);
  Log.notice(F("minrssi: %d" CR), -abs(BTConfig.minRssi));
  Log.notice(F("Low Power Mode: %d" CR), lowpowermode);
  Log.notice(F("Presence Away Timer: %d" CR), BTConfig.presenceAwayTimer);
  Log.notice(F("Moving Timer: %d" CR), BTConfig.movingTimer);
  Log.notice(F("Force passive scan: %T" CR), BTConfig.forcePassiveScan);
  Log.notice(F("Enabled BLE: %T" CR), BTConfig.enabled);

  atomic_init(&forceBTScan, 0);

  semaphoreCreateOrUpdateDevice = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphoreCreateOrUpdateDevice);

  semaphoreBLEOperation = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphoreBLEOperation);

  BLEQueue = xQueueCreate(QueueSize, sizeof(NimBLEAdvertisedDevice*));
  if (BTConfig.enabled) {
    setupBTTasksAndBLE();
    Log.notice(F("ZgatewayBT multicore ESP32 setup done" CR));
  } else {
    Log.notice(F("ZgatewayBT multicore ESP32 setup disabled" CR));
  }
}

bool BTtoMQTT() {
  BLEscan();
  return true;
}

boolean valid_service_data(const char* data, int size) {
  for (int i = 0; i < size; ++i) {
    if (data[i] != 48)
      return true;
  }
  return false;
}

# ifdef ZmqttDiscovery


void launchBTDiscovery(bool overrideDiscovery) {
  if (!overrideDiscovery && newDevices == 0)
    return;
  if (xSemaphoreTake(semaphoreCreateOrUpdateDevice, pdMS_TO_TICKS(QueueSemaphoreTimeOutTask)) == pdFALSE) {
    Log.error(F("Semaphore NOT taken" CR));
    return;
  }
  newDevices = 0;
  vector<BLEdevice*> localDevices = devices;
  xSemaphoreGive(semaphoreCreateOrUpdateDevice);
  for (vector<BLEdevice*>::iterator it = localDevices.begin(); it != localDevices.end(); ++it) {
    BLEdevice* p = *it;
    Log.trace(F("Device mac %s" CR), p->macAdr);

    if (overrideDiscovery || !isDiscovered(p)) {
      String macWOdots = String(p->macAdr);
      macWOdots.replace(":", "");
      if (p->sensorModel_id >= 0) {
        Log.trace(F("Looking for Model_id: %d" CR), p->sensorModel_id);
        std::string properties = decoder.getTheengProperties(p->sensorModel_id);
        Log.trace(F("properties: %s" CR), properties.c_str());
        std::string brand = decoder.getTheengAttribute(p->sensorModel_id, "brand");
        std::string model = decoder.getTheengAttribute(p->sensorModel_id, "model");
        std::string model_id = decoder.getTheengAttribute(p->sensorModel_id, "model_id");


        bool isTracker = false;
        std::string tag = decoder.getTheengAttribute(p->sensorModel_id, "tag");
        if (tag.length() >= 4) {
          isTracker = checkIfIsTracker(tag[3]);
        }

        String discovery_topic = String(subjectBTtoMQTT) + "/" + macWOdots;
        if (!BTConfig.extDecoderEnable &&
            p->sensorModel_id > TheengsDecoder::BLE_ID_NUM::UNKNOWN_MODEL &&
            p->sensorModel_id < TheengsDecoder::BLE_ID_NUM::BLE_ID_MAX &&
            p->sensorModel_id != TheengsDecoder::BLE_ID_NUM::HHCCJCY01HHCC && p->sensorModel_id != TheengsDecoder::BLE_ID_NUM::BM2) {
          if (isTracker) {
            String tracker_name = String(model_id.c_str()) + "-tracker";
            String tracker_id = macWOdots + "-tracker";
            createDiscovery("device_tracker",
                            discovery_topic.c_str(), tracker_name.c_str(), tracker_id.c_str(),
                            will_Topic, "occupancy", "{% if value_json.get('rssi') -%}home{%- else -%}not_home{%- endif %}",
                            "", "", "",
                            0, "", "", false, "",
                            model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                            stateClassNone);
          }
          if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::BC08) {
            String sensor_name = String(model_id.c_str()) + "-moving";
            String sensor_id = macWOdots + "-moving";
            createDiscovery("binary_sensor",
                            discovery_topic.c_str(), sensor_name.c_str(), sensor_id.c_str(),
                            will_Topic, "moving", "{% if value_json.get('accx') -%}on{%- else -%}off{%- endif %}",
                            "on", "off", "",
                            0, "", "", false, "",
                            model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                            stateClassNone);
          }
          if (!properties.empty()) {
            StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
            auto error = deserializeJson(jsonBuffer, properties);
            if (error) {
              if (jsonBuffer.overflowed()) {


                Log.error(F("JSON deserialization of Theengs properties overflowed (error %s), buffer capacity: %u. Program might crash. Properties json: %s" CR),
                          error.c_str(), jsonBuffer.capacity(), properties.c_str());
              } else {
                Log.error(F("JSON deserialization of Theengs properties errored: %" CR),
                          error.c_str());
              }
            }
            for (JsonPair prop : jsonBuffer["properties"].as<JsonObject>()) {
              Log.trace(F("Key: %s"), prop.key().c_str());
              Log.trace(F("Unit: %s"), prop.value()["unit"].as<const char*>());
              Log.trace(F("Name: %s"), prop.value()["name"].as<const char*>());
              String entity_name = String(model_id.c_str()) + "-" + String(prop.key().c_str());
              String unique_id = macWOdots + "-" + String(prop.key().c_str());
              String value_template = "{{ value_json." + String(prop.key().c_str()) + " | is_defined }}";
              if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::SBS1 && strcmp(prop.key().c_str(), "state") != 0) {
                String payload_on = "{\"SBS1\":\"on\",\"mac\":\"" + String(p->macAdr) + "\"}";
                String payload_off = "{\"SBS1\":\"off\",\"mac\":\"" + String(p->macAdr) + "\"}";
                createDiscovery("switch",
                                discovery_topic.c_str(), entity_name.c_str(), unique_id.c_str(),
                                will_Topic, "switch", value_template.c_str(),
                                payload_on.c_str(), payload_off.c_str(), "", 0,
                                Gateway_AnnouncementMsg, will_Message, false, subjectMQTTtoBTset,
                                model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                                stateClassNone, "off", "on");
              } else if ((p->sensorModel_id == TheengsDecoder::XMTZC04HMKG || p->sensorModel_id == TheengsDecoder::XMTZC04HMLB || p->sensorModel_id == TheengsDecoder::XMTZC05HMKG || p->sensorModel_id == TheengsDecoder::XMTZC05HMLB) &&
                         strcmp(prop.key().c_str(), "weighing_mode") == 0) {
                createDiscovery("sensor",
                                discovery_topic.c_str(), entity_name.c_str(), unique_id.c_str(),
                                will_Topic, "enum", value_template.c_str(),
                                "", "", prop.value()["unit"],
                                0, "", "", false, "",
                                model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                                stateClassMeasurement, nullptr, nullptr, "[\"person\",\"object\"]");
              } else if ((p->sensorModel_id == TheengsDecoder::XMTZC04HMKG || p->sensorModel_id == TheengsDecoder::XMTZC04HMLB || p->sensorModel_id == TheengsDecoder::XMTZC05HMKG || p->sensorModel_id == TheengsDecoder::XMTZC05HMLB) &&
                         strcmp(prop.key().c_str(), "unit") == 0) {
                createDiscovery("sensor",
                                discovery_topic.c_str(), entity_name.c_str(), unique_id.c_str(),
                                will_Topic, "enum", value_template.c_str(),
                                "", "", prop.value()["unit"],
                                0, "", "", false, "",
                                model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                                stateClassMeasurement, nullptr, nullptr, "[\"lb\",\"kg\",\"jin\"]");
              } else if (strcmp(prop.value()["unit"], "string") == 0 && strcmp(prop.key().c_str(), "mac") != 0) {
                createDiscovery("sensor",
                                discovery_topic.c_str(), entity_name.c_str(), unique_id.c_str(),
                                will_Topic, prop.value()["name"], value_template.c_str(),
                                "", "", "",
                                0, "", "", false, "",
                                model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                                stateClassNone);
              } else if (p->sensorModel_id == TheengsDecoder::MUE4094RT && strcmp(prop.value()["unit"], "status") == 0) {
                createDiscovery("binary_sensor",
                                discovery_topic.c_str(), entity_name.c_str(), unique_id.c_str(),
                                will_Topic, prop.value()["name"], value_template.c_str(),
                                "True", "False", "",
                                BTConfig.presenceAwayTimer / 1000, "", "", false, "",
                                model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                                stateClassNone);
              } else if (strcmp(prop.value()["unit"], "status") == 0) {
                createDiscovery("binary_sensor",
                                discovery_topic.c_str(), entity_name.c_str(), unique_id.c_str(),
                                will_Topic, prop.value()["name"], value_template.c_str(),
                                "True", "False", "",
                                0, "", "", false, "",
                                model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                                stateClassNone);
              } else if (strcmp(prop.key().c_str(), "device") != 0 && strcmp(prop.key().c_str(), "mac") != 0) {
                createDiscovery("sensor",
                                discovery_topic.c_str(), entity_name.c_str(), unique_id.c_str(),
                                will_Topic, prop.value()["name"], value_template.c_str(),
                                "", "", prop.value()["unit"],
                                0, "", "", false, "",
                                model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                                stateClassMeasurement);
              }
            }
          }
        } else {
          if (p->sensorModel_id > BLEconectable::id::MIN &&
                  p->sensorModel_id < BLEconectable::id::MAX ||
              p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::HHCCJCY01HHCC || p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::BM2) {

            if (p->sensorModel_id == BLEconectable::id::DT24_BLE) {
              DT24Discovery(macWOdots.c_str(), "DT24-BLE");
            }
            if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::BM2) {

              BM2Discovery(macWOdots.c_str(), "BM2");

              String tracker_id = macWOdots + "-tracker";
              createDiscovery("device_tracker",
                              discovery_topic.c_str(), "BM2-tracker", tracker_id.c_str(),
                              will_Topic, "occupancy", "{% if value_json.get('rssi') -%}home{%- else -%}not_home{%- endif %}",
                              "", "", "",
                              0, "", "", false, "",
                              model.c_str(), brand.c_str(), model_id.c_str(), macWOdots.c_str(), false,
                              stateClassNone);
            }
            if (p->sensorModel_id == BLEconectable::id::LYWSD03MMC) {
              LYWSD03MMCDiscovery(macWOdots.c_str(), "LYWSD03MMC");
            }
            if (p->sensorModel_id == BLEconectable::id::MHO_C401) {
              MHO_C401Discovery(macWOdots.c_str(), "MHO-C401");
            }
            if (p->sensorModel_id == BLEconectable::id::XMWSDJ04MMC) {
              XMWSDJ04MMCDiscovery(macWOdots.c_str(), "XMWSDJ04MMC");
            }
            if (p->sensorModel_id == TheengsDecoder::BLE_ID_NUM::HHCCJCY01HHCC) {
              HHCCJCY01HHCCDiscovery(macWOdots.c_str(), "HHCCJCY01HHCC");
            }
          } else {
            Log.trace(F("Device UNKNOWN_MODEL %s" CR), p->macAdr);
          }
        }
      }
      p->isDisc = true;
    } else {
      Log.trace(F("Device already discovered or that doesn't require discovery %s" CR), p->macAdr);
    }
  }
}
# endif

void PublishDeviceData(JsonObject& BLEdata) {
  if (abs((int)BLEdata["rssi"] | 0) < abs(BTConfig.minRssi)) {

    process_bledata(BLEdata);

    if (!BTConfig.pubRandomMACs && (BLEdata["type"].as<string>()).compare("RMAC") == 0) {
      Log.trace(F("Random MAC, device filtered" CR));
      return;
    }

    if (!BTConfig.pubAdvData) {
      BLEdata.remove("servicedatauuid");
      BLEdata.remove("servicedata");
      BLEdata.remove("manufacturerdata");
      BLEdata.remove("mac_type");
      BLEdata.remove("adv_type");


      BLEdata.remove("cidc");
      BLEdata.remove("acts");
      BLEdata.remove("cont");
      BLEdata.remove("track");
    }

    if (!BTConfig.pubOnlySensors || BLEdata.containsKey("model")) {
      buildTopicFromId(BLEdata, subjectBTtoMQTT);
      handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
    } else {
      Log.notice(F("Not a sensor device filtered" CR));
      return;
    }
  } else if (BLEdata.containsKey("distance")) {
    if (BLEdata.containsKey("servicedatauuid"))
      BLEdata.remove("servicedatauuid");
    if (BLEdata.containsKey("servicedata"))
      BLEdata.remove("servicedata");
    if (BLEdata.containsKey("manufacturerdata"))
      BLEdata.remove("manufacturerdata");
    if (BTConfig.presenceUseBeaconUuid && BLEdata.containsKey("model_id") && BLEdata["model_id"].as<String>() == "IBEACON") {
      BLEdata["mac"] = BLEdata["id"];
      BLEdata["id"] = BLEdata["uuid"];
    }
    handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
  } else {
    Log.notice(F("Low rssi, device filtered" CR));
    return;
  }
}

void process_bledata(JsonObject& BLEdata) {
  yield();
  if (!BLEdata.containsKey("id")) {
    Log.error(F("No mac address in the payload" CR));
    return;
  }
  const char* mac = BLEdata["id"].as<const char*>();
  Log.trace(F("Processing BLE data %s" CR), BLEdata["id"].as<const char*>());
  int model_id = BTConfig.extDecoderEnable ? -1 : decoder.decodeBLEJson(BLEdata);
  int mac_type = BLEdata["mac_type"].as<int>();


  if (BLEdata["prmac"]) {
    BLEdata.remove("prmac");
    if (BLEdata["track"]) {
      BLEdata.remove("track");
    }
    BLEdata["type"] = "RMAC";
    Log.trace(F("Potential RMAC (prmac) converted to RMAC" CR));
  }

  if ((BLEdata["type"].as<string>()).compare("RMAC") != 0 && model_id != TheengsDecoder::BLE_ID_NUM::IBEACON) {
    if (model_id >= 0) {
      Log.trace(F("Decoder found device: %s" CR), BLEdata["model_id"].as<const char*>());
      if (model_id == TheengsDecoder::BLE_ID_NUM::HHCCJCY01HHCC || model_id == TheengsDecoder::BLE_ID_NUM::BM2) {
        createOrUpdateDevice(mac, device_flags_connect, model_id, mac_type);
      } else {
        createOrUpdateDevice(mac, device_flags_init, model_id, mac_type);
        if (BTConfig.adaptiveScan == true && (BTConfig.BLEinterval != MinTimeBtwScan || BTConfig.intervalActiveScan != MinTimeBtwScan)) {
          if (BLEdata.containsKey("acts") && BLEdata.containsKey("cont")) {
            if (BLEdata["acts"] && BLEdata["cont"]) {
              BTConfig.BLEinterval = MinTimeBtwScan;
              BTConfig.intervalActiveScan = MinTimeBtwScan;
              BTConfig.scanDuration = MinScanDuration;
              Log.notice(F("Active and continuous scanning required, paramaters adapted" CR));
              stateBTMeasures(false);
            }
          } else if (BLEdata.containsKey("cont") && BTConfig.BLEinterval != MinTimeBtwScan) {
            if (BLEdata["cont"]) {
              BTConfig.BLEinterval = MinTimeBtwScan;
              if ((BLEdata["type"].as<string>()).compare("CTMO") == 0) {
                BTConfig.scanDuration = MinScanDuration;
              }
              Log.notice(F("Passive continuous scanning required, paramaters adapted" CR));
              stateBTMeasures(false);
            }
          }
        }
      }
    } else {
      if (BLEdata.containsKey("name")) {
        std::string name = BLEdata["name"];
        if (name.compare("LYWSD03MMC") == 0)
          model_id = BLEconectable::id::LYWSD03MMC;
        else if (name.compare("DT24-BLE") == 0)
          model_id = BLEconectable::id::DT24_BLE;
        else if (name.compare("MHO-C401") == 0)
          model_id = BLEconectable::id::MHO_C401;
        else if (name.compare("XMWSDJ04MMC") == 0)
          model_id = BLEconectable::id::XMWSDJ04MMC;

        if (model_id > 0) {
          Log.trace(F("Connectable device found: %s" CR), name.c_str());
          createOrUpdateDevice(mac, device_flags_connect, model_id, mac_type);
        }
      } else if (BTConfig.extDecoderEnable && model_id < 0 && BLEdata.containsKey("servicedata")) {
        const char* service_data = (const char*)(BLEdata["servicedata"] | "");
        if (strstr(service_data, "209800") != NULL) {
          model_id == TheengsDecoder::BLE_ID_NUM::HHCCJCY01HHCC;
          Log.trace(F("Connectable device found: HHCCJCY01HHCC" CR));
          createOrUpdateDevice(mac, device_flags_connect, model_id, mac_type);
        }
      }
    }
    Log.trace(F("Random MAC or iBeacon device filtered" CR));
  }
  if (!BTConfig.extDecoderEnable && model_id < 0) {
    Log.trace(F("No eligible device found " CR));
  }
}

void hass_presence(JsonObject& HomePresence) {
  int BLErssi = HomePresence["rssi"];
  Log.trace(F("BLErssi %d" CR), BLErssi);
  int txPower = HomePresence["txpower"] | 0;
  if (txPower >= 0)
    txPower = -59;
  Log.trace(F("TxPower: %d" CR), txPower);
  double ratio = BLErssi * 1.0 / txPower;
  double distance;
  if (ratio < 1.0) {
    distance = pow(ratio, 10);
  } else {
    distance = (0.89976) * pow(ratio, 7.7095) + 0.111;
  }
  HomePresence["distance"] = distance;
  Log.trace(F("Ble distance %D" CR), distance);
}

void BTforceScan() {
  if (!BTProcessLock) {
    BTtoMQTT();
    Log.trace(F("Scan done" CR));
    if (BTConfig.bleConnect)
      BLEconnect();
  } else {
    Log.trace(F("Cannot launch scan due to other process running" CR));
  }
}

void immediateBTAction(void* pvParameters) {
  if (BLEactions.size()) {

    BTProcessLock = true;
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan->isScanning()) {
      pScan->stop();
    }

    if (xSemaphoreTake(semaphoreBLEOperation, pdMS_TO_TICKS(5000)) == pdTRUE) {

      std::vector<BLEdevice*> dev_swap;
      dev_swap.push_back(getDeviceByMac(BLEactions.back().addr));
      std::swap(devices, dev_swap);

      std::vector<BLEAction> act_swap;
      act_swap.push_back(BLEactions.back());
      BLEactions.pop_back();
      std::swap(BLEactions, act_swap);


      BTProcessLock = false;
      BLEconnect();

      std::swap(devices, dev_swap);
      std::swap(BLEactions, act_swap);


      if (millis() > (timeBetweenConnect + BTConfig.intervalConnect) && BTConfig.bleConnect) {
        timeBetweenConnect = millis();
        BLEconnect();
      }
      xSemaphoreGive(semaphoreBLEOperation);
    } else {
      Log.error(F("BLE busy - command not sent" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> BLEdataBuffer;
      JsonObject BLEdata = BLEdataBuffer.to<JsonObject>();
      BLEdata["id"] = BLEactions.back().addr;
      BLEdata["success"] = false;
      buildTopicFromId(BLEdata, subjectBTtoMQTT);
      handleJsonEnqueue(BLEdata, QueueSemaphoreTimeOutTask);
      BLEactions.pop_back();
      BTProcessLock = false;
    }
  }
  vTaskDelete(NULL);
}

void startBTActionTask() {
  TaskHandle_t th;
  xTaskCreatePinnedToCore(
      immediateBTAction,
      "imActTask",
      5120,
      NULL,
      3,
      &th,
      1);
}

void MQTTtoBTAction(JsonObject& BTdata) {
  BLEAction action;
  memset(&action, 0, sizeof(BLEAction));
  if (BTdata.containsKey("SBS1")) {
    strcpy(action.addr, (const char*)BTdata["mac"]);
    action.write = true;
    std::string val = BTdata["SBS1"].as<std::string>();
    action.value = val;
    action.ttl = 1;
    createOrUpdateDevice(action.addr, device_flags_connect,
                         TheengsDecoder::BLE_ID_NUM::SBS1, 1);
    BLEactions.push_back(action);
    startBTActionTask();
    return;
  }

  action.ttl = BTdata.containsKey("ttl") ? (uint8_t)BTdata["ttl"] : 1;
  action.addr_type = BTdata.containsKey("mac_type") ? BTdata["mac_type"].as<int>() : 0;
  action.value_type = BLE_VAL_STRING;
  if (BTdata.containsKey("value_type")) {
    String vt = BTdata["value_type"];
    vt.toUpperCase();
    if (vt == "HEX")
      action.value_type = BLE_VAL_HEX;
    else if (vt == "INT")
      action.value_type = BLE_VAL_INT;
    else if (vt == "FLOAT")
      action.value_type = BLE_VAL_FLOAT;
    else if (vt != "STRING") {
      Log.error(F("BLE value type invalid %s" CR), vt.c_str());
      return;
    }
  }

  Log.trace(F("BLE ACTION TTL = %u" CR), action.ttl);
  action.complete = false;
  if (BTdata.containsKey("ble_write_address") &&
      BTdata.containsKey("ble_write_service") &&
      BTdata.containsKey("ble_write_char") &&
      BTdata.containsKey("ble_write_value")) {
    strcpy(action.addr, (const char*)BTdata["ble_write_address"]);
    action.service = NimBLEUUID((const char*)BTdata["ble_write_service"]);
    action.characteristic = NimBLEUUID((const char*)BTdata["ble_write_char"]);
    std::string val = BTdata["ble_write_value"].as<std::string>();
    action.value = val;
    action.write = true;
    Log.trace(F("BLE ACTION Write" CR));
  } else if (BTdata.containsKey("ble_read_address") &&
             BTdata.containsKey("ble_read_service") &&
             BTdata.containsKey("ble_read_char")) {
    strcpy(action.addr, (const char*)BTdata["ble_read_address"]);
    action.service = NimBLEUUID((const char*)BTdata["ble_read_service"]);
    action.characteristic = NimBLEUUID((const char*)BTdata["ble_read_char"]);
    action.write = false;
    Log.trace(F("BLE ACTION Read" CR));
  } else {
    return;
  }

  createOrUpdateDevice(action.addr, device_flags_connect,
                       TheengsDecoder::BLE_ID_NUM::UNKNOWN_MODEL,
                       action.addr_type);

  BLEactions.push_back(action);
  if (BTdata.containsKey("immediate") && BTdata["immediate"].as<bool>()) {
    startBTActionTask();
  }
}

void MQTTtoBT(char* topicOri, JsonObject& BTdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoBTset)) {
    Log.trace(F("MQTTtoBT json set" CR));


    bool WorBupdated;
    WorBupdated = updateWorB(BTdata, true);
    WorBupdated |= updateWorB(BTdata, false);

    if (WorBupdated) {
      if (xSemaphoreTake(semaphoreCreateOrUpdateDevice, pdMS_TO_TICKS(QueueSemaphoreTimeOutTask)) == pdTRUE) {

        xSemaphoreGive(semaphoreCreateOrUpdateDevice);
      }
    }


    if (BTdata.containsKey("interval") && BTdata["interval"] == 0) {
      Log.notice(F("BLE forced scan" CR));
      atomic_store_explicit(&forceBTScan, 1, ::memory_order_seq_cst);
    }







    if (BTdata.containsKey("init") && BTdata["init"].as<bool>()) {

      BTConfig_init();
    } else if (BTdata.containsKey("load") && BTdata["load"].as<bool>()) {

      BTConfig_load();
    }


    BTConfig_fromJson(BTdata);

    if (BTdata.containsKey("lowpowermode")) {
      changelowpowermode((int)BTdata["lowpowermode"]);
    }
  } else if (cmpToMainTopic(topicOri, subjectMQTTtoBT)) {
    MQTTtoBTAction(BTdata);
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayGFSunInverter.ino"
# 26 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayGFSunInverter.ino"
#include "User_config.h"

#ifdef ZgatewayGFSunInverter

GfSun2000 GF = GfSun2000();

void GFSunInverterDataHandler(GfSun2000Data data) {
  StaticJsonDocument<JSON_MSG_BUFFER> jdataBuffer;
  JsonObject jdata = jdataBuffer.to<JsonObject>();

  jdata["device_id"] = (char*)data.deviceID;
  Log.trace(F("Device ID     : %s\n" CR), data.deviceID);
  jdata["ac_voltage"] = data.ACVoltage;
  Log.trace(F("AC Voltage    : %.1f\tV\n" CR), data.ACVoltage);
  jdata["dc_voltage"] = data.DCVoltage;
  Log.trace(F("DC Voltage    : %.1f\tV\n" CR), data.DCVoltage);
  jdata["power"] = data.averagePower;
  Log.trace(F("Output Power  : %.1f\tW (5min avg)\n" CR), data.averagePower);
  jdata["c_energy"] = data.customEnergyCounter;
  Log.trace(F("Custom Energy : %.1f\tkW/h (can be reseted)\n" CR), data.customEnergyCounter);
  jdata["t_energy"] = data.totalEnergyCounter;
  Log.trace(F("Total Energy  : %.1f\tkW/h\n" CR), data.totalEnergyCounter);

# ifdef GFSUNINVERTER_DEVEL
  StaticJsonDocument<JSON_MSG_BUFFER> jregisterBuffer;
  JsonObject jregister = jregisterBuffer.to<JsonObject>();
  char buffer[4];
  std::map<int16_t, int16_t>::iterator itr;
  for (itr = data.modbusRegistry.begin(); itr != data.modbusRegistry.end(); ++itr) {
    Log.notice("%d: %d\n", itr->first, itr->second);
    sprintf(buffer, "%d", itr->first);
    jregister[buffer] = itr->second;
  }
  jdata["register"] = jregister;
# endif
  jdata["origin"] = subjectRFtoMQTT;
  handleJsonEnqueue(jdata);
}

void GFSunInverterErrorHandler(int errorId, char* errorMessage) {
  char buffer[50];
  sprintf(buffer, "Error response: %02X - %s\n", errorId, errorMessage);
  Log.error(buffer);
  StaticJsonDocument<JSON_MSG_BUFFER> jdataBuffer;
  JsonObject jdata = jdataBuffer.to<JsonObject>();
  jdata["status"] = "error";
  jdata["msg"] = errorMessage;
  jdata["id"] = errorId;
  jdata["origin"] = subjectRFtoMQTT;
  handleJsonEnqueue(jdata);
}

void setupGFSunInverter() {
  GF.setup(Serial2);
  GF.setDataHandler(GFSunInverterDataHandler);
  GF.setErrorHandler(GFSunInverterErrorHandler);
  Log.trace(F("ZgatewayGFSunInverter setup done " CR));
}

void ZgatewayGFSunInverterMQTT() {
  GF.readData();
  delay(GFSUNINVERTER_DELAY);
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayIR.ino"
# 28 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayIR.ino"
#include "User_config.h"

#ifdef ZgatewayIR

# if defined(ESP8266) || defined(ESP32)
# include <IRrecv.h>
# include <IRremoteESP8266.h>
# include <IRsend.h>
# include <IRutils.h>
# ifdef DumpMode
IRrecv irrecv(IR_RECEIVER_GPIO, 1024, 15U, true);
# else
IRrecv irrecv(IR_RECEIVER_GPIO);
# endif
IRsend irsend(IR_EMITTER_GPIO, IR_EMITTER_INVERTED);
# else
# include <IRremote.h>
IRrecv irrecv(IR_RECEIVER_GPIO);
IRsend irsend;
# endif


# ifndef NEC_BITS
#define NEC_BITS 32U
# endif
# ifndef SAMSUNG_BITS
#define SAMSUNG_BITS 32U
# endif
# ifndef SHARP_BITS
#define SHARP_ADDRESS_BITS 5U
#define SHARP_COMMAND_BITS 8U
#define SHARP_BITS (SHARP_ADDRESS_BITS + SHARP_COMMAND_BITS + 2)
# endif
# ifndef RC5_BITS
#define RC5_RAW_BITS 14U
#define RC5_BITS RC5_RAW_BITS - 2U
# endif
# ifndef DISH_BITS
#define DISH_BITS 16U
# endif
# ifndef SONY_20_BITS
#define SONY_20_BITS 20
# endif
# ifndef SONY_12_BITS
#define SONY_12_BITS 12U
# endif
# ifndef LG_BITS
#define LG_BITS 28U
# endif
# ifndef WHYNTER_BITS
#define WHYNTER_BITS 32U
# endif


uint64_t getUInt64fromHex(char const* str) {
  uint64_t result = 0;
  uint16_t offset = 0;

  if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) offset = 2;
  for (; isxdigit((unsigned char)str[offset]); offset++) {
    char c = str[offset];
    result *= 16;
    if (isdigit(c))
      result += c - '0';
    else if (isupper(c))
      result += c - 'A' + 10;
    else
      result += c - 'a' + 10;
  }
  return result;
}

void setupIR() {

# if defined(ESP8266) || defined(ESP32)
  irsend.begin();
# endif

  irrecv.enableIRIn();

  Log.notice(F("IR_EMITTER_GPIO: %d " CR), IR_EMITTER_GPIO);
  Log.notice(F("IR_RECEIVER_GPIO: %d " CR), IR_RECEIVER_GPIO);
  Log.trace(F("ZgatewayIR setup done " CR));
}

void IRtoMQTT() {
  decode_results results;

  if (irrecv.decode(&results)) {
    Log.trace(F("Creating IR buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> IRdataBuffer;
    JsonObject IRdata = IRdataBuffer.to<JsonObject>();

    Log.trace(F("Rcv. IR" CR));
# ifdef ESP32
    Log.trace(F("IR Task running on core :%d" CR), xPortGetCoreID());
# endif
    IRdata["value"] = (SIGNAL_SIZE_UL_ULL)(results.value);
    IRdata["protocol"] = (int)(results.decode_type);
    IRdata["bits"] = (int)(results.bits);
# if defined(ESP8266) || defined(ESP32)
    IRdata["hex"] = resultToHexidecimal(&results);
    IRdata["protocol_name"] = typeToString(results.decode_type, false);
# endif
    String rawCode = "";

    for (uint16_t i = 1; i < results.rawlen; i++) {
# if defined(ESP8266) || defined(ESP32)
      if (i % 100 == 0)
        yield();
      rawCode = rawCode + (results.rawbuf[i] * RAWTICK);
# else
      rawCode = rawCode + (results.rawbuf[i] * USECPERTICK);
# endif
      if (i < results.rawlen - 1)
        rawCode = rawCode + ",";
    }
    IRdata["raw"] = rawCode;

# ifdef RawDirectForward
# if defined(ESP8266) || defined(ESP32)
    uint16_t rawsend[results.rawlen];
    for (uint16_t i = 1; i < results.rawlen; i++) {
      if (i % 100 == 0)
        yield();
# else
    unsigned int rawsend[results.rawlen];
    for (int i = 1; i < results.rawlen; i++) {
# endif
      rawsend[i] = results.rawbuf[i];
    }
    irsend.sendRaw(rawsend, results.rawlen, RawFrequency);
    Log.trace(F("raw redirected" CR));
# endif
    irrecv.resume();
    SIGNAL_SIZE_UL_ULL MQTTvalue = IRdata["value"].as<SIGNAL_SIZE_UL_ULL>();

    if ((pubIRunknownPrtcl == false && IRdata["protocol"].as<int>() == -1)) {
      Log.notice(F("--no pub unknwn prt--" CR));
    } else if (!isAduplicateSignal(MQTTvalue) && MQTTvalue != 0) {
      Log.trace(F("Adv data IRtoMQTT" CR));
      IRdata["origin"] = subjectIRtoMQTT;
      handleJsonEnqueue(IRdata);
      Log.trace(F("Store val: %D" CR), MQTTvalue);
      storeSignalValue(MQTTvalue);
      if (repeatIRwMQTT) {
        Log.trace(F("Pub. IR for rpt" CR));
        pubMQTT(subjectForwardMQTTtoIR, MQTTvalue);
      }
    }
  }
}

bool sendIdentifiedProtocol(const char* protocol_name, SIGNAL_SIZE_UL_ULL data, const char* hex, unsigned int valueBITS, uint16_t valueRPT);

# if jsonReceiving
void MQTTtoIR(char* topicOri, JsonObject& IRdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoIR)) {
    Log.trace(F("MQTTtoIR json" CR));
    uint64_t data = IRdata["value"];
    const char* raw = IRdata["raw"];
    const char* hex = IRdata["hex"];
    if (hex) {
      Log.trace(F("hex: %s" CR), hex);
      data = getUInt64fromHex(hex);
    }
    if (data != 0 || raw) {
      Log.trace(F("MQTTtoIR value || raw  detected" CR));
      bool signalSent = false;
      const char* protocol_name = IRdata["protocol_name"];
      unsigned int valueBITS = IRdata["bits"] | 0;
      uint16_t valueRPT = IRdata["repeat"] | repeatIRwNumber;

      if (raw) {
        Log.trace(F("Raw: %s" CR), raw);
        unsigned int s = strlen(raw);

        int count = 0;
        for (int i = 0; i < s; i++) {
          if (raw[i] == ',') {
            count++;
          }
        }
# ifdef IR_GC
        if (strcmp(protocol_name, "GC") == 0) {
          Log.trace(F("GC" CR));

          uint16_t GC[count + 1];
          String value = "";
          int j = 0;
          for (int i = 0; i < s; i++) {
            if (raw[i] != ',') {
              value = value + String(raw[i]);
            }
            if ((raw[i] == ',') || (i == s - 1)) {
              GC[j] = value.toInt();
              value = "";
              j++;
            }
          }
          irsend.sendGC(GC, j);
          signalSent = true;
        }
# endif
# ifdef IR_RAW
        if (strcmp(protocol_name, "Raw") == 0) {
          Log.trace(F("Raw" CR));

# if defined(ESP8266) || defined(ESP32)
          uint16_t Raw[count + 1];
# else
          unsigned int Raw[count + 1];
# endif
          String value = "";
          int j = 0;
          for (int i = 0; i < s; i++) {
            if (raw[i] != ',') {
              value = value + String(raw[i]);
            }
            if ((raw[i] == ',') || (i == s - 1)) {
              Raw[j] = value.toInt();
              value = "";
              j++;
            }
          }
          irsend.sendRaw(Raw, j, RawFrequency);
          signalSent = true;
        }
# endif
      } else if (protocol_name && (strcmp(protocol_name, "NEC") != 0)) {
        Log.trace(F("Using Identified Protocol: %s  bits: %d repeat: %d" CR), protocol_name, valueBITS, valueRPT);
        signalSent = sendIdentifiedProtocol(protocol_name, data, hex, valueBITS, valueRPT);
      } else {
        Log.trace(F("Using NEC protocol" CR));
        Log.notice(F("Sending IR signal with %s" CR), protocol_name);
        if (valueBITS == 0)
          valueBITS = NEC_BITS;
# if defined(ESP8266) || defined(ESP32)
        irsend.sendNEC(data, valueBITS, valueRPT);
# else
        for (int i = 0; i <= valueRPT; i++)
          irsend.sendNEC(data, valueBITS);
# endif
        signalSent = true;
      }
      if (signalSent) {
        Log.notice(F("MQTTtoIR OK" CR));
        pub(subjectGTWIRtoMQTT, IRdata);
      }
      irrecv.enableIRIn();
    } else {
      Log.error(F("MQTTtoIR failed json read" CR));
    }
  }
}
# endif

bool sendIdentifiedProtocol(const char* protocol_name, SIGNAL_SIZE_UL_ULL data, const char* hex, unsigned int valueBITS, uint16_t valueRPT) {
  uint8_t dataarray[valueBITS];
  if (hex) {
    const char* ptr = NULL;
    (strstr(hex, "0x") != NULL) ? ptr = hex += 2 : ptr = hex;
    for (int i = 0; i < sizeof dataarray / sizeof *dataarray; i++) {
      sscanf(ptr, "%2hhx", &dataarray[i]);
      ptr += 2;
    }
    for (int i = 0; i < valueBITS; i++) {
      Log.trace(F("%x"), dataarray[i]);
    }
  }
# ifdef IR_WHYNTER
  if (strcmp(protocol_name, "WHYNTER") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = WHYNTER_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendWhynter(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendWhynter(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_LG
  if (strcmp(protocol_name, "LG") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = LG_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendLG(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendLG(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_SONY
  if (strcmp(protocol_name, "SONY") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
# if defined(ESP8266) || defined(ESP32)
    if (valueBITS == 0)
      valueBITS = SONY_20_BITS;
    irsend.sendSony(data, valueBITS, valueRPT);
# else
    if (valueBITS == 0)
      valueBITS = SONY_12_BITS;
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendSony(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_DISH
  if (strcmp(protocol_name, "DISH") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = DISH_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendDISH(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendDISH(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_RC5
  if (strcmp(protocol_name, "RC5") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = RC5_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendRC5(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendRC5(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_RC6
  if (strcmp(protocol_name, "RC6") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = RC6_MODE0_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendRC6(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendRC6(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_SHARP
  if (strcmp(protocol_name, "SHARP") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = SHARP_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendSharpRaw(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendSharpRaw(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_SAMSUNG
  if (strcmp(protocol_name, "SAMSUNG") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = SAMSUNG_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendSAMSUNG(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendSAMSUNG(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_JVC
  if (strcmp(protocol_name, "JVC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = JVC_BITS;
# if defined(ESP8266) || defined(ESP32)
    irsend.sendJVC(data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendJVC(data, valueBITS);
# endif
    return true;
  }
# endif
# ifdef IR_PANASONIC
  if (strcmp(protocol_name, "PANASONIC") == 0) {
# if defined(ESP8266) || defined(ESP32)
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = PANASONIC_BITS;
    irsend.sendPanasonic(PanasonicAddress, data, valueBITS, valueRPT);
# else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendPanasonic(PanasonicAddress, data);
# endif
    return true;
  }
# endif

# if defined(ESP8266) || defined(ESP32)
# ifdef IR_COOLIX
  if (strcmp(protocol_name, "COOLIX") == 0) {
    Log.trace(F("Sending %s:" CR), protocol_name);
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kCoolixBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kCoolixDefaultRepeat);
    irsend.sendCOOLIX(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_RCMM
  if (strcmp(protocol_name, "RCMM") == 0) {
    Log.trace(F("Sending %s:" CR), protocol_name);
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kRCMMBits;
    irsend.sendRCMM(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DENON
  if (strcmp(protocol_name, "DENON") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = DENON_BITS;
    irsend.sendDenon(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_GICABLE
  if (strcmp(protocol_name, "GICABLE") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kGicableBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kGicableMinRepeat);
    irsend.sendGICable(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SHERWOOD
  if (strcmp(protocol_name, "SHERWOOD") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kSherwoodBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kSherwoodMinRepeat);
    irsend.sendSherwood(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MITSUBISHI
  if (strcmp(protocol_name, "MITSUBISHI") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMitsubishiBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiMinRepeat);
    irsend.sendMitsubishi(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_NIKAI
  if (strcmp(protocol_name, "NIKAI") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kNikaiBits;
    irsend.sendNikai(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MIDEA
  if (strcmp(protocol_name, "MIDEA") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMideaBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMideaMinRepeat);
    irsend.sendMidea(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MAGIQUEST
  if (strcmp(protocol_name, "MAGIQUEST") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMagiquestBits;
    irsend.sendMagiQuest(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_LASERTAG
  if (strcmp(protocol_name, "LASERTAG") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kLasertagBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kLasertagMinRepeat);
    irsend.sendLasertag(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_CARRIER_AC
  if (strcmp(protocol_name, "CARRIER_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kCarrierAcBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kCarrierAcMinRepeat);
    irsend.sendCarrierAC(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MITSUBISHI2
  if (strcmp(protocol_name, "MITSUBISHI2") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMitsubishiBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiMinRepeat);
    irsend.sendMitsubishi2(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_AIWA_RC_T501
  if (strcmp(protocol_name, "AIWA_RC_T501") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kAiwaRcT501Bits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kAiwaRcT501MinRepeats);
    irsend.sendAiwaRCT501(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN
  if (strcmp(protocol_name, "DAIKIN") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDaikinStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikinDefaultRepeat);
    irsend.sendDaikin(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_KELVINATOR
  if (strcmp(protocol_name, "KELVINATOR") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kKelvinatorStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kKelvinatorDefaultRepeat);
    irsend.sendKelvinator(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MITSUBISHI_AC
  if (strcmp(protocol_name, "MITSUBISHI_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMitsubishiACStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiACMinRepeat);
    irsend.sendMitsubishiAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SANYO
  if (strcmp(protocol_name, "SANYOLC7461") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kSanyoLC7461Bits;
    irsend.sendSanyoLC7461(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_GREE
  if (strcmp(protocol_name, "GREE") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kGreeStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kGreeDefaultRepeat);
    irsend.sendGree(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_ARGO
  if (strcmp(protocol_name, "ARGO") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kArgoStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kArgoDefaultRepeat);
    irsend.sendArgo(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TROTEC
  if (strcmp(protocol_name, "TROTEC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kTrotecStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kTrotecDefaultRepeat);
    irsend.sendTrotec(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TOSHIBA_AC
  if (strcmp(protocol_name, "TOSHIBA_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kToshibaACBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kToshibaACMinRepeat);
    irsend.sendToshibaAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_FUJITSU_AC
  if (strcmp(protocol_name, "FUJITSU_AC") == 0) {
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kFujitsuAcMinRepeat);
    irsend.sendFujitsuAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HAIER_AC
  if (strcmp(protocol_name, "HAIER_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHaierACStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHaierAcDefaultRepeat);
    irsend.sendHaierAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC
  if (strcmp(protocol_name, "HITACHI_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHitachiAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC1
  if (strcmp(protocol_name, "HITACHI_AC1") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHitachiAc1StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAC1(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC2
  if (strcmp(protocol_name, "HITACHI_AC2") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHitachiAc2StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAC2(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HAIER_AC_YRW02
  if (strcmp(protocol_name, "HAIER_AC_YRW02") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHaierACYRW02StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHaierAcYrw02DefaultRepeat);
    irsend.sendHaierACYRW02(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_WHIRLPOOL_AC
  if (strcmp(protocol_name, "WHIRLPOOL_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kWhirlpoolAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kWhirlpoolAcDefaultRepeat);
    irsend.sendWhirlpoolAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SAMSUNG_AC
  if (strcmp(protocol_name, "SAMSUNG_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kSamsungAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kSamsungAcDefaultRepeat);
    irsend.sendSamsungAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_LUTRON
  if (strcmp(protocol_name, "LUTRON") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kLutronBits;
    irsend.sendLutron(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_ELECTRA_AC
  if (strcmp(protocol_name, "ELECTRA_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kElectraAcStateLength;
    irsend.sendElectraAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_PANASONIC_AC
  if (strcmp(protocol_name, "PANASONIC_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kPanasonicAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kPanasonicAcDefaultRepeat);
    irsend.sendPanasonicAC(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_PIONEER
  if (strcmp(protocol_name, "PIONEER") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kPioneerBits;
    irsend.sendPioneer(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_LG2
  if (strcmp(protocol_name, "LG2") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kLgBits;
    irsend.sendLG2(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MWM
  if (strcmp(protocol_name, "MWM") == 0) {
    irsend.sendMWM(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN2
  if (strcmp(protocol_name, "DAIKIN2") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDaikin2StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin2DefaultRepeat);
    irsend.sendDaikin2(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_VESTEL_AC
  if (strcmp(protocol_name, "VESTEL_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kVestelAcBits;
    irsend.sendVestelAc(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SAMSUNG36
  if (strcmp(protocol_name, "SAMSUNG36") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kSamsung36Bits;
    irsend.sendSamsung36(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TCL112AC
  if (strcmp(protocol_name, "TCL112AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kTcl112AcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kTcl112AcDefaultRepeat);
    irsend.sendTcl112Ac(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TECO
  if (strcmp(protocol_name, "TECO") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kTecoBits;
    irsend.sendTeco(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_LEGOPF
  if (strcmp(protocol_name, "LEGOPF") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kLegoPfBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kLegoPfMinRepeat);
    irsend.sendLegoPf(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MITSUBISHIHEAVY88
  if (strcmp(protocol_name, "MITSUBISHIHEAVY88") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMitsubishiHeavy88StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiHeavy88MinRepeat);
    irsend.sendMitsubishiHeavy88(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MITSUBISHIHEAVY152
  if (strcmp(protocol_name, "MITSUBISHIHEAVY152") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMitsubishiHeavy152StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiHeavy152MinRepeat);
    irsend.sendMitsubishiHeavy152(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN216
  if (strcmp(protocol_name, "DAIKIN216") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDaikin216StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin216DefaultRepeat);
    irsend.sendDaikin216(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SHARP_AC
  if (strcmp(protocol_name, "SHARP_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kSharpAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kSharpAcDefaultRepeat);
    irsend.sendSharpAc(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_GOODWEATHER
  if (strcmp(protocol_name, "GOODWEATHER_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kGoodweatherBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kGoodweatherMinRepeat);
    irsend.sendGoodweather(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_INAX
  if (strcmp(protocol_name, "INAX") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kInaxBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kInaxMinRepeat);
    irsend.sendInax(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN160
  if (strcmp(protocol_name, "DAIKIN160") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDaikin160StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin160DefaultRepeat);
    irsend.sendDaikin160(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_NEOCLIMA
  if (strcmp(protocol_name, "NEOCLIMA") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kNeoclimaStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kNeoclimaMinRepeat);
    irsend.sendNeoclima(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN176
  if (strcmp(protocol_name, "DAIKIN176") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDaikin176StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin176DefaultRepeat);
    irsend.sendDaikin176(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN128
  if (strcmp(protocol_name, "DAIKIN128") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDaikin128StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin128DefaultRepeat);
    irsend.sendDaikin128(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_AMCOR
  if (strcmp(protocol_name, "AMCOR") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kAmcorStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kAmcorDefaultRepeat);
    irsend.sendAmcor(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN152
  if (strcmp(protocol_name, "DAIKIN152") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDaikin152StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin152DefaultRepeat);
    irsend.sendDaikin152(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MITSUBISHI136
  if (strcmp(protocol_name, "MITSUBISHI136") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMitsubishi136StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishi136MinRepeat);
    irsend.sendMitsubishi136(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MITSUBISHI112
  if (strcmp(protocol_name, "MITSUBISHI112") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kMitsubishi112StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishi112MinRepeat);
    irsend.sendMitsubishi112(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC424
  if (strcmp(protocol_name, "HITACHI_AC424") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHitachiAc424StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAc424(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SONY_38K
  if (strcmp(protocol_name, "SONY_38K") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, (uint16_t)(kSonyMinRepeat + 1));
    if (valueBITS == 0)
      valueBITS = kSony20Bits;
    irsend.sendSony38(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_EPSON
  if (strcmp(protocol_name, "EPSON") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kEpsonMinRepeat);
    if (valueBITS == 0)
      valueBITS = kEpsonBits;
    irsend.sendEpson(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SYMPHONY
  if (strcmp(protocol_name, "SYMPHONY") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kSymphonyDefaultRepeat);
    if (valueBITS == 0)
      valueBITS = kSymphonyBits;
    irsend.sendSymphony(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC3
  if (strcmp(protocol_name, "HITACHI_AC3") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    if (valueBITS == 0)
      Log.error(F("For this protocol you should have a BIT number as there is no default one defined" CR));
    irsend.sendHitachiAc3(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DAIKIN64
  if (strcmp(protocol_name, "DAIKIN64") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin64DefaultRepeat);
    if (valueBITS == 0)
      valueBITS = kDaikin64Bits;
    irsend.sendDaikin64(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_AIRWELL
  if (strcmp(protocol_name, "AIRWELL") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kAirwellMinRepeats);
    if (valueBITS == 0)
      valueBITS = kAirwellBits;
    irsend.sendAirwell(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DELONGHI_AC
  if (strcmp(protocol_name, "DELONGHI_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDelonghiAcDefaultRepeat);
    if (valueBITS == 0)
      valueBITS = kDelonghiAcBits;
    irsend.sendDelonghiAc(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_DOSHISHA
  if (strcmp(protocol_name, "DOSHISHA") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kDoshishaBits;
    irsend.sendDoshisha(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_CARRIER_AC40
  if (strcmp(protocol_name, "CARRIER_AC40") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kCarrierAc40MinRepeat);
    if (valueBITS == 0)
      valueBITS = kCarrierAc40Bits;
    irsend.sendCarrierAC40(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_CARRIER_AC64
  if (strcmp(protocol_name, "CARRIER_AC64") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kCarrierAc64MinRepeat);
    if (valueBITS == 0)
      valueBITS = kCarrierAc64Bits;
    irsend.sendCarrierAC64(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC344
  if (strcmp(protocol_name, "HITACHI_AC344") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHitachiAc344StateLength;
    irsend.sendHitachiAc344(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_CORONA_AC
  if (strcmp(protocol_name, "CORONA_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kCoronaAcStateLength;
    irsend.sendCoronaAc(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MIDEA24
  if (strcmp(protocol_name, "MIDEA24") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMidea24MinRepeat);
    if (valueBITS == 0)
      valueBITS = kMidea24Bits;
    irsend.sendMidea24(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_ZEPEAL
  if (strcmp(protocol_name, "ZEPEAL") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kZepealMinRepeat);
    if (valueBITS == 0)
      valueBITS = kZepealBits;
    irsend.sendZepeal(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SANYO_AC
  if (strcmp(protocol_name, "SANYO_AC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kSanyoAcStateLength;
    irsend.sendSanyoAc(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_VOLTAS
  if (strcmp(protocol_name, "VOLTAS") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kVoltasStateLength;
    irsend.sendVoltas(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_METZ
  if (strcmp(protocol_name, "METZ") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMetzMinRepeat);
    if (valueBITS == 0)
      valueBITS = kMetzBits;
    irsend.sendMetz(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TRANSCOLD
  if (strcmp(protocol_name, "TRANSCOLD") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kTranscoldDefaultRepeat);
    if (valueBITS == 0)
      valueBITS = kTranscoldBits;
    irsend.sendTranscold(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TECHNIBEL_AC
  if (strcmp(protocol_name, "TECHNIBELAC") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kTechnibelAcDefaultRepeat);
    if (valueBITS == 0)
      valueBITS = kTechnibelAcBits;
    irsend.sendTechnibelAc(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MIRAGE
  if (strcmp(protocol_name, "MIRAGE") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMirageMinRepeat);
    if (valueBITS == 0)
      valueBITS = kMirageStateLength;
    irsend.sendMirage(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_ELITESCREENS
  if (strcmp(protocol_name, "ELITESCREENS") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kEliteScreensDefaultRepeat);
    if (valueBITS == 0)
      valueBITS = kEliteScreensBits;
    irsend.sendElitescreens(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_PANASONIC_AC32
  if (strcmp(protocol_name, "PANASONIC_AC32") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kPioneerBits;
    irsend.sendPanasonicAC32(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_MILESTAG2
  if (strcmp(protocol_name, "MILESTAG2") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMilesMinRepeat);
    if (valueBITS == 0)
      valueBITS = kMilesTag2ShotBits;
    irsend.sendMilestag2(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_ECOCLIM
  if (strcmp(protocol_name, "ECOCLIM") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kEcoclimBits;
    irsend.sendEcoclim(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_XMP
  if (strcmp(protocol_name, "XMP") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kXmpBits;
    irsend.sendXmp(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_KELON168
  if (strcmp(protocol_name, "KELON168") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kKelon168Bits;
    irsend.sendKelon168(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TEKNOPOINT
  if (strcmp(protocol_name, "TEKNOPOINT") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kTeknopointBits;
    irsend.sendTeknopoint(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HAIER_AC176
  if (strcmp(protocol_name, "HAIER_AC176") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHaierAC176Bits;
    irsend.sendHaierAC176(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_BOSE
  if (strcmp(protocol_name, "BOSE") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kBoseBits;
    irsend.sendBose(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_SANYO_AC88
  if (strcmp(protocol_name, "SANYO_AC88") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kSanyoAc88Bits;
    irsend.sendSanyoAc88(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_TROTEC_3550
  if (strcmp(protocol_name, "TROTEC_3550") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kTrotecBits;
    irsend.sendTrotec3550(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_ARRIS
  if (strcmp(protocol_name, "ARRIS") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kArrisBits;
    irsend.sendArris(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_RHOSS
  if (strcmp(protocol_name, "RHOSS") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kRhossBits;
    irsend.sendRhoss(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_AIRTON
  if (strcmp(protocol_name, "AIRTON") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kAirtonBits;
    irsend.sendAirton(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_COOLIX48
  if (strcmp(protocol_name, "COOLIX48") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kCoolix48Bits;
    irsend.sendCoolix48(data, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC264
  if (strcmp(protocol_name, "HITACHI_AC264") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHitachiAc264Bits;
    irsend.sendHitachiAc264(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
# ifdef IR_HITACHI_AC296
  if (strcmp(protocol_name, "HITACHI_AC296") == 0) {
    Log.notice(F("Sending IR signal with %s" CR), protocol_name);
    if (valueBITS == 0)
      valueBITS = kHitachiAc296Bits;
    irsend.sendHitachiAc296(dataarray, valueBITS, valueRPT);
    return true;
  }
# endif
  Log.warning(F("Unknown IR protocol" CR));
  return false;
# endif
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayLORA.ino"
# 28 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayLORA.ino"
#include "User_config.h"

#ifdef ZgatewayLORA

# include <LoRa.h>
# include <SPI.h>
# include <Wire.h>

#define WIPHONE_MESSAGE_MAGIC 0x6c6d
#define WIPHONE_MESSAGE_MIN_LEN sizeof(wiphone_message) - WIPHONE_MAX_MESSAGE_LEN
#define WIPHONE_MAX_MESSAGE_LEN 230

LORAConfig_s LORAConfig;

# ifdef ZmqttDiscovery
SemaphoreHandle_t semaphorecreateOrUpdateDeviceLORA;
std::vector<LORAdevice*> LORAdevices;
int newLORADevices = 0;

static LORAdevice NO_LORA_DEVICE_FOUND = {{0},
                                          0,
                                          false};

LORAdevice* getDeviceById(const char* id);
LORAdevice* getDeviceById(const char* id) {
  Log.trace(F("getDeviceById %s" CR), id);

  for (std::vector<LORAdevice*>::iterator it = LORAdevices.begin(); it != LORAdevices.end(); ++it) {
    if ((strcmp((*it)->uniqueId, id) == 0)) {
      return *it;
    }
  }
  return &NO_LORA_DEVICE_FOUND;
}

void dumpLORADevices() {
  for (std::vector<LORAdevice*>::iterator it = LORAdevices.begin(); it != LORAdevices.end(); ++it) {
    LORAdevice* p = *it;
    Log.trace(F("uniqueId %s" CR), p->uniqueId);
    Log.trace(F("modelName %s" CR), p->modelName);
    Log.trace(F("isDisc %d" CR), p->isDisc);
  }
}

void createOrUpdateDeviceLORA(const char* id, const char* model, uint8_t flags) {
  if (xSemaphoreTake(semaphorecreateOrUpdateDeviceLORA, pdMS_TO_TICKS(30000)) == pdFALSE) {
    Log.error(F("[LORA] semaphorecreateOrUpdateDeviceLORA Semaphore NOT taken" CR));
    return;
  }

  LORAdevice* device = getDeviceById(id);
  if (device == &NO_LORA_DEVICE_FOUND) {
    Log.trace(F("add %s" CR), id);

    device = new LORAdevice();
    if (strlcpy(device->uniqueId, id, uniqueIdSize) > uniqueIdSize) {
      Log.warning(F("[LORA] Device id %s exceeds available space" CR), id);
    };
    if (strlcpy(device->modelName, model, modelNameSize) > modelNameSize) {
      Log.warning(F("[LORA] Device model %s exceeds available space" CR), id);
    };
    device->isDisc = flags & device_flags_isDisc;
    LORAdevices.push_back(device);
    newLORADevices++;
  } else {
    Log.trace(F("update %s" CR), id);

    if (flags & device_flags_isDisc) {
      device->isDisc = true;
    }
  }

  xSemaphoreGive(semaphorecreateOrUpdateDeviceLORA);
}



void launchLORADiscovery(bool overrideDiscovery) {
  if (!overrideDiscovery && newLORADevices == 0)
    return;
  if (xSemaphoreTake(semaphorecreateOrUpdateDeviceLORA, pdMS_TO_TICKS(QueueSemaphoreTimeOutLoop)) == pdFALSE) {
    Log.error(F("[LORA] semaphorecreateOrUpdateDeviceLORA Semaphore NOT taken" CR));
    return;
  }
  newLORADevices = 0;
  std::vector<LORAdevice*> localDevices = LORAdevices;
  xSemaphoreGive(semaphorecreateOrUpdateDeviceLORA);
  for (std::vector<LORAdevice*>::iterator it = localDevices.begin(); it != localDevices.end(); ++it) {
    LORAdevice* pdevice = *it;
    Log.trace(F("Device id %s" CR), pdevice->uniqueId);

    if (overrideDiscovery || !isDiscovered(pdevice)) {
      size_t numRows = sizeof(LORAparameters) / sizeof(LORAparameters[0]);
      for (int i = 0; i < numRows; i++) {
        if (strstr(pdevice->uniqueId, LORAparameters[i][0]) != 0) {

          String idWoKey = pdevice->uniqueId;
          idWoKey.remove(idWoKey.length() - (strlen(LORAparameters[i][0]) + 1));
          Log.trace(F("idWoKey %s" CR), idWoKey.c_str());
          String value_template = "{{ value_json." + String(LORAparameters[i][0]) + " | is_defined }}";

          String topic = idWoKey;
          topic = String(subjectLORAtoMQTT) + "/" + topic;

          createDiscovery("sensor",
                          (char*)topic.c_str(), LORAparameters[i][1], pdevice->uniqueId,
                          "", LORAparameters[i][3], (char*)value_template.c_str(),
                          "", "", LORAparameters[i][2],
                          0,
                          "", "", false, "",
                          (char*)idWoKey.c_str(), "", pdevice->modelName, (char*)idWoKey.c_str(), false,
                          stateClassMeasurement
          );
          pdevice->isDisc = true;
          dumpLORADevices();
          break;
        }
      }
      if (!pdevice->isDisc) {
        Log.trace(F("Device id %s was not discovered" CR), pdevice->uniqueId);
      }
    } else {
      Log.trace(F("Device already discovered or that doesn't require discovery %s" CR), pdevice->uniqueId);
    }
  }
}

void storeLORADiscovery(JsonObject& RFLORA_ESPdata, const char* model, const char* uniqueid) {

  String modelSanitized = model;
  modelSanitized.replace(" ", "_");
  modelSanitized.replace("/", "_");
  modelSanitized.replace(".", "_");
  modelSanitized.replace("&", "");


  size_t numRows = sizeof(LORAparameters) / sizeof(LORAparameters[0]);

  for (int i = 0; i < numRows; i++) {
    if (RFLORA_ESPdata.containsKey(LORAparameters[i][0])) {
      String key_id = String(uniqueid) + "-" + String(LORAparameters[i][0]);
      createOrUpdateDeviceLORA((char*)key_id.c_str(), (char*)modelSanitized.c_str(), device_flags_init);
    }
  }
}
# endif

typedef struct __attribute__((packed)) {

  uint8_t rh_to;
  uint8_t rh_from;
  uint8_t rh_id;
  uint8_t rh_flags;
  uint16_t magic;
  uint32_t to;
  uint32_t from;
  char message[WIPHONE_MAX_MESSAGE_LEN];
} wiphone_message;

enum LORA_ID_NUM {
  UNKNOWN_DEVICE = -1,
  WIPHONE,
};
typedef enum LORA_ID_NUM LORA_ID_NUM;




uint8_t _determineDevice(byte* packet, int packetSize) {

  if (packetSize >= WIPHONE_MESSAGE_MIN_LEN && ((wiphone_message*)packet)->magic == WIPHONE_MESSAGE_MAGIC)
    return WIPHONE;


  return UNKNOWN_DEVICE;
}




uint8_t _determineDevice(JsonObject& LORAdata) {
  const char* protocol_name = LORAdata["type"];


  if (!protocol_name)
    return UNKNOWN_DEVICE;

  if (strcmp(protocol_name, "WiPhone") == 0)
    return WIPHONE;


  return UNKNOWN_DEVICE;
}




boolean _WiPhoneToMQTT(byte* packet, JsonObject& LORAdata) {

  wiphone_message* msg = (wiphone_message*)packet;


  char from[9] = {0};
  char to[9] = {0};
  snprintf(from, 9, "%06X", msg->from);
  snprintf(to, 9, "%06X", msg->to);



  LORAdata["from"] = from;
  LORAdata["to"] = to;

  LORAdata["message"] = msg->message;
  LORAdata["type"] = "WiPhone";
  return true;
}




boolean _MQTTtoWiPhone(JsonObject& LORAdata) {

  wiphone_message wiphonemsg;
  wiphonemsg.rh_to = 0xff;
  wiphonemsg.rh_from = 0xff;
  wiphonemsg.rh_id = 0x00;
  wiphonemsg.rh_flags = 0x00;

  wiphonemsg.magic = WIPHONE_MESSAGE_MAGIC;
  wiphonemsg.from = strtol(LORAdata["from"], NULL, 16);
  wiphonemsg.to = strtol(LORAdata["to"], NULL, 16);
  const char* message = LORAdata["message"];
  strlcpy(wiphonemsg.message, message, WIPHONE_MAX_MESSAGE_LEN);
  LoRa.write((uint8_t*)&wiphonemsg, strlen(message) + WIPHONE_MESSAGE_MIN_LEN + 1);
  return true;
}

void LORAConfig_init() {
  LORAConfig.frequency = LORA_BAND;
  LORAConfig.txPower = LORA_TX_POWER;
  LORAConfig.spreadingFactor = LORA_SPREADING_FACTOR;
  LORAConfig.signalBandwidth = LORA_SIGNAL_BANDWIDTH;
  LORAConfig.codingRateDenominator = LORA_CODING_RATE;
  LORAConfig.preambleLength = LORA_PREAMBLE_LENGTH;
  LORAConfig.syncWord = LORA_SYNC_WORD;
  LORAConfig.crc = DEFAULT_CRC;
  LORAConfig.invertIQ = INVERT_IQ;
  LORAConfig.onlyKnown = LORA_ONLY_KNOWN;
}

void LORAConfig_load() {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  preferences.begin(Gateway_Short_Name, true);
  if (preferences.isKey("LORAConfig")) {
    auto error = deserializeJson(jsonBuffer, preferences.getString("LORAConfig", "{}"));
    preferences.end();
    Log.notice(F("LORA Config loaded" CR));
    if (error) {
      Log.error(F("LORA Config deserialization failed: %s, buffer capacity: %u" CR), error.c_str(), jsonBuffer.capacity());
      return;
    }
    if (jsonBuffer.isNull()) {
      Log.warning(F("LORA Config is null" CR));
      return;
    }
    JsonObject jo = jsonBuffer.as<JsonObject>();
    LORAConfig_fromJson(jo);
    Log.notice(F("LORA Config loaded" CR));
  } else {
    preferences.end();
    Log.notice(F("LORA Config not found" CR));
  }
}

byte hexStringToByte(const String& hexString) {
  return (byte)strtol(hexString.c_str(), NULL, 16);
}

void LORAConfig_fromJson(JsonObject& LORAdata) {
  Config_update(LORAdata, "frequency", LORAConfig.frequency);
  Config_update(LORAdata, "txpower", LORAConfig.txPower);
  Config_update(LORAdata, "spreadingfactor", LORAConfig.spreadingFactor);
  Config_update(LORAdata, "signalbandwidth", LORAConfig.signalBandwidth);
  Config_update(LORAdata, "codingrate", LORAConfig.codingRateDenominator);
  Config_update(LORAdata, "preamblelength", LORAConfig.preambleLength);
  Config_update(LORAdata, "onlyknown", LORAConfig.onlyKnown);

  if (LORAdata.containsKey("syncword")) {
    String syncWordStr = LORAdata["syncword"].as<String>();
    LORAConfig.syncWord = hexStringToByte(syncWordStr);
    Log.notice(F("Config syncword changed: %d" CR), LORAConfig.syncWord);
  }
  Config_update(LORAdata, "enablecrc", LORAConfig.crc);
  Config_update(LORAdata, "invertiq", LORAConfig.invertIQ);

  LoRa.setFrequency(LORAConfig.frequency);
  LoRa.setTxPower(LORAConfig.txPower);
  LoRa.setSpreadingFactor(LORAConfig.spreadingFactor);
  LoRa.setSignalBandwidth(LORAConfig.signalBandwidth);
  LoRa.setCodingRate4(LORAConfig.codingRateDenominator);
  LoRa.setPreambleLength(LORAConfig.preambleLength);
  LoRa.setSyncWord(LORAConfig.syncWord);
  LORAConfig.crc ? LoRa.enableCrc() : LoRa.disableCrc();
  LORAConfig.invertIQ ? LoRa.enableInvertIQ() : LoRa.disableInvertIQ();

  if (LORAdata.containsKey("erase") && LORAdata["erase"].as<bool>()) {

    preferences.begin(Gateway_Short_Name, false);
    if (preferences.isKey("LORAConfig")) {
      int result = preferences.remove("LORAConfig");
      Log.notice(F("LORA config erase result: %d" CR), result);
      preferences.end();
      return;
    } else {
      Log.notice(F("LORA config not found" CR));
      preferences.end();
    }
  }
  if (LORAdata.containsKey("save") && LORAdata["save"].as<bool>()) {
    StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject jo = jsonBuffer.to<JsonObject>();
    jo["frequency"] = LORAConfig.frequency;

    String conf = "";
    serializeJson(jsonBuffer, conf);
    preferences.begin(Gateway_Short_Name, false);
    int result = preferences.putString("LORAConfig", conf);
    preferences.end();
    Log.notice(F("LORA Config_save: %s, result: %d" CR), conf.c_str(), result);
  }
}

void setupLORA() {
  LORAConfig_init();
  LORAConfig_load();
# ifdef ZmqttDiscovery
  semaphorecreateOrUpdateDeviceLORA = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphorecreateOrUpdateDeviceLORA);
# endif
  Log.notice(F("LORA Frequency: %d" CR), LORAConfig.frequency);
# ifdef ESP8266
  SPI.begin();
# else
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
# endif

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);

  if (!LoRa.begin(LORAConfig.frequency)) {
    Log.error(F("ZgatewayLORA setup failed!" CR));
    while (1)
      ;
  }
  LoRa.receive();
  Log.notice(F("LORA_SCK: %d" CR), LORA_SCK);
  Log.notice(F("LORA_MISO: %d" CR), LORA_MISO);
  Log.notice(F("LORA_MOSI: %d" CR), LORA_MOSI);
  Log.notice(F("LORA_SS: %d" CR), LORA_SS);
  Log.notice(F("LORA_RST: %d" CR), LORA_RST);
  Log.notice(F("LORA_DI0: %d" CR), LORA_DI0);
  Log.trace(F("ZgatewayLORA setup done" CR));
}

void LORAtoMQTT() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    StaticJsonDocument<JSON_MSG_BUFFER> LORAdataBuffer;
    JsonObject LORAdata = LORAdataBuffer.to<JsonObject>();
    Log.trace(F("Rcv. LORA" CR));
# ifdef ESP32
    String taskMessage = "LORA Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();

# endif

    byte packet[packetSize + 1];
    boolean binary = false;
    for (int i = 0; i < packetSize; i++) {
      packet[i] = (char)LoRa.read();

      if (packet[i] < 32 || packet[i] > 127)
        binary = true;
    }

    packet[packetSize] = 0;
    uint8_t deviceId = _determineDevice(packet, packetSize);
    if (deviceId == WIPHONE) {
      _WiPhoneToMQTT(packet, LORAdata);
    } else if (binary) {
      if (LORAConfig.onlyKnown) {
        Log.trace(F("Ignoring non identifiable packet" CR));
        return;
      }

      char hex[packetSize * 2 + 1];
      _rawToHex(packet, hex, packetSize);

      hex[packetSize * 2] = 0;

      LORAdata["hex"] = hex;
    } else {

      std::string packetStrStd = (char*)packet;
      auto result = deserializeJson(LORAdataBuffer, packetStrStd);
      if (result) {
        Log.notice(F("LORA packet deserialization failed, not a json, sending raw message" CR));
        LORAdata = LORAdataBuffer.to<JsonObject>();
        LORAdata["message"] = (char*)packet;
      } else {
        Log.trace(F("LORA packet deserialization OK" CR));
      }
    }

    LORAdata["rssi"] = (int)LoRa.packetRssi();
    LORAdata["snr"] = (float)LoRa.packetSnr();
    LORAdata["pferror"] = (float)LoRa.packetFrequencyError();
    LORAdata["packetSize"] = (int)packetSize;

    if (LORAdata.containsKey("id")) {
      std::string id = LORAdata["id"];
      id.erase(std::remove(id.begin(), id.end(), ':'), id.end());
# ifdef ZmqttDiscovery
      if (SYSConfig.discovery) {
        if (!LORAdata.containsKey("model"))
          LORAdataBuffer["model"] = "LORA_NODE";
        storeLORADiscovery(LORAdata, LORAdata["model"].as<char*>(), id.c_str());
      }
# endif
      buildTopicFromId(LORAdata, subjectLORAtoMQTT);
    } else {
      LORAdataBuffer["origin"] = subjectLORAtoMQTT;
    }

    handleJsonEnqueue(LORAdata);
    if (repeatLORAwMQTT) {
      Log.trace(F("Pub LORA for rpt" CR));
      LORAdata["origin"] = subjectMQTTtoLORA;
      handleJsonEnqueue(LORAdata);
    }
  }
}

# if jsonReceiving
void MQTTtoLORA(char* topicOri, JsonObject& LORAdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoLORA)) {
    Log.trace(F("MQTTtoLORA json" CR));
    const char* message = LORAdata["message"];
    const char* hex = LORAdata["hex"];
    LORAConfig_fromJson(LORAdata);
    if (message || hex) {
      LoRa.beginPacket();
      uint8_t deviceId = _determineDevice(LORAdata);
      if (deviceId == WIPHONE) {
        _MQTTtoWiPhone(LORAdata);
      } else if (hex) {

        byte raw[strlen(hex) / 2];
        _hexToRaw(hex, raw, sizeof(raw));
        LoRa.write((uint8_t*)raw, sizeof(raw));
      } else {

        LoRa.print(message);
      }

      LoRa.endPacket();
      Log.trace(F("MQTTtoLORA OK" CR));

      pub(subjectGTWLORAtoMQTT, LORAdata);
    } else {
      Log.error(F("MQTTtoLORA Fail json" CR));
    }
  }
  if (cmpToMainTopic(topicOri, subjectMQTTtoLORAset)) {
    Log.trace(F("MQTTtoLORA json set" CR));






    if (LORAdata.containsKey("init") && LORAdata["init"].as<bool>()) {

      LORAConfig_init();
    } else if (LORAdata.containsKey("load") && LORAdata["load"].as<bool>()) {

      LORAConfig_load();
    }


    LORAConfig_fromJson(LORAdata);
    stateLORAMeasures();
  }
}
# endif
# if simpleReceiving
void MQTTtoLORA(char* topicOri, char* LORAarray) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoLORA)) {
    LoRa.beginPacket();
    LoRa.print(LORAarray);
    LoRa.endPacket();
    Log.notice(F("MQTTtoLORA OK" CR));

    pub(subjectGTWLORAtoMQTT, LORAarray);
  }
}
# endif
String stateLORAMeasures() {

  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject LORAdata = jsonBuffer.to<JsonObject>();
  LORAdata["frequency"] = LORAConfig.frequency;
  LORAdata["txpower"] = LORAConfig.txPower;
  LORAdata["spreadingfactor"] = LORAConfig.spreadingFactor;
  LORAdata["signalbandwidth"] = LORAConfig.signalBandwidth;
  LORAdata["codingrate"] = LORAConfig.codingRateDenominator;
  LORAdata["preamblelength"] = LORAConfig.preambleLength;

  char syncWordHex[5];
  snprintf(syncWordHex, sizeof(syncWordHex), "0x%02X", LORAConfig.syncWord);
  LORAdata["syncword"] = syncWordHex;
  LORAdata["enablecrc"] = LORAConfig.crc;
  LORAdata["invertiq"] = LORAConfig.invertIQ;
  LORAdata["onlyknown"] = LORAConfig.onlyKnown;

  pub(subjectGTWLORAtoMQTT, LORAdata);

  String output;
  serializeJson(LORAdata, output);
  return output;
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayPilight.ino"
# 28 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayPilight.ino"
#include "User_config.h"

#ifdef ZgatewayPilight

# ifdef ZradioCC1101
# include <ELECHOUSE_CC1101_SRC_DRV.h>
# endif

# include <ESPiLight.h>
ESPiLight rf(RF_EMITTER_GPIO);

# ifdef Pilight_rawEnabled

bool pilightRawEnabled = 0;
# endif

void pilightCallback(const String& protocol, const String& message, int status,
                     size_t repeats, const String& deviceID) {
  if (status == VALID) {
    Log.trace(F("Creating RF PiLight buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> RFPiLightdataBuffer;
    JsonObject RFPiLightdata = RFPiLightdataBuffer.to<JsonObject>();
    StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer2;
    JsonObject msg = jsonBuffer2.to<JsonObject>();
    if (message.length() > 0) {
      auto error = deserializeJson(jsonBuffer2, message);
      if (error) {
        Log.error(F("deserializeJson() failed: %s" CR), error.c_str());
        return;
      }
      RFPiLightdata["message"] = msg;
    }
    if (protocol.length() > 0) {
      RFPiLightdata["protocol"] = protocol;
    }
    if (deviceID.length() > 0) {
      RFPiLightdata["value"] = deviceID;
      const char* device_id = deviceID.c_str();
      if (!strlen(device_id) && !msg.isNull()) {


        char* choices[] = {"key", "unit", "device_id", "systemcode", "unitcode", "programcode"};

        for (uint8_t i = 0; i < 6; i++) {
          if (msg[choices[i]]) {
            device_id = (const char*)msg[choices[i]];
            break;
          }
        }
      }
      RFPiLightdata["value"] = device_id;
    }

    RFPiLightdata["repeats"] = (int)repeats;
    RFPiLightdata["status"] = (int)status;
    RFPiLightdata["origin"] = subjectPilighttoMQTT;
    handleJsonEnqueue(RFPiLightdata);
    if (repeatPilightwMQTT) {
      Log.trace(F("Pub Pilight for rpt" CR));
      RFPiLightdata["origin"] = subjectMQTTtoPilight;
      handleJsonEnqueue(RFPiLightdata);
    }
  }
}

# ifdef Pilight_rawEnabled
void pilightRawCallback(const uint16_t* pulses, size_t length) {
  uint16_t pulse;

  if (!pilightRawEnabled) {
    Log.trace(F("Pilight RAW not enabled" CR));
    return;
  }

  StaticJsonDocument<JSON_MSG_BUFFER> RFPiLightdataBuffer;
  JsonObject RFPiLightdata = RFPiLightdataBuffer.to<JsonObject>();

  RFPiLightdata["format"] = "RAW";
  RFPiLightdata["rawlen"] = length;
  RFPiLightdata["pulsesString"] = rf.pulseTrainToString(pulses, length);


  RFPiLightdata["origin"] = subjectPilighttoMQTT;
  handleJsonEnqueue(RFPiLightdata);
}
# endif

void savePilightConfig() {
  Log.trace(F("saving Pilight config" CR));
  DynamicJsonDocument json(4096);
  deserializeJson(json, rf.enabledProtocols());

  File configFile = SPIFFS.open("/pilight.json", "w");
  if (!configFile) {
    Log.error(F("failed to open config file for writing" CR));
  }

  serializeJsonPretty(json, Serial);
  serializeJson(json, configFile);
  configFile.close();
}

void loadPilightConfig() {
  Log.trace(F("reading Pilight config file" CR));
  File configFile = SPIFFS.open("/pilight.json", "r");
  if (configFile) {
    Log.trace(F("opened Pilight config file" CR));
    DynamicJsonDocument json(configFile.size() * 4);
    auto error = deserializeJson(json, configFile);
    if (error) {
      Log.error(F("deserialize config failed: %s, buffer capacity: %u" CR), error.c_str(), json.capacity());
    }
    serializeJson(json, Serial);
    if (!json.isNull()) {
      String rflimit;
      serializeJson(json, rflimit);
      rf.limitProtocols(rflimit);
    } else {
      Log.warning(F("failed to load json config" CR));
    }
    configFile.close();
  }
}

void PilighttoMQTT() {
  rf.loop();
}

void MQTTtoPilight(char* topicOri, JsonObject& Pilightdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoPilightProtocol)) {
    bool success = false;
    if (Pilightdata.containsKey("reset")) {
      rf.limitProtocols(rf.availableProtocols());
      savePilightConfig();
      success = true;
    }
    if (Pilightdata.containsKey("limit")) {
      String output;
      serializeJson(Pilightdata["limit"], output);
      rf.limitProtocols(output);
      savePilightConfig();
      success = true;
    }
    if (Pilightdata.containsKey("enabled")) {
      Log.notice(F("PiLight protocols enabled: %s" CR), rf.enabledProtocols().c_str());
      success = true;
    }
    if (Pilightdata.containsKey("available")) {
      Log.notice(F("PiLight protocols available: %s" CR), rf.availableProtocols().c_str());
      success = true;
    }
# ifdef Pilight_rawEnabled
    if (Pilightdata.containsKey("rawEnabled")) {
      Log.notice(F("Setting PiLight raw output enabled: %T" CR), (bool)Pilightdata["rawEnabled"]);
      pilightRawEnabled = (bool)Pilightdata["rawEnabled"];
      disablePilightReceive();
      delay(1);
      enablePilightReceive();
      success = true;
    }
# endif

    if (success) {

      pub(subjectGTWPilighttoMQTT, Pilightdata);
    } else {
      pub(subjectGTWPilighttoMQTT, "{\"Status\": \"Error\"}");
      Log.error(F("MQTTtoPilightProtocol Fail json" CR));
    }
  } else if (cmpToMainTopic(topicOri, subjectMQTTtoPilight)) {
    const char* message = Pilightdata["message"];
    Log.notice(F("MQTTtoPilight message: %s" CR), message);
    const char* protocol = Pilightdata["protocol"];
    Log.notice(F("MQTTtoPilight protocol: %s" CR), protocol);
    const char* raw = Pilightdata["raw"];
    float txFrequency = Pilightdata["frequency"] | RFConfig.frequency;
    bool success = false;
    disableCurrentReceiver();
    initCC1101();
# ifdef ZradioCC1101
    ELECHOUSE_cc1101.SetTx(txFrequency);
    Log.notice(F("Transmit frequency: %F" CR), txFrequency);
# endif
    pinMode(RF_EMITTER_GPIO, OUTPUT);
    if (raw) {
      uint16_t codes[MAXPULSESTREAMLENGTH];
      int repeats = rf.stringToRepeats(raw);
      if (repeats < 0) {
        switch (repeats) {
          case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_R:
            Log.trace(F("'r' not found in string, or has no data" CR));
            break;
          case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_END:
            Log.trace(F("';' or '@' not found in data string" CR));
            break;
        }
        repeats = 10;
      }
      int msgLength = rf.stringToPulseTrain(raw, codes, MAXPULSESTREAMLENGTH);
      if (msgLength > 0) {
        rf.sendPulseTrain(codes, msgLength, repeats);
        Log.notice(F("MQTTtoPilight raw ok" CR));
        success = true;
      } else {
        Log.trace(F("MQTTtoPilight raw KO" CR));
        switch (msgLength) {
          case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_C:
            Log.trace(F("'c' not found in string, or has no data" CR));
            break;
          case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_P:
            Log.trace(F("'p' not found in string, or has no data" CR));
            break;
          case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_END:
            Log.trace(F("';' or '@' not found in data string" CR));
            break;
          case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_TYPE:
            Log.trace(F("pulse type not defined" CR));
            break;
        }
        Log.error(F("Invalid JSON: raw data malformed" CR));
      }
    }
    if (message && protocol) {
      Log.trace(F("MQTTtoPilight msg & protocol ok" CR));
      int msgLength = rf.send(protocol, message);
      if (msgLength > 0) {
        Log.trace(F("Adv data MQTTtoPilight push state via PilighttoMQTT" CR));

        pub(subjectGTWPilighttoMQTT, message);
        success = true;
      } else {
        switch (msgLength) {
          case ESPiLight::ERROR_UNAVAILABLE_PROTOCOL:
            Log.error(F("protocol is not available" CR));
            break;
          case ESPiLight::ERROR_INVALID_PILIGHT_MSG:
            Log.error(F("message is invalid" CR));
            break;
          case ESPiLight::ERROR_INVALID_JSON:
            Log.error(F("message is not a proper json object" CR));
            break;
          case ESPiLight::ERROR_NO_OUTPUT_PIN:
            Log.error(F("no transmitter pin" CR));
            break;
          default:
            Log.error(F("Invalid JSON: can't read message/protocol" CR));
        }
      }
    }
    if (!success) {
      pub(subjectGTWPilighttoMQTT, "{\"Status\": \"Error\"}");
      Log.error(F("MQTTtoPilight Fail json" CR));
    }
    enableActiveReceiver();
  }
}

extern void disablePilightReceive() {
  Log.trace(F("disablePilightReceive" CR));
  rf.initReceiver(-1);
  rf.disableReceiver();
};

extern void enablePilightReceive() {
  Log.notice(F("Switching to Pilight Receiver: %F" CR), RFConfig.frequency);
  Log.notice(F("RF_EMITTER_GPIO: %d " CR), RF_EMITTER_GPIO);
  Log.notice(F("RF_RECEIVER_GPIO: %d " CR), RF_RECEIVER_GPIO);
  Log.trace(F("ZgatewayPilight command topic: %s%s%s" CR), mqtt_topic, gateway_name, subjectMQTTtoPilight);

  initCC1101();

  rf.setCallback(pilightCallback);
# ifdef Pilight_rawEnabled
  if (pilightRawEnabled) {
    rf.setPulseTrainCallBack(pilightRawCallback);
  }
# endif
  rf.initReceiver(RF_RECEIVER_GPIO);
  pinMode(RF_EMITTER_GPIO, OUTPUT);
  rf.enableReceiver();
  loadPilightConfig();
  Log.trace(F("ZgatewayPilight setup done " CR));
};
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRF.ino"
# 28 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRF.ino"
#include "User_config.h"

#ifdef ZgatewayRF

# ifdef ZradioCC1101
# include <ELECHOUSE_CC1101_SRC_DRV.h>
# endif

# include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();


static const char* bin2tristate(const char* bin) {
  static char returnValue[50];
  int pos = 0;
  int pos2 = 0;
  while (bin[pos] != '\0' && bin[pos + 1] != '\0') {
    if (bin[pos] == '0' && bin[pos + 1] == '0') {
      returnValue[pos2] = '0';
    } else if (bin[pos] == '1' && bin[pos + 1] == '1') {
      returnValue[pos2] = '1';
    } else if (bin[pos] == '0' && bin[pos + 1] == '1') {
      returnValue[pos2] = 'F';
    } else {
      return "-";
    }
    pos = pos + 2;
    pos2++;
  }
  returnValue[pos2] = '\0';
  return returnValue;
}

static char* dec2binWzerofill(unsigned long Dec, unsigned int bitLength) {
  static char bin[64];
  unsigned int i = 0;

  while (Dec > 0) {
    bin[32 + i++] = ((Dec & 1) > 0) ? '1' : '0';
    Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j < bitLength; j++) {
    if (j >= bitLength - i) {
      bin[j] = bin[31 + i - (j - (bitLength - i))];
    } else {
      bin[j] = '0';
    }
  }
  bin[bitLength] = '\0';

  return bin;
}

# if defined(ZmqttDiscovery) && !defined(RF_DISABLE_TRANSMIT) && defined(RFmqttDiscovery)

void RFtoMQTTdiscovery(SIGNAL_SIZE_UL_ULL MQTTvalue) {

  char val[11];
  sprintf(val, "%lu", MQTTvalue);
  Log.trace(F("RF Entity Discovered, create HA Discovery CFG" CR));
  char* switchRF[2] = {val, "RF"};
  Log.trace(F("CreateDiscoverySwitch: %s" CR), switchRF[1]);
# if valueAsATopic
  String discovery_topic = String(subjectRFtoMQTT) + "/" + String(switchRF[0]);
# else
  String discovery_topic = String(subjectRFtoMQTT);
# endif

  String theUniqueId = getUniqueId("-" + String(switchRF[0]), "-" + String(switchRF[1]));

  announceDeviceTrigger(
      false,
      (char*)discovery_topic.c_str(),
      "", "",
      (char*)theUniqueId.c_str(),
      "", "", "", "");
}
# endif

void RFtoMQTT() {
  if (mySwitch.available()) {
    StaticJsonDocument<JSON_MSG_BUFFER> RFdataBuffer;
    JsonObject RFdata = RFdataBuffer.to<JsonObject>();
    Log.trace(F("Rcv. RF" CR));
# ifdef ESP32
    Log.trace(F("RF Task running on core :%d" CR), xPortGetCoreID());
# endif
    SIGNAL_SIZE_UL_ULL MQTTvalue = mySwitch.getReceivedValue();
    int length = mySwitch.getReceivedBitlength();
    const char* binary = dec2binWzerofill(MQTTvalue, length);

    RFdata["value"] = (SIGNAL_SIZE_UL_ULL)MQTTvalue;
    RFdata["protocol"] = (int)mySwitch.getReceivedProtocol();
    RFdata["length"] = (int)mySwitch.getReceivedBitlength();
    RFdata["delay"] = (int)mySwitch.getReceivedDelay();
    RFdata["tre_state"] = bin2tristate(binary);
    RFdata["binary"] = binary;

# if defined(ESP32) || defined(ESP8266)
    unsigned int* raw = mySwitch.getReceivedRawdata();
    std::string rawDump;
    for (unsigned int i = 0; i < length * 2; i++) {
      if (i != 0)
        rawDump += ",";
      rawDump += std::to_string(raw[i]);
    }
    RFdata["raw"] = rawDump;
# endif
# ifdef ZradioCC1101
    RFdata["frequency"] = RFConfig.frequency;
# endif
    mySwitch.resetAvailable();

    if (!isAduplicateSignal(MQTTvalue) && MQTTvalue != 0) {
# if defined(ZmqttDiscovery) && !defined(RF_DISABLE_TRANSMIT) && defined(RFmqttDiscovery)
      if (SYSConfig.discovery)
        RFtoMQTTdiscovery(MQTTvalue);
# endif
      RFdata["origin"] = subjectRFtoMQTT;
      handleJsonEnqueue(RFdata);

      Log.trace(F("Store val: %u" CR), (unsigned long)MQTTvalue);
      storeSignalValue(MQTTvalue);
      if (repeatRFwMQTT) {
        Log.trace(F("Pub RF for rpt" CR));
        RFdata["origin"] = subjectMQTTtoRF;
        handleJsonEnqueue(RFdata);
      }
    }
  }
}

# if simpleReceiving
void MQTTtoRF(char* topicOri, char* datacallback) {
# ifdef ZradioCC1101
  disableCurrentReceiver();
  ELECHOUSE_cc1101.SetTx(RFConfig.frequency);
  Log.notice(F("Transmit frequency: %F" CR), RFConfig.frequency);
# endif
  mySwitch.disableReceive();
  mySwitch.enableTransmit(RF_EMITTER_GPIO);
  SIGNAL_SIZE_UL_ULL data = STRTO_UL_ULL(datacallback, NULL, 10);



  String topic = topicOri;
  int valuePRT = 0;
  int valuePLSL = 0;
  int valueBITS = 0;
  int pos = topic.lastIndexOf(RFprotocolKey);
  if (pos != -1) {
    pos = pos + +strlen(RFprotocolKey);
    valuePRT = (topic.substring(pos, pos + 1)).toInt();
  }

  int pos2 = topic.lastIndexOf(RFpulselengthKey);
  if (pos2 != -1) {
    pos2 = pos2 + strlen(RFpulselengthKey);
    valuePLSL = (topic.substring(pos2, pos2 + 3)).toInt();
  }
  int pos3 = topic.lastIndexOf(RFbitsKey);
  if (pos3 != -1) {
    pos3 = pos3 + strlen(RFbitsKey);
    valueBITS = (topic.substring(pos3, pos3 + 2)).toInt();
  }

  if ((cmpToMainTopic(topicOri, subjectMQTTtoRF)) && (valuePRT == 0) && (valuePLSL == 0) && (valueBITS == 0)) {
    Log.trace(F("MQTTtoRF dflt" CR));
    mySwitch.setProtocol(1, 350);
    mySwitch.send(data, 24);

    pub(subjectGTWRFtoMQTT, datacallback);
  } else if ((valuePRT != 0) || (valuePLSL != 0) || (valueBITS != 0)) {
    Log.trace(F("MQTTtoRF usr par." CR));
    if (valuePRT == 0)
      valuePRT = 1;
    if (valuePLSL == 0)
      valuePLSL = 350;
    if (valueBITS == 0)
      valueBITS = 24;
    Log.notice(F("RF Protocol:%d" CR), valuePRT);
    Log.notice(F("RF Pulse Lgth: %d" CR), valuePLSL);
    Log.notice(F("Bits nb: %d" CR), valueBITS);
    mySwitch.setProtocol(valuePRT, valuePLSL);
    mySwitch.send(data, valueBITS);

    pub(subjectGTWRFtoMQTT, datacallback);
  }
# ifdef ZradioCC1101
  ELECHOUSE_cc1101.SetRx(RFConfig.frequency);
  mySwitch.disableTransmit();
  mySwitch.enableReceive(RF_RECEIVER_GPIO);
# endif
}
# endif

# if jsonReceiving
void MQTTtoRF(char* topicOri, JsonObject& RFdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoRF)) {
    Log.trace(F("MQTTtoRF json" CR));
    SIGNAL_SIZE_UL_ULL data = RFdata["value"];
    if (data != 0) {
      int valuePRT = RFdata["protocol"] | 1;
      int valuePLSL = RFdata["delay"] | 350;
      int valueBITS = RFdata["length"] | 24;
      int valueRPT = RFdata["repeat"] | RF_EMITTER_REPEAT;
      Log.notice(F("RF Protocol:%d" CR), valuePRT);
      Log.notice(F("RF Pulse Lgth: %d" CR), valuePLSL);
      Log.notice(F("Bits nb: %d" CR), valueBITS);
# ifdef ZradioCC1101
      disableCurrentReceiver();
      initCC1101();
      int txPower = RFdata["txpower"] | RF_CC1101_TXPOWER;
      ELECHOUSE_cc1101.setPA((int)txPower);
      Log.notice(F("CC1101 TX Power: %d" CR), txPower);
      float txFrequency = RFdata["frequency"] | RFConfig.frequency;
      ELECHOUSE_cc1101.SetTx(txFrequency);
      Log.notice(F("Transmit frequency: %F" CR), txFrequency);
# endif
      mySwitch.enableTransmit(RF_EMITTER_GPIO);
      mySwitch.setRepeatTransmit(valueRPT);
      mySwitch.setProtocol(valuePRT, valuePLSL);
      mySwitch.send(data, valueBITS);
      Log.notice(F("MQTTtoRF OK" CR));

      pub(subjectGTWRFtoMQTT, RFdata);
      mySwitch.setRepeatTransmit(RF_EMITTER_REPEAT);
    }

    enableActiveReceiver();
  }
}
# endif

int receiveInterupt = -1;

void disableRFReceive() {
  Log.trace(F("disableRFReceive %d" CR), receiveInterupt);
  mySwitch.disableReceive();
}

void enableRFReceive() {
  Log.notice(F("Enable RF Receiver: %FMhz" CR), RFConfig.frequency);

  Log.notice(F("RF_EMITTER_GPIO: %d " CR), RF_EMITTER_GPIO);
  Log.notice(F("RF_RECEIVER_GPIO: %d " CR), RF_RECEIVER_GPIO);

# ifdef RF_DISABLE_TRANSMIT
  mySwitch.disableTransmit();
# else
  mySwitch.enableTransmit(RF_EMITTER_GPIO);
# endif
  receiveInterupt = RF_RECEIVER_GPIO;
  mySwitch.setRepeatTransmit(RF_EMITTER_REPEAT);
  mySwitch.enableReceive(receiveInterupt);
  Log.trace(F("ZgatewayRF command topic: %s%s%s" CR), mqtt_topic, gateway_name, subjectMQTTtoRF);
  Log.trace(F("ZgatewayRF setup done" CR));
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRF2.ino"
# 36 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRF2.ino"
#include "User_config.h"

#ifdef ZgatewayRF2

# ifdef ZradioCC1101
# include <ELECHOUSE_CC1101_SRC_DRV.h>
# endif

# include <NewRemoteReceiver.h>
# include <NewRemoteTransmitter.h>

struct RF2rxd {
  unsigned int period;
  unsigned long address;
  unsigned long groupBit;
  unsigned long unit;
  unsigned long switchType;
  bool hasNewData;
};

RF2rxd rf2rd;

# ifdef ZmqttDiscovery

void RF2toMQTTdiscovery(JsonObject& data) {
  Log.trace(F("switchRF2Discovery" CR));
  String payloadonstr;
  String payloadoffstr;

  int org_switchtype = data["switchType"];
  data["switchType"] = 1;
  serializeJson(data, payloadonstr);
  data["switchType"] = 0;
  serializeJson(data, payloadoffstr);
  data["switchType"] = org_switchtype;

  String switchname;
  switchname = "RF2_" + String((int)data["unit"]) + "_" +
               String((int)data["groupbit"]) + "_" +
               String((unsigned long)data["address"]);

  char* switchRF[8] = {"switch",
                       (char*)switchname.c_str(),
                       "",
                       "",
                       "",
                       (char*)payloadonstr.c_str(),
                       (char*)payloadoffstr.c_str(),
                       ""};



  Log.trace(F("CreateDiscoverySwitch: %s" CR), switchRF[1]);






  createDiscovery(switchRF[0], "", switchRF[1],
                  (char*)getUniqueId(switchRF[1], "").c_str(), will_Topic,
                  switchRF[3], switchRF[4], switchRF[5], switchRF[6],
                  switchRF[7], 0, "", "", true, subjectMQTTtoRF2,
                  "", "", "", "", false,
                  stateClassNone);
}
# endif

void RF2toMQTT() {
  if (rf2rd.hasNewData) {
    StaticJsonDocument<JSON_MSG_BUFFER> RF2dataBuffer;
    JsonObject RF2data = RF2dataBuffer.to<JsonObject>();
    rf2rd.hasNewData = false;

    Log.trace(F("Rcv. RF2" CR));
    RF2data["unit"] = (int)rf2rd.unit;
    RF2data["groupBit"] = (int)rf2rd.groupBit;
    RF2data["period"] = (int)rf2rd.period;
    RF2data["address"] = (unsigned long)rf2rd.address;
    RF2data["switchType"] = (int)rf2rd.switchType;
# ifdef ZmqttDiscovery
    if (SYSConfig.discovery)
      RF2toMQTTdiscovery(RF2data);
# endif
    RF2data["origin"] = subjectRF2toMQTT;
    handleJsonEnqueue(RF2data);
  }
}

void rf2Callback(unsigned int period, unsigned long address, unsigned long groupBit, unsigned long unit, unsigned long switchType) {
  rf2rd.period = period;
  rf2rd.address = address;
  rf2rd.groupBit = groupBit;
  rf2rd.unit = unit;
  rf2rd.switchType = switchType;
  rf2rd.hasNewData = true;
}

# if simpleReceiving
void MQTTtoRF2(char* topicOri, char* datacallback) {
  NewRemoteReceiver::disable();
  pinMode(RF_EMITTER_GPIO, OUTPUT);
  initCC1101();



  String topic = topicOri;
  bool boolSWITCHTYPE;
  boolSWITCHTYPE = to_bool(datacallback);
  bool isDimCommand = false;

  long valueCODE = 0;
  int valueUNIT = -1;
  int valuePERIOD = 0;
  int valueGROUP = 0;
  int valueDIM = -1;

  int pos = topic.lastIndexOf(RF2codeKey);
  if (pos != -1) {
    pos = pos + +strlen(RF2codeKey);
    valueCODE = (topic.substring(pos, pos + 8)).toInt();
    Log.notice(F("RF2 code: %l" CR), valueCODE);
  }
  int pos2 = topic.lastIndexOf(RF2periodKey);
  if (pos2 != -1) {
    pos2 = pos2 + strlen(RF2periodKey);
    valuePERIOD = (topic.substring(pos2, pos2 + 3)).toInt();
    Log.notice(F("RF2 Period: %d" CR), valuePERIOD);
  }
  int pos3 = topic.lastIndexOf(RF2unitKey);
  if (pos3 != -1) {
    pos3 = pos3 + strlen(RF2unitKey);
    valueUNIT = (topic.substring(pos3, topic.indexOf("/", pos3))).toInt();
    Log.notice(F("Unit: %d" CR), valueUNIT);
  }
  int pos4 = topic.lastIndexOf(RF2groupKey);
  if (pos4 != -1) {
    pos4 = pos4 + strlen(RF2groupKey);
    valueGROUP = (topic.substring(pos4, pos4 + 1)).toInt();
    Log.notice(F("RF2 Group: %d" CR), valueGROUP);
  }
  int pos5 = topic.lastIndexOf(RF2dimKey);
  if (pos5 != -1) {
    isDimCommand = true;
    valueDIM = atoi(datacallback);
    Log.notice(F("RF2 Dim: %d" CR), valueDIM);
  }

  if ((topic == subjectMQTTtoRF2) || (valueCODE != 0) || (valueUNIT != -1) || (valuePERIOD != 0)) {
    Log.trace(F("MQTTtoRF2" CR));
    if (valueCODE == 0)
      valueCODE = 8233378;
    if (valueUNIT == -1)
      valueUNIT = 0;
    if (valuePERIOD == 0)
      valuePERIOD = 272;
    NewRemoteReceiver::disable();
    Log.trace(F("Creating transmitter" CR));
    NewRemoteTransmitter transmitter(valueCODE, RF_EMITTER_GPIO, valuePERIOD, RF2_EMITTER_REPEAT);
    Log.trace(F("Sending data" CR));
    if (valueGROUP) {
      if (isDimCommand) {
        transmitter.sendGroupDim(valueDIM);
      } else {
        transmitter.sendGroup(boolSWITCHTYPE);
      }
    } else {
      if (isDimCommand) {
        transmitter.sendDim(valueUNIT, valueDIM);
      } else {
        transmitter.sendUnit(valueUNIT, boolSWITCHTYPE);
      }
    }
    Log.trace(F("Data sent" CR));
    NewRemoteReceiver::enable();


    String MQTTAddress;
    String MQTTperiod;
    String MQTTunit;
    String MQTTgroupBit;
    String MQTTswitchType;
    String MQTTdimLevel;

    MQTTAddress = String(valueCODE);
    MQTTperiod = String(valuePERIOD);
    MQTTunit = String(valueUNIT);
    MQTTgroupBit = String(rf2rd.groupBit);
    MQTTswitchType = String(boolSWITCHTYPE);
    MQTTdimLevel = String(valueDIM);
    String MQTTRF2string;
    Log.trace(F("Adv data MQTTtoRF2 push state via RF2toMQTT" CR));
    if (isDimCommand) {
      MQTTRF2string = subjectRF2toMQTT + String("/") + RF2codeKey + MQTTAddress + String("/") + RF2unitKey + MQTTunit + String("/") + RF2groupKey + MQTTgroupBit + String("/") + RF2dimKey + String("/") + RF2periodKey + MQTTperiod;
      pub((char*)MQTTRF2string.c_str(), (char*)MQTTdimLevel.c_str());
    } else {
      MQTTRF2string = subjectRF2toMQTT + String("/") + RF2codeKey + MQTTAddress + String("/") + RF2unitKey + MQTTunit + String("/") + RF2groupKey + MQTTgroupBit + String("/") + RF2periodKey + MQTTperiod;
      pub((char*)MQTTRF2string.c_str(), (char*)MQTTswitchType.c_str());
    }
  }
# ifdef ZradioCC1101
  ELECHOUSE_cc1101.SetRx(RFConfig.frequency);
  NewRemoteReceiver::enable();
# endif
}
# endif

# if jsonReceiving
void MQTTtoRF2(char* topicOri, JsonObject& RF2data) {

  if (cmpToMainTopic(topicOri, subjectMQTTtoRF2)) {
    Log.trace(F("MQTTtoRF2 json" CR));
    int boolSWITCHTYPE = RF2data["switchType"] | 99;
    bool success = false;
    if (boolSWITCHTYPE != 99) {
      NewRemoteReceiver::disable();
      pinMode(RF_EMITTER_GPIO, OUTPUT);
      initCC1101();
      Log.trace(F("MQTTtoRF2 switch type ok" CR));
      bool isDimCommand = boolSWITCHTYPE == 2;
      unsigned long valueCODE = RF2data["address"];
      int valueUNIT = RF2data["unit"] | -1;
      int valuePERIOD = RF2data["period"];
      int valueGROUP = RF2data["group"];
      int valueDIM = RF2data["dim"] | -1;
      if ((valueCODE != 0) || (valueUNIT != -1) || (valuePERIOD != 0)) {
        Log.trace(F("MQTTtoRF2" CR));
        if (valueCODE == 0)
          valueCODE = 8233378;
        if (valueUNIT == -1)
          valueUNIT = 0;
        if (valuePERIOD == 0)
          valuePERIOD = 272;
        NewRemoteReceiver::disable();
        NewRemoteTransmitter transmitter(valueCODE, RF_EMITTER_GPIO, valuePERIOD, RF2_EMITTER_REPEAT);
        Log.trace(F("Sending" CR));
        if (valueGROUP) {
          if (isDimCommand) {
            transmitter.sendGroupDim(valueDIM);
          } else {
            transmitter.sendGroup(boolSWITCHTYPE);
          }
        } else {
          if (isDimCommand) {
            transmitter.sendDim(valueUNIT, valueDIM);
          } else {
            transmitter.sendUnit(valueUNIT, boolSWITCHTYPE);
          }
        }
        Log.notice(F("MQTTtoRF2 OK" CR));
        NewRemoteReceiver::enable();

        success = true;
      }
    }
    if (success) {

      pub(subjectRF2toMQTT, RF2data);
    } else {
# ifndef ARDUINO_AVR_UNO
      pub(subjectGTWRF2toMQTT, "{\"Status\": \"Error\"}");
# endif
      Log.error(F("MQTTtoRF2 failed json read" CR));
    }
    enableActiveReceiver();
  }
}
# endif

void disableRF2Receive() {
  Log.trace(F("disableRF2Receive" CR));
  NewRemoteReceiver::disable();
}

void enableRF2Receive() {
  Log.trace(F("enableRF2Receive" CR));
  NewRemoteReceiver::init(RF_RECEIVER_GPIO, 2, rf2Callback);

  Log.notice(F("RF_EMITTER_GPIO: %d " CR), RF_EMITTER_GPIO);
  Log.notice(F("RF_RECEIVER_GPIO: %d " CR), RF_RECEIVER_GPIO);
  Log.trace(F("ZgatewayRF2 command topic: %s%s%s" CR), mqtt_topic, gateway_name, subjectMQTTtoRF2);
  pinMode(RF_EMITTER_GPIO, OUTPUT);
  digitalWrite(RF_EMITTER_GPIO, LOW);
  Log.trace(F("ZgatewayRF2 setup done " CR));
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRFM69.ino"
# 31 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRFM69.ino"
#include "User_config.h"

#ifdef ZgatewayRFM69

# include <EEPROM.h>
# include <RFM69.h>

char RadioConfig[128];



struct _GLOBAL_CONFIG {
  uint32_t checksum;
  char rfmapname[32];
  char encryptkey[16 + 1];
  uint8_t networkid;
  uint8_t nodeid;
  uint8_t powerlevel;
  uint8_t rfmfrequency;
};

#define GC_POWER_LEVEL (pGC->powerlevel & 0x1F)
#define GC_IS_RFM69HCW ((pGC->powerlevel & 0x80) != 0)
#define SELECTED_FREQ(f) ((pGC->rfmfrequency == f) ? "selected" : "")

struct _GLOBAL_CONFIG* pGC;


uint32_t gc_checksum() {
  uint8_t* p = (uint8_t*)pGC;
  uint32_t checksum = 0;
  p += sizeof(pGC->checksum);
  for (size_t i = 0; i < (sizeof(*pGC) - 4); i++) {
    checksum += *p++;
  }
  return checksum;
}

# if defined(ESP8266) || defined(ESP32)
void eeprom_setup() {
  EEPROM.begin(4096);
  pGC = (struct _GLOBAL_CONFIG*)EEPROM.getDataPtr();

  if (gc_checksum() != pGC->checksum) {
    Log.trace(F("Factory reset" CR));
    memset(pGC, 0, sizeof(*pGC));
    strcpy_P(pGC->encryptkey, ENCRYPTKEY);
    strcpy_P(pGC->rfmapname, RFM69AP_NAME);
    pGC->networkid = NETWORKID;
    pGC->nodeid = NODEID;
    pGC->powerlevel = ((IS_RFM69HCW) ? 0x80 : 0x00) | POWER_LEVEL;
    pGC->rfmfrequency = FREQUENCY;
    pGC->checksum = gc_checksum();
    EEPROM.commit();
  }
}
# endif

RFM69 radio;

void setupRFM69(void) {
# if defined(ESP8266) || defined(ESP32)
  eeprom_setup();
# endif
  int freq;
  static const char PROGMEM JSONtemplate[] =
      R"({"msgType":"config","freq":%d,"rfm69hcw":%d,"netid":%d,"power":%d})";
  char payload[128];

  radio = RFM69(RFM69_CS, RFM69_IRQ, GC_IS_RFM69HCW, RFM69_IRQN);


  if (!radio.initialize(pGC->rfmfrequency, pGC->nodeid, pGC->networkid)) {
    Log.error(F("ZgatewayRFM69 initialization failed" CR));
  }

  if (GC_IS_RFM69HCW) {
    radio.setHighPower();
  }
  radio.setPowerLevel(GC_POWER_LEVEL);

  if (pGC->encryptkey[0] != '\0')
    radio.encrypt(pGC->encryptkey);

  switch (pGC->rfmfrequency) {
    case RF69_433MHZ:
      freq = 433;
      break;
    case RF69_868MHZ:
      freq = 868;
      break;
    case RF69_915MHZ:
      freq = 915;
      break;
    case RF69_315MHZ:
      freq = 315;
      break;
    default:
      freq = -1;
      break;
  }
  Log.notice(F("ZgatewayRFM69 Listening and transmitting at: %d" CR), freq);

  size_t len = snprintf_P(RadioConfig, sizeof(RadioConfig), JSONtemplate,
                          freq, GC_IS_RFM69HCW, pGC->networkid, GC_POWER_LEVEL);
  if (len >= sizeof(RadioConfig)) {
    Log.trace(F("\n\n*** RFM69 config truncated ***\n" CR));
  }
}

bool RFM69toMQTT(void) {

  if (radio.receiveDone()) {
    StaticJsonDocument<JSON_MSG_BUFFER> RFM69dataBuffer;
    JsonObject RFM69data = RFM69dataBuffer.to<JsonObject>();
    uint8_t data[RF69_MAX_DATA_LEN + 1];
    uint8_t SENDERID = radio.SENDERID;
    uint8_t DATALEN = radio.DATALEN;
    uint16_t RSSI = radio.RSSI;


    memcpy(data, (void*)radio.DATA, DATALEN);
    data[DATALEN] = '\0';



    if (radio.ACKRequested()) {
      radio.sendACK();
    }


    Log.trace(F("Data received: %s" CR), (const char*)data);

    char buff[sizeof(subjectRFM69toMQTT) + 4];
    sprintf(buff, "%s/%d", subjectRFM69toMQTT, SENDERID);
    RFM69data["data"] = (char*)data;
    RFM69data["rssi"] = (int)radio.RSSI;
    RFM69data["senderid"] = (int)radio.SENDERID;
    pub(buff, RFM69data);

    return true;
  } else {
    return false;
  }
}

# if simpleReceiving
void MQTTtoRFM69(char* topicOri, char* datacallback) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoRFM69)) {
    Log.trace(F("MQTTtoRFM69 data analysis" CR));
    char data[RF69_MAX_DATA_LEN + 1];
    memcpy(data, (void*)datacallback, RF69_MAX_DATA_LEN);
    data[RF69_MAX_DATA_LEN] = '\0';


    String topic = topicOri;
    int valueRCV = defaultRFM69ReceiverId;
    int pos = topic.lastIndexOf(RFM69receiverKey);
    if (pos != -1) {
      pos = pos + +strlen(RFM69receiverKey);
      valueRCV = (topic.substring(pos, pos + 3)).toInt();
      Log.notice(F("RFM69 receiver ID: %d" CR), valueRCV);
    }
    if (radio.sendWithRetry(valueRCV, data, strlen(data), 10)) {
      Log.notice(F(" OK " CR));

      char buff[sizeof(subjectGTWRFM69toMQTT) + 4];
      sprintf(buff, "%s/%d", subjectGTWRFM69toMQTT, radio.SENDERID);
      pub(buff, data);
    } else {
      Log.error(F("RFM69 sending failed" CR));
    }
  }
}
# endif
# if jsonReceiving
void MQTTtoRFM69(char* topicOri, JsonObject& RFM69data) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoRFM69)) {
    const char* data = RFM69data["data"];
    Log.trace(F("MQTTtoRFM69 json data analysis" CR));
    if (data) {
      Log.trace(F("MQTTtoRFM69 data ok" CR));
      int valueRCV = RFM69data["receiverid"] | defaultRFM69ReceiverId;
      Log.notice(F("RFM69 receiver ID: %d" CR), valueRCV);
      if (radio.sendWithRetry(valueRCV, data, strlen(data), 10)) {
        Log.notice(F(" OK " CR));

        RFM69data["origin"] = subjectGTWRFM69toMQTT;
        handleJsonEnqueue(RFM69data);
      } else {
        Log.error(F("MQTTtoRFM69 sending failed" CR));
      }
    } else {
      Log.error(F("MQTTtoRFM69 failed json read" CR));
    }
  }
}
# endif
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRS232.ino"
# 28 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRS232.ino"
#include "User_config.h"

#ifdef ZgatewayRS232

# ifndef RS232_UART
# include <SoftwareSerial.h>
SoftwareSerial RS232SoftSerial(RS232_RX_GPIO, RS232_TX_GPIO);
# endif



Stream* RS232Stream = NULL;

void setupRS232() {

# ifdef RS232_UART
# if RS232_UART == 0
  Serial.end();
# ifdef ESP32
  Serial.begin(RS232Baud, SERIAL_8N1, RS232_RX_GPIO, RS232_TX_GPIO);
# else
  Serial.begin(RS232Baud, SERIAL_8N1);
# endif
# if defined(ESP8266) && defined(RS232_UART0_SWAP)
  Serial.swap();
# endif
  RS232Stream = &Serial;
  Log.notice(F("RS232 HW UART0" CR));

# elif RS232_UART == 1
  Serial1.end();
# ifdef ESP32
  Serial1.begin(RS232Baud, SERIAL_8N1, RS232_RX_GPIO, RS232_TX_GPIO);
# else
  Serial1.begin(RS232Baud, SERIAL_8N1);
# endif
  RS232Stream = &Serial1;
  Log.notice(F("RS232 HW UART1" CR));

# elif RS232_UART == 2
  Serial2.end();
# ifdef ESP32
  Serial2.begin(RS232Baud, SERIAL_8N1, RS232_RX_GPIO, RS232_TX_GPIO);
# else
  Serial2.begin(RS232Baud, SERIAL_8N1);
# endif
  RS232Stream = &Serial2;
  Log.notice(F("RS232 HW UART2" CR));

# elif RS232_UART == 3
  Serial3.end();
  Serial3.begin(RS232Baud, SERIAL_8N1);
  RS232Stream = &Serial3;
  Log.notice(F("RS232 HW UART3" CR));
# endif

# else

  pinMode(RS232_RX_GPIO, INPUT);
  pinMode(RS232_TX_GPIO, OUTPUT);
  RS232SoftSerial.begin(RS232Baud);
  RS232Stream = &RS232SoftSerial;

  Log.notice(F("RS232_RX_GPIO: %d" CR), RS232_RX_GPIO);
  Log.notice(F("RS232_TX_GPIO: %d" CR), RS232_TX_GPIO);
# endif


  while (RS232Stream->available() > 0)
    RS232Stream->read();

  Log.notice(F("RS232Baud: %d" CR), RS232Baud);
  Log.trace(F("ZgatewayRS232 setup done" CR));
}

# if RS232toMQTTmode == 0
void RS232toMQTT() {


  if (RS232Stream->available()) {
    Log.trace(F("RS232toMQTT" CR));
    static char RS232data[MAX_INPUT];
    static unsigned int input_pos = 0;
    static char inChar;
    do {
      if (RS232Stream->available()) {
        inChar = RS232Stream->read();
        RS232data[input_pos] = inChar;
        input_pos++;
        Log.trace(F("Received %c" CR), inChar);
      }
    } while (inChar != RS232InPost && input_pos < MAX_INPUT);
    RS232data[input_pos] = 0;
    input_pos = 0;
    Log.trace(F("Publish %s" CR), RS232data);
    char* output = RS232data + sizeof(RS232Pre) - 1;
    pub(subjectRS232toMQTT, output);
  }
}

# elif RS232toMQTTmode == 1
void RS232toMQTT() {


  if (RS232Stream->available()) {
    Log.trace(F("RS232toMQTT" CR));


    StaticJsonDocument<RS232JSONDocSize> doc;


    DeserializationError err = deserializeJson(doc, *RS232Stream);

    if (err == DeserializationError::Ok) {

      char topic[mqtt_topic_max_size + 1] = subjectRS232toMQTT;
      sendMQTTfromNestedJson(doc.as<JsonVariant>(), topic, 0, RS232maxJSONlevel);
    } else {

      Log.error(F("Error in RS232JSONtoMQTT, deserializeJson() returned %s"), err.c_str());


      while (RS232Stream->available() > 0)
        RS232Stream->read();
    }
  }
}

void sendMQTTfromNestedJson(JsonVariant obj, char* topic, int level, int maxLevel) {

  if (level < maxLevel && obj.is<JsonObject>()) {
    int topicLength = strlen(topic);

    for (JsonPair pair : obj.as<JsonObject>()) {

      const char* key = pair.key().c_str();
      Log.trace(F("level=%d, key='%s'" CR), level, pair.key().c_str());
      if (topicLength + 2 + strlen(key) <= mqtt_topic_max_size) {

        topic[topicLength] = '/';
        topic[topicLength + 1] = '\0';
        strncat(topic + topicLength, key, mqtt_topic_max_size - topicLength - 2);


        sendMQTTfromNestedJson(pair.value(), topic, level + 1, maxLevel);


        topic[topicLength] = '\0';
      } else {
        Log.error(F("Nested key '%s' at level %d does not fit within max topic length of %d, skipping"),
                  key, level, mqtt_topic_max_size);
      }
    }

  } else {

    char output[MAX_INPUT + 1];
    serializeJson(obj, output, MAX_INPUT);
    Log.notice(F("level=%d, topic=%s, value: %s\n"), level, topic, output);


    pub(topic, &output[0]);
  }
}
# endif

void MQTTtoRS232(char* topicOri, JsonObject& RS232data) {
  Log.trace(F("json" CR));
  if (cmpToMainTopic(topicOri, subjectMQTTtoRS232)) {
    Log.trace(F("MQTTtoRS232 json" CR));
    const char* data = RS232data["value"];
    const char* prefix = RS232data["prefix"] | RS232Pre;
    const char* postfix = RS232data["postfix"] | RS232Post;
    Log.trace(F("Value set: %s" CR), data);
    Log.trace(F("Prefix set: %s" CR), prefix);
    Log.trace(F("Postfix set: %s" CR), postfix);
    RS232Stream->print(prefix);
    RS232Stream->print(data);
    RS232Stream->print(postfix);
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRTL_433.ino"
# 29 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayRTL_433.ino"
#ifdef ZgatewayRTL_433

# include <ArduinoJson.h>
# include <config_RF.h>
# include <rtl_433_ESP.h>

# include "ArduinoLog.h"
# include "User_config.h"
# ifdef ZmqttDiscovery
# include "config_mqttDiscovery.h"
# endif

char messageBuffer[JSON_MSG_BUFFER];

# ifdef ZmqttDiscovery
SemaphoreHandle_t semaphorecreateOrUpdateDeviceRTL_433;
std::vector<RTL_433device*> RTL_433devices;
int newRTL_433Devices = 0;

static RTL_433device NO_RTL_433_DEVICE_FOUND = {{0},
                                                0,
                                                false};

RTL_433device* getDeviceById(const char* id);
RTL_433device* getDeviceById(const char* id) {
  DISCOVERY_TRACE_LOG(F("getDeviceById %s" CR), id);

  for (std::vector<RTL_433device*>::iterator it = RTL_433devices.begin(); it != RTL_433devices.end(); ++it) {
    if ((strcmp((*it)->uniqueId, id) == 0)) {
      return *it;
    }
  }
  return &NO_RTL_433_DEVICE_FOUND;
}

void dumpRTL_433Devices() {
  for (std::vector<RTL_433device*>::iterator it = RTL_433devices.begin(); it != RTL_433devices.end(); ++it) {
    RTL_433device* p = *it;
    DISCOVERY_TRACE_LOG(F("uniqueId %s" CR), p->uniqueId);
    DISCOVERY_TRACE_LOG(F("modelName %s" CR), p->modelName);
    DISCOVERY_TRACE_LOG(F("isDisc %d" CR), p->isDisc);
  }
}

void createOrUpdateDeviceRTL_433(const char* id, const char* model, uint8_t flags) {
  if (xSemaphoreTake(semaphorecreateOrUpdateDeviceRTL_433, pdMS_TO_TICKS(30000)) == pdFALSE) {
    Log.error(F("[rtl_433] semaphorecreateOrUpdateDeviceRTL_433 Semaphore NOT taken" CR));
    return;
  }

  RTL_433device* device = getDeviceById(id);
  if (device == &NO_RTL_433_DEVICE_FOUND) {
    DISCOVERY_TRACE_LOG(F("add %s" CR), id);

    device = new RTL_433device();
    if (strlcpy(device->uniqueId, id, uniqueIdSize) > uniqueIdSize) {
      Log.warning(F("[rtl_433] Device id %s exceeds available space" CR), id);
    };
    if (strlcpy(device->modelName, model, modelNameSize) > modelNameSize) {
      Log.warning(F("[rtl_433] Device model %s exceeds available space" CR), id);
    };
    device->isDisc = flags & device_flags_isDisc;
    RTL_433devices.push_back(device);
    newRTL_433Devices++;
  } else {
    DISCOVERY_TRACE_LOG(F("update %s" CR), id);

    if (flags & device_flags_isDisc) {
      device->isDisc = true;
    }
  }

  xSemaphoreGive(semaphorecreateOrUpdateDeviceRTL_433);
}



void launchRTL_433Discovery(bool overrideDiscovery) {
  if (!overrideDiscovery && newRTL_433Devices == 0)
    return;
  if (xSemaphoreTake(semaphorecreateOrUpdateDeviceRTL_433, pdMS_TO_TICKS(QueueSemaphoreTimeOutLoop)) == pdFALSE) {
    Log.error(F("[rtl_433] semaphorecreateOrUpdateDeviceRTL_433 Semaphore NOT taken" CR));
    return;
  }
  newRTL_433Devices = 0;
  std::vector<RTL_433device*> localDevices = RTL_433devices;
  xSemaphoreGive(semaphorecreateOrUpdateDeviceRTL_433);
  for (std::vector<RTL_433device*>::iterator it = localDevices.begin(); it != localDevices.end(); ++it) {
    RTL_433device* pdevice = *it;
    DISCOVERY_TRACE_LOG(F("Device id %s" CR), pdevice->uniqueId);

    if (overrideDiscovery || !isDiscovered(pdevice)) {
      size_t numRows = sizeof(parameters) / sizeof(parameters[0]);
      for (int i = 0; i < numRows; i++) {
        if (strstr(pdevice->uniqueId, parameters[i][0]) != 0) {

          String idWoKey = pdevice->uniqueId;
          idWoKey.remove(idWoKey.length() - (strlen(parameters[i][0]) + 1));
          DISCOVERY_TRACE_LOG(F("idWoKey %s" CR), idWoKey.c_str());
          String value_template = "{{ value_json." + String(parameters[i][0]) + " | is_defined }}";
          if (strcmp(parameters[i][0], "battery_ok") == 0) {
            if (SYSConfig.ohdiscovery) {
              value_template = "{{ value_json." + String(parameters[i][0]) + " * 99 + 1 }}";
            } else {
              value_template = "{{ float(value_json." + String(parameters[i][0]) + ") * 99 + 1 | is_defined }}";
            }
          }
          String topic = subjectRTL_433toMQTT;
# if valueAsATopic

          String idWoKeyAndModel = idWoKey;
          idWoKeyAndModel.remove(0, strlen(pdevice->modelName));
          idWoKeyAndModel.replace("-", "/");
          DISCOVERY_TRACE_LOG(F("idWoKeyAndModel %s" CR), idWoKeyAndModel.c_str());
          topic = topic + "/" + String(pdevice->modelName) + idWoKeyAndModel;
# endif
          if (strcmp(parameters[i][0], "tamper") == 0 || strcmp(parameters[i][0], "alarm") == 0 || strcmp(parameters[i][0], "motion") == 0) {
            createDiscovery("binary_sensor",
                            (char*)topic.c_str(), parameters[i][1], pdevice->uniqueId,
                            "", parameters[i][3], (char*)value_template.c_str(),
                            "1", "0", parameters[i][2],
                            0,
                            "", "", false, "",
                            (char*)idWoKey.c_str(), "", pdevice->modelName, (char*)idWoKey.c_str(), false,
                            ""
            );
          } else if (strcmp(parameters[i][0], "strike_count") == 0) {
            createDiscovery("sensor",
                            (char*)topic.c_str(), parameters[i][1], pdevice->uniqueId,
                            "", parameters[i][3], (char*)value_template.c_str(),
                            "1", "0", parameters[i][2],
                            0,
                            "", "", false, "",
                            (char*)idWoKey.c_str(), "", pdevice->modelName, (char*)idWoKey.c_str(), false,
                            stateClassTotalIncreasing
            );
          } else if (strcmp(parameters[i][0], "event") == 0 && strcmp(pdevice->modelName, "Govee-Water") == 0) {
            createDiscovery("binary_sensor",
                            (char*)topic.c_str(), parameters[i][1], pdevice->uniqueId,
                            "", parameters[i][3], (char*)value_template.c_str(),
                            "Water Leak", "", parameters[i][2],
                            60,
                            "", "", false, "",
                            (char*)idWoKey.c_str(), "Govee", pdevice->modelName, (char*)idWoKey.c_str(), false,
                            stateClassMeasurement
            );
          } else if (strcmp(pdevice->modelName, "Interlogix-Security") != 0) {
            createDiscovery("sensor",
                            (char*)topic.c_str(), parameters[i][1], pdevice->uniqueId,
                            "", parameters[i][3], (char*)value_template.c_str(),
                            "", "", parameters[i][2],
                            0,
                            "", "", false, "",
                            (char*)idWoKey.c_str(), "", pdevice->modelName, (char*)idWoKey.c_str(), false,
                            stateClassMeasurement
            );
          }
          pdevice->isDisc = true;
          dumpRTL_433Devices();
          break;
        }
      }
      if (!pdevice->isDisc) {
        DISCOVERY_TRACE_LOG(F("Device id %s was not discovered" CR), pdevice->uniqueId);
      }
    } else {
      DISCOVERY_TRACE_LOG(F("Device already discovered or that doesn't require discovery %s" CR), pdevice->uniqueId);
    }
  }
}

void storeRTL_433Discovery(JsonObject& RFrtl_433_ESPdata, const char* model, const char* uniqueid) {

  String modelSanitized = model;
  modelSanitized.replace(" ", "_");
  modelSanitized.replace("/", "_");
  modelSanitized.replace(".", "_");
  modelSanitized.replace("&", "");


  size_t numRows = sizeof(parameters) / sizeof(parameters[0]);

  for (int i = 0; i < numRows; i++) {
    if (RFrtl_433_ESPdata.containsKey(parameters[i][0])) {
      String key_id = String(uniqueid) + "-" + String(parameters[i][0]);
      createOrUpdateDeviceRTL_433((char*)key_id.c_str(), (char*)modelSanitized.c_str(), device_flags_init);
    }
  }
}
# endif

void rtl_433_Callback(char* message) {
  DynamicJsonDocument jsonBuffer2(JSON_MSG_BUFFER);
  JsonObject RFrtl_433_ESPdata = jsonBuffer2.to<JsonObject>();
  auto error = deserializeJson(jsonBuffer2, message);
  if (error) {
    Log.error(F("[rtl_433] deserializeJson() failed: %s" CR), error.c_str());
    return;
  }

  unsigned long MQTTvalue = (int)RFrtl_433_ESPdata["id"] + round((float)RFrtl_433_ESPdata["temperature_C"]);
  String topic = subjectRTL_433toMQTT;
  String model = RFrtl_433_ESPdata["model"];
  String uniqueid;

  const char naming_keys[5][8] = {"type", "model", "subtype", "channel", "id"};
  size_t numRows = sizeof(naming_keys) / sizeof(naming_keys[0]);
  for (int i = 0; i < numRows; i++) {
    if (RFrtl_433_ESPdata.containsKey(naming_keys[i])) {
      if (uniqueid == 0) {
        uniqueid = RFrtl_433_ESPdata[naming_keys[i]].as<String>();
      } else {
        uniqueid = uniqueid + "/" + RFrtl_433_ESPdata[naming_keys[i]].as<String>();
      }
    }
  }

# if valueAsATopic
  topic = topic + "/" + uniqueid;
# endif

  uniqueid.replace("/", "-");


  if (!isAduplicateSignal(MQTTvalue)) {
# ifdef ZmqttDiscovery
    if (SYSConfig.discovery)
      storeRTL_433Discovery(RFrtl_433_ESPdata, (char*)model.c_str(), (char*)uniqueid.c_str());
# endif


    pub(topic.c_str(), RFrtl_433_ESPdata);
    storeSignalValue(MQTTvalue);
    pubOled((char*)topic.c_str(), RFrtl_433_ESPdata);
  }
# ifdef MEMORY_DEBUG
  Log.trace(F("Post rtl_433_Callback: %d" CR), ESP.getFreeHeap());
# endif
}

void setupRTL_433() {
  rtl_433.setCallback(rtl_433_Callback, messageBuffer, JSON_MSG_BUFFER);
# ifdef ZmqttDiscovery
  semaphorecreateOrUpdateDeviceRTL_433 = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphorecreateOrUpdateDeviceRTL_433);
# endif
  Log.trace(F("ZgatewayRTL_433 command topic: %s%s%s" CR), mqtt_topic, gateway_name, subjectMQTTtoRFset);
  Log.notice(F("ZgatewayRTL_433 setup done " CR));
}

void RTL_433Loop() {
  rtl_433.loop();
}

extern void enableRTLreceive() {
  Log.notice(F("Enable RTL_433 Receiver: %FMhz" CR), RFConfig.frequency);
  rtl_433.initReceiver(RF_MODULE_RECEIVER_GPIO, RFConfig.frequency);
  rtl_433.enableReceiver();
}

extern void disableRTLreceive() {
  Log.trace(F("disableRTLreceive" CR));
  rtl_433.disableReceiver();
}

extern int getRTLrssiThreshold() {
  return rtl_433.rssiThreshold;
}

extern int getRTLAverageRSSI() {
  return rtl_433.averageRssi;
}

extern int getRTLCurrentRSSI() {
  return rtl_433.currentRssi;
}

extern int getRTLMessageCount() {
  return rtl_433.messageCount;
}

# if defined(RF_SX1276) || defined(RF_SX1278)
extern int getOOKThresh() {
  return rtl_433.OokFixedThreshold;
}
# endif

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewaySRFB.ino"
# 31 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewaySRFB.ino"
#include "User_config.h"

#ifdef ZgatewaySRFB

unsigned char _uartbuf[RF_MESSAGE_SIZE + 3] = {0};
unsigned char _uartpos = 0;

void setupSRFB() {
  Log.trace(F("ZgatewaySRFB setup done " CR));
  Log.trace(F("Serial Baud: %l" CR), SERIAL_BAUD);
}

void _rfbSend(byte* message) {
  Serial.println();
  Serial.write(RF_CODE_START);
  Serial.write(RF_CODE_RFOUT);
  for (unsigned char j = 0; j < RF_MESSAGE_SIZE; j++) {
    Serial.write(message[j]);
  }
  Serial.write(RF_CODE_STOP);
  Serial.flush();
  Serial.println();
}

void _rfbSend(byte* message, int times) {
  char buffer[RF_MESSAGE_SIZE];
  _rawToHex(message, buffer, RF_MESSAGE_SIZE);
  Log.notice(F("[RFBRIDGE] Sending MESSAGE" CR));

  for (int i = 0; i < times; i++) {
    if (i > 0) {
      unsigned long start = millis();
      while (millis() - start < RF_SEND_DELAY)
        delay(1);
    }
    _rfbSend(message);
  }
}

bool SRFBtoMQTT() {
  static bool receiving = false;

  while (Serial.available()) {
    yield();
    byte c = Serial.read();

    if (receiving) {
      if (c == RF_CODE_STOP) {
        _rfbDecode();
        receiving = false;
      } else {
        _uartbuf[_uartpos++] = c;
      }
    } else if (c == RF_CODE_START) {
      _uartpos = 0;
      receiving = true;
    }
  }
  return receiving;
}

void _rfbDecode() {
  static unsigned long last = 0;
  if (millis() - last < RF_RECEIVE_DELAY)
    return;
  last = millis();

  byte action = _uartbuf[0];
  char buffer[RF_MESSAGE_SIZE * 2 + 1] = {0};

  if (action == RF_CODE_RFIN) {
    _rawToHex(&_uartbuf[1], buffer, RF_MESSAGE_SIZE);

    Log.trace(F("Creating SRFB buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> SRFBdataBuffer;
    JsonObject SRFBdata = SRFBdataBuffer.to<JsonObject>();
    SRFBdata["raw"] = String(buffer).substring(0, 18);

    int val_Tsyn = (int)(int)value_from_hex_data(buffer, 0, 4, false, false);
    SRFBdata["delay"] = (int)val_Tsyn;

    int val_Tlow = (int)value_from_hex_data(buffer, 4, 4, false, false);
    SRFBdata["val_Tlow"] = (int)val_Tlow;

    int val_Thigh = (int)value_from_hex_data(buffer, 8, 4, false, false);
    SRFBdata["val_Thigh"] = (int)val_Thigh;

    unsigned long MQTTvalue = (unsigned long)value_from_hex_data(buffer, 12, 8, false, false);
    SRFBdata["value"] = (unsigned long)MQTTvalue;

    if (!isAduplicateSignal(MQTTvalue) && MQTTvalue != 0) {
      Log.trace(F("Adv data SRFBtoMQTT" CR));
      SRFBdata["origin"] = subjectSRFBtoMQTT;
      handleJsonEnqueue(SRFBdata);
      Log.trace(F("Store val: %lu" CR), MQTTvalue);
      storeSignalValue(MQTTvalue);
      if (repeatSRFBwMQTT) {
        Log.trace(F("Publish SRFB for rpt" CR));
        SRFBdata["origin"] = subjectMQTTtoSRFB;
        handleJsonEnqueue(SRFBdata);
      }
    }
    _rfbAck();
  }
}

void _rfbAck() {
  Log.trace(F("[RFBRIDGE] Sending ACK\n" CR));
  Serial.println();
  Serial.write(RF_CODE_START);
  Serial.write(RF_CODE_ACK);
  Serial.write(RF_CODE_STOP);
  Serial.flush();
  Serial.println();
}

# if simpleReceiving
void MQTTtoSRFB(char* topicOri, char* datacallback) {

  String topic = topicOri;
  int valueRPT = 0;

  if (topic == subjectMQTTtoSRFB) {
    int valueMiniPLSL = 0;
    int valueMaxiPLSL = 0;
    int valueSYNC = 0;

    int pos = topic.lastIndexOf(SRFBRptKey);
    if (pos != -1) {
      pos = pos + +strlen(SRFBRptKey);
      valueRPT = (topic.substring(pos, pos + 1)).toInt();
      Log.notice(F("SRFB Repeat: %d" CR), valueRPT);
    }

    int pos2 = topic.lastIndexOf(SRFBminipulselengthKey);
    if (pos2 != -1) {
      pos2 = pos2 + strlen(SRFBminipulselengthKey);
      valueMiniPLSL = (topic.substring(pos2, pos2 + 3)).toInt();
      Log.notice(F("RF Mini Pulse Lgth: %d" CR), valueMiniPLSL);
    }

    int pos3 = topic.lastIndexOf(SRFBmaxipulselengthKey);
    if (pos3 != -1) {
      pos3 = pos3 + strlen(SRFBmaxipulselengthKey);
      valueMaxiPLSL = (topic.substring(pos3, pos3 + 2)).toInt();
      Log.notice(F("RF Maxi Pulse Lgth: %d" CR), valueMaxiPLSL);
    }

    int pos4 = topic.lastIndexOf(SRFBsyncKey);
    if (pos4 != -1) {
      pos4 = pos4 + strlen(SRFBsyncKey);
      valueSYNC = (topic.substring(pos4, pos4 + 2)).toInt();
      Log.notice(F("RF sync: %d" CR), valueSYNC);
    }

    Log.trace(F("MQTTtoSRFB prts" CR));
    if (valueRPT == 0)
      valueRPT = 1;
    if (valueMiniPLSL == 0)
      valueMiniPLSL = 320;
    if (valueMaxiPLSL == 0)
      valueMaxiPLSL = 900;
    if (valueSYNC == 0)
      valueSYNC = 9500;

    byte hex_valueMiniPLSL[2];
    hex_valueMiniPLSL[0] = (int)((valueMiniPLSL >> 8) & 0xFF);
    hex_valueMiniPLSL[1] = (int)(valueMiniPLSL & 0xFF);

    byte hex_valueMaxiPLSL[2];
    hex_valueMaxiPLSL[0] = (int)((valueMaxiPLSL >> 8) & 0xFF);
    hex_valueMaxiPLSL[1] = (int)(valueMaxiPLSL & 0xFF);

    byte hex_valueSYNC[2];
    hex_valueSYNC[0] = (int)((valueSYNC >> 8) & 0xFF);
    hex_valueSYNC[1] = (int)(valueSYNC & 0xFF);

    unsigned long data = strtoul(datacallback, NULL, 10);
    byte hex_data[3];
    hex_data[0] = (unsigned long)((data >> 16) & 0xFF);
    hex_data[1] = (unsigned long)((data >> 8) & 0xFF);
    hex_data[2] = (unsigned long)(data & 0xFF);

    byte message_b[RF_MESSAGE_SIZE];

    memcpy(message_b, hex_valueSYNC, 2);
    memcpy(message_b + 2, hex_valueMiniPLSL, 2);
    memcpy(message_b + 4, hex_valueMaxiPLSL, 2);
    memcpy(message_b + 6, hex_data, 3);

    _rfbSend(message_b, valueRPT);

    pub(subjectGTWSRFBtoMQTT, datacallback);
  }
  if (topic == subjectMQTTtoSRFBRaw) {
    int pos = topic.lastIndexOf(SRFBRptKey);
    if (pos != -1) {
      pos = pos + +strlen(SRFBRptKey);
      valueRPT = (topic.substring(pos, pos + 1)).toInt();
      Log.notice(F("SRFB Repeat: %d" CR), valueRPT);
    }
    if (valueRPT == 0)
      valueRPT = 1;

    byte message_b[RF_MESSAGE_SIZE];
    _hexToRaw(datacallback, message_b, RF_MESSAGE_SIZE);
    _rfbSend(message_b, valueRPT);

    pub(subjectGTWSRFBtoMQTT, datacallback);
  }
}
# endif
# if jsonReceiving
void MQTTtoSRFB(char* topicOri, JsonObject& SRFBdata) {

  const char* raw = SRFBdata["raw"];
  int valueRPT = SRFBdata["repeat"] | 1;
  if (cmpToMainTopic(topicOri, subjectMQTTtoSRFB)) {
    Log.trace(F("MQTTtoSRFB json" CR));
    if (raw) {
      Log.trace(F("MQTTtoSRFB raw ok" CR));
      byte message_b[RF_MESSAGE_SIZE];
      _hexToRaw(raw, message_b, RF_MESSAGE_SIZE);
      _rfbSend(message_b, valueRPT);
    } else {
      unsigned long data = SRFBdata["value"];
      if (data != 0) {
        Log.notice(F("MQTTtoSRFB data ok" CR));
        int valueMiniPLSL = SRFBdata["val_Tlow"];
        int valueMaxiPLSL = SRFBdata["val_Thigh"];
        int valueSYNC = SRFBdata["delay"];

        if (valueRPT == 0)
          valueRPT = 1;
        if (valueMiniPLSL == 0)
          valueMiniPLSL = 320;
        if (valueMaxiPLSL == 0)
          valueMaxiPLSL = 900;
        if (valueSYNC == 0)
          valueSYNC = 9500;

        Log.notice(F("SRFB Repeat: %d" CR), valueRPT);
        Log.notice(F("RF Mini Pulse Lgth: %d" CR), valueMiniPLSL);
        Log.notice(F("RF Maxi Pulse Lgth: %d" CR), valueMaxiPLSL);
        Log.notice(F("RF sync: %d" CR), valueSYNC);

        byte hex_valueMiniPLSL[2];
        hex_valueMiniPLSL[0] = (int)((valueMiniPLSL >> 8) & 0xFF);
        hex_valueMiniPLSL[1] = (int)(valueMiniPLSL & 0xFF);

        byte hex_valueMaxiPLSL[2];
        hex_valueMaxiPLSL[0] = (int)((valueMaxiPLSL >> 8) & 0xFF);
        hex_valueMaxiPLSL[1] = (int)(valueMaxiPLSL & 0xFF);

        byte hex_valueSYNC[2];
        hex_valueSYNC[0] = (int)((valueSYNC >> 8) & 0xFF);
        hex_valueSYNC[1] = (int)(valueSYNC & 0xFF);

        byte hex_data[3];
        hex_data[0] = (unsigned long)((data >> 16) & 0xFF);
        hex_data[1] = (unsigned long)((data >> 8) & 0xFF);
        hex_data[2] = (unsigned long)(data & 0xFF);

        byte message_b[RF_MESSAGE_SIZE];

        memcpy(message_b, hex_valueSYNC, 2);
        memcpy(message_b + 2, hex_valueMiniPLSL, 2);
        memcpy(message_b + 4, hex_valueMaxiPLSL, 2);
        memcpy(message_b + 6, hex_data, 3);

        Log.notice(F("MQTTtoSRFB OK" CR));
        _rfbSend(message_b, valueRPT);
        pub(subjectGTWSRFBtoMQTT, SRFBdata);
      } else {
        Log.error(F("MQTTtoSRFB error decoding value" CR));
      }
    }
  }
}
# endif
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayWeatherStation.ino"
# 26 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZgatewayWeatherStation.ino"
#include "User_config.h"

#ifdef ZgatewayWeatherStation
# include <WeatherStationDataRx.h>
WeatherStationDataRx wsdr(RF_WS_RECEIVER_GPIO, true);

void PairedDeviceAdded(byte newID) {
# if defined(ESP8266) || defined(ESP32)
  Serial.printf("ZgatewayWeatherStation: New device paired %d\r\n", newID);
# else
  Serial.print("ZgatewayWeatherStation: New device paired ");
  Serial.println(newID, DEC);
# endif
  StaticJsonDocument<JSON_MSG_BUFFER> RFdataBuffer;
  JsonObject RFdata = RFdataBuffer.to<JsonObject>();
  RFdata["sensor"] = newID;
  RFdata["action"] = "paired";
  RFdata["origin"] = subjectRFtoMQTT;
  handleJsonEnqueue(RFdata);
  wsdr.pair(NULL, PairedDeviceAdded);
}

void setupWeatherStation() {
  Log.notice(F("RF_WS_RECEIVER_GPIO %d" CR), RF_WS_RECEIVER_GPIO);
  wsdr.begin();
  wsdr.pair(NULL, PairedDeviceAdded);
  Log.trace(F("ZgatewayWeatherStation setup done " CR));
}

void sendWindSpeedData(byte id, float wind_speed, byte battery_status) {
  unsigned long MQTTvalue = 10000 + round(wind_speed);
  if (!isAduplicateSignal(MQTTvalue)) {
    StaticJsonDocument<JSON_MSG_BUFFER> RFdataBuffer;
    JsonObject RFdata = RFdataBuffer.to<JsonObject>();
    RFdata["sensor"] = id;
    RFdata["wind_speed"] = wind_speed;
    RFdata["battery"] = bitRead(battery_status, 0) == 0 ? "OK" : "Low";
    RFdata["origin"] = subjectRFtoMQTT;
    handleJsonEnqueue(RFdata);
    Log.trace(F("Store wind speed val: %lu" CR), MQTTvalue);
    storeSignalValue(MQTTvalue);
  }
}

void sendRainData(byte id, float rain_volume, byte battery_status) {
  unsigned long MQTTvalue = 11000 + round(rain_volume * 10.0);
  if (!isAduplicateSignal(MQTTvalue)) {
    StaticJsonDocument<JSON_MSG_BUFFER> RFdataBuffer;
    JsonObject RFdata = RFdataBuffer.to<JsonObject>();
    RFdata["sensor"] = id;
    RFdata["rain_volume"] = rain_volume;
    RFdata["battery"] = bitRead(battery_status, 1) == 0 ? "OK" : "Low";
    RFdata["origin"] = subjectRFtoMQTT;
    handleJsonEnqueue(RFdata);
    Log.trace(F("Store rain_volume: %lu" CR), MQTTvalue);
    storeSignalValue(MQTTvalue);
  }
}

void sendWindData(byte id, int wind_direction, float wind_gust, byte battery_status) {
  unsigned long MQTTvalue = 20000 + round(wind_gust * 10.0) + wind_direction;
  if (!isAduplicateSignal(MQTTvalue)) {
    StaticJsonDocument<JSON_MSG_BUFFER> RFdataBuffer;
    JsonObject RFdata = RFdataBuffer.to<JsonObject>();
    RFdata["sensor"] = id;
    RFdata["wind_direction"] = wind_direction;
    RFdata["wind_gust"] = wind_gust;
    RFdata["battery"] = bitRead(battery_status, 0) == 0 ? "OK" : "Low";
    RFdata["origin"] = subjectRFtoMQTT;
    handleJsonEnqueue(RFdata);
    Log.trace(F("Store wind data val: %lu" CR), MQTTvalue);
    storeSignalValue(MQTTvalue);
  }
}

void sendTemperatureData(byte id, float temperature, int humidity, byte battery_status) {
  unsigned long MQTTvalue = 40000 + abs(round(temperature * 100.0)) + humidity;
  if (!isAduplicateSignal(MQTTvalue)) {
    StaticJsonDocument<JSON_MSG_BUFFER> RFdataBuffer;
    JsonObject RFdata = RFdataBuffer.to<JsonObject>();
    RFdata["sensor"] = id;
    RFdata["tempc"] = temperature;
    RFdata["tempf"] = wsdr.convertCtoF(temperature);
    RFdata["humidity"] = humidity;
    RFdata["battery"] = bitRead(battery_status, 0) == 0 ? "OK" : "Low";
    RFdata["origin"] = subjectRFtoMQTT;
    handleJsonEnqueue(RFdata);
    Log.trace(F("Store temp val: %lu" CR), MQTTvalue);
    storeSignalValue(MQTTvalue);
  }
}

void ZgatewayWeatherStationtoMQTT() {
  char newData = wsdr.readData();
  switch (newData) {
    case 'T':
      Log.trace(F("Temperature" CR));
      sendTemperatureData(wsdr.sensorID(), wsdr.readTemperature(), wsdr.readHumidity(), wsdr.batteryStatus());
      break;

    case 'S':
      Log.trace(F("Wind speed" CR));
      sendWindSpeedData(wsdr.sensorID(), wsdr.readWindSpeed(), wsdr.batteryStatus());
      break;

    case 'G':
      Log.trace(F("Wind direction" CR));
      sendWindData(wsdr.sensorID(), wsdr.readWindDirection(), wsdr.readWindGust(), wsdr.batteryStatus());
      break;

    case 'R':
      Log.trace(F("Rain volume" CR));
      sendRainData(wsdr.sensorID(), wsdr.readRainVolume(), wsdr.batteryStatus());
      break;

    default:
      break;
  }
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZmqttDiscovery.ino"
# 26 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZmqttDiscovery.ino"
#include "User_config.h"

#ifdef ZmqttDiscovery

String getMacAddress() {
  uint8_t baseMac[6];
  char baseMacChr[13] = {0};
# if defined(ESP8266)
  WiFi.macAddress(baseMac);
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
# elif defined(ESP32)
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
# else
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
# endif
  return String(baseMacChr);
}

String getUniqueId(String name, String sufix) {
  String uniqueId = (String)getMacAddress() + "-" + name + sufix;
  return String(uniqueId);
}

# ifdef ZgatewayBT
# 69 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZmqttDiscovery.ino"
void createDiscoveryFromList(const char* mac,
                             const char* sensorList[][9],
                             int sensorCount,
                             const char* device_name,
                             const char* device_manufacturer,
                             const char* device_model) {
  for (int i = 0; i < sensorCount; i++) {
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + sensorList[i][1];

    createDiscovery(sensorList[i][0],
                    discovery_topic.c_str(), sensorList[i][1], unique_id.c_str(),
                    will_Topic, sensorList[i][3], sensorList[i][4],
                    sensorList[i][5], sensorList[i][6], sensorList[i][7],
                    0, "", "", false, "",
                    device_name, device_manufacturer, device_model, mac, false,
                    sensorList[i][8]
    );
  }
}
# endif
# 103 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZmqttDiscovery.ino"
void announceDeviceTrigger(bool use_gateway_info, char* topic, char* type, char* subtype, char* unique_id, char* device_name, char* device_manufacturer, char* device_model, char* device_id) {

  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject sensor = jsonBuffer.to<JsonObject>();


  sensor["automation_type"] = "trigger";


  if (type && type[0] != 0) {
    sensor["type"] = type;
  } else {
    sensor["type"] = "button_short_press";
  }


  if (subtype && subtype[0] != 0) {
    sensor["subtype"] = subtype;
  } else {
    sensor["subtype"] = "turn_on";
  }


  if (topic && topic[0]) {
    char state_topic[mqtt_topic_max_size];

    strcpy(state_topic, mqtt_topic);
    strcat(state_topic, gateway_name);

    strcat(state_topic, topic);
    sensor["topic"] = state_topic;
  }


  StaticJsonDocument<JSON_MSG_BUFFER> jsonDeviceBuffer;
  JsonObject device = jsonDeviceBuffer.to<JsonObject>();
  JsonArray identifiers = device.createNestedArray("identifiers");

  if (use_gateway_info) {
    device["name"] = gateway_name;
# ifndef GATEWAY_MODEL
    String model = "";
    serializeJson(modules, model);
    device["model"] = model;
# else
    device["model"] = GATEWAY_MODEL;
# endif

    device["manufacturer"] = GATEWAY_MANUFACTURER;
    device["sw_version"] = OMG_VERSION;
    identifiers.add(getMacAddress());

  } else {
    char deviceid[13];
    memcpy(deviceid, &unique_id[0], 12);
    deviceid[12] = '\0';

    identifiers.add(deviceid);


    if (device_id && device_id[0] != 0) {
      JsonArray connections = device.createNestedArray("connections");
      JsonArray connection_mac = connections.createNestedArray();
      connection_mac.add("mac");
      connection_mac.add(device_id);
    }


    if (device_manufacturer && device_manufacturer[0]) {
      device["manufacturer"] = device_manufacturer;
    }


    if (device_name && device_name[0]) {
      device["name"] = device_name;
    }


    if (device_model && device_model[0]) {
      device["model"] = device_model;
    }

    device["via_device"] = gateway_name;
  }
  sensor["device"] = device;


  String topic_to_publish = String(discovery_Topic) + "/device_automation/" + String(unique_id) + "/config";
  Log.trace(F("Announce Device Trigger  %s" CR), topic_to_publish.c_str());
  pub_custom_topic((char*)topic_to_publish.c_str(), sensor, true);
}




std::string remove_substring(std::string s, const std::string& p) {
  std::string::size_type n = p.length();

  for (std::string::size_type i = s.find(p);
       i != std::string::npos;
       i = s.find(p))
    s.erase(i, n);

  return s;
}
# 235 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZmqttDiscovery.ino"
void createDiscovery(const char* sensor_type,
                     const char* st_topic, const char* s_name, const char* unique_id,
                     const char* availability_topic, const char* device_class, const char* value_template,
                     const char* payload_on, const char* payload_off, const char* unit_of_meas,
                     int off_delay,
                     const char* payload_available, const char* payload_not_available, bool gateway_entity, const char* cmd_topic,
                     const char* device_name, const char* device_manufacturer, const char* device_model, const char* device_id, bool retainCmd,
                     const char* state_class, const char* state_off, const char* state_on, const char* enum_options) {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject sensor = jsonBuffer.to<JsonObject>();







  if (st_topic[0]) {
    char state_topic[mqtt_topic_max_size];


    if (gateway_entity) {
      strcpy(state_topic, mqtt_topic);
      strcat(state_topic, gateway_name);
    } else {
      strcpy(state_topic, "+/+");
    }
    strcat(state_topic, st_topic);
    sensor["stat_t"] = state_topic;
  }

  if (availability_topic[0] && gateway_entity) {
    char avty_topic[mqtt_topic_max_size];
    strcpy(avty_topic, mqtt_topic);
    strcat(avty_topic, gateway_name);
    strcat(avty_topic, availability_topic);
    sensor["avty_t"] = avty_topic;
  }

  if (device_class[0]) {

    int num_classes = sizeof(availableHASSClasses) / sizeof(availableHASSClasses[0]);
    for (int i = 0; i < num_classes; i++) {
      if (strcmp(availableHASSClasses[i], device_class) == 0) {
        sensor["dev_cla"] = device_class;
      }
    }
  }

  if (unit_of_meas[0]) {

    int num_units = sizeof(availableHASSUnits) / sizeof(availableHASSUnits[0]);
    for (int i = 0; i < num_units; i++) {
      if (strcmp(availableHASSUnits[i], unit_of_meas) == 0) {
        sensor["unit_of_meas"] = unit_of_meas;
      }
    }
  }
  sensor["name"] = s_name;
  sensor["uniq_id"] = unique_id;
  if (retainCmd)
    sensor["retain"] = retainCmd;
  if (value_template[0]) {
    if (strstr(value_template, " | is_defined") != NULL && SYSConfig.ohdiscovery) {
      sensor["val_tpl"] = remove_substring(value_template, " | is_defined");
    } else {
      sensor["val_tpl"] = value_template;
    }
  }
  if (payload_on[0]) {
    if (strcmp(sensor_type, "button") == 0) {
      sensor["pl_prs"] = payload_on;
    } else if (strcmp(sensor_type, "number") == 0) {
      sensor["cmd_tpl"] = payload_on;
    } else if (strcmp(sensor_type, "update") == 0) {
      sensor["payload_install"] = payload_on;
    } else {
      sensor["pl_on"] = payload_on;
    }
  }
  if (payload_off[0])
    sensor["pl_off"] = payload_off;
  if (strcmp(sensor_type, "device_tracker") == 0)
    sensor["source_type"] = "bluetooth_le";
  if (off_delay != 0)
    sensor["off_delay"] = off_delay;
  if (payload_available[0])
    sensor["pl_avail"] = payload_available;
  if (payload_not_available[0])
    sensor["pl_not_avail"] = payload_not_available;
  if (state_class && state_class[0])
    sensor["state_class"] = state_class;
  if (state_on != nullptr)
    if (strcmp(state_on, "true") == 0) {
      sensor["stat_on"] = true;
    } else {
      sensor["stat_on"] = state_on;
    }
  if (state_off != nullptr)
    if (strcmp(state_off, "false") == 0) {
      sensor["stat_off"] = false;
    } else {
      sensor["stat_off"] = state_off;
    }
  if (cmd_topic[0]) {
    char command_topic[mqtt_topic_max_size];
    strcpy(command_topic, mqtt_topic);
    strcat(command_topic, gateway_name);
    strcat(command_topic, cmd_topic);
    sensor["cmd_t"] = command_topic;
  }

  if (enum_options != nullptr) {
    sensor["options"] = enum_options;
  }

  StaticJsonDocument<JSON_MSG_BUFFER> jsonDeviceBuffer;
  JsonObject device = jsonDeviceBuffer.to<JsonObject>();
  JsonArray identifiers = device.createNestedArray("ids");

  if (gateway_entity) {

    device["name"] = String(gateway_name);
# ifndef GATEWAY_MODEL
    String model = "";
    serializeJson(modules, model);
    device["mdl"] = model;
# else
    device["mdl"] = GATEWAY_MODEL;
# endif
    device["mf"] = GATEWAY_MANUFACTURER;
    if (ethConnected) {
# ifdef ESP32_ETHERNET
      device["cu"] = String("http://") + String(ETH.localIP().toString()) + String("/");
# endif
    } else {
      device["cu"] = String("http://") + String(WiFi.localIP().toString()) + String("/");
    }

    device["sw"] = OMG_VERSION;
    identifiers.add(String(getMacAddress()));
  } else {

    if (device_id[0]) {
      JsonArray connections = device.createNestedArray("cns");
      JsonArray connection_mac = connections.createNestedArray();
      connection_mac.add("mac");
      connection_mac.add(device_id);


      identifiers.add(device_id);
    }

    if (device_manufacturer[0]) {
      device["mf"] = device_manufacturer;
    }

    if (device_model[0]) {
      device["mdl"] = device_model;
    }


    if (device_name[0]) {
      if (strcmp(device_id, device_name) != 0 && device_id[0]) {
        device["name"] = device_name + String("-") + String(device_id + 6);
      } else {
        device["name"] = device_name;
      }
    }

    device["via_device"] = String(gateway_name);
  }

  sensor["device"] = device;

  String topic = String(discovery_Topic) + "/" + String(sensor_type) + "/" + String(unique_id) + "/config";
  Log.trace(F("Announce Device %s on  %s" CR), String(sensor_type).c_str(), topic.c_str());
  pub_custom_topic((char*)topic.c_str(), sensor, true);
}

void eraseTopic(const char* sensor_type, const char* unique_id) {
  if (sensor_type == NULL || unique_id == NULL) {
    return;
  }
  String topic = String(discovery_Topic) + "/" + String(sensor_type) + "/" + String(unique_id) + "/config";
  Log.trace(F("Erase entity discovery %s on  %s" CR), String(sensor_type).c_str(), topic.c_str());
  pubMQTT((char*)topic.c_str(), "", true);
}

# ifdef ZgatewayBT
void btPresenceParametersDiscovery() {
  if (BTConfig.presenceEnable) {
    createDiscovery("number",
                    subjectBTtoMQTT, "BT: Presence detection timer", (char*)getUniqueId("presenceawaytimer", "").c_str(),
                    will_Topic, "", "{{ value_json.presenceawaytimer/60000 }}",
                    "{\"presenceawaytimer\":{{value*60000}},\"save\":true}", "", "min",
                    0,
                    Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                    "", "", "", "", false,
                    stateClassNone
    );
  }
}

void btScanParametersDiscovery() {
  if (!BTConfig.adaptiveScan) {
    createDiscovery("number",
                    subjectBTtoMQTT, "BT: Interval between scans", (char*)getUniqueId("interval", "").c_str(),
                    will_Topic, "", "{{ value_json.interval/1000 }}",
                    "{\"interval\":{{value*1000}},\"save\":true}", "", "s",
                    0,
                    Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                    "", "", "", "", false,
                    stateClassNone
    );
    createDiscovery("number",
                    subjectBTtoMQTT, "BT: Interval between active scans", (char*)getUniqueId("intervalacts", "").c_str(),
                    will_Topic, "", "{{ value_json.intervalacts/1000 }}",
                    "{\"intervalacts\":{{value*1000}},\"save\":true}", "", "s",
                    0,
                    Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                    "", "", "", "", false,
                    stateClassNone
    );
  }
}
# endif

void pubMqttDiscovery() {
  Log.trace(F("omgStatusDiscovery" CR));
  createDiscovery("binary_sensor",
                  will_Topic, "SYS: Connectivity", (char*)getUniqueId("connectivity", "").c_str(),
                  will_Topic, "connectivity", "",
                  Gateway_AnnouncementMsg, will_Message, "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Uptime", (char*)getUniqueId("uptime", "").c_str(),
                  will_Topic, "duration", "{{ value_json.uptime }}",
                  "", "", "s",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassMeasurement
  );
# if defined(ESP8266) || defined(ESP32)
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Free memory", (char*)getUniqueId("freemem", "").c_str(),
                  will_Topic, "data_size", "{{ value_json.freemem }}",
                  "", "", "B",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassMeasurement
  );
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: IP", (char*)getUniqueId("ip", "").c_str(),
                  will_Topic, "", "{{ value_json.ip }}",
                  "", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("switch",
                  subjectSYStoMQTT, "SYS: Auto discovery", (char*)getUniqueId("disc", "").c_str(),
                  will_Topic, "", "{{ value_json.disc }}",
                  "{\"disc\":true,\"save\":true}", "{\"disc\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
  createDiscovery("switch",
                  subjectSYStoMQTT, "SYS: OpenHAB discovery", (char*)getUniqueId("ohdisc", "").c_str(),
                  will_Topic, "", "{{ value_json.ohdisc }}",
                  "{\"ohdisc\":true,\"save\":true}", "{\"ohdisc\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
# ifdef RGB_INDICATORS
  createDiscovery("number",
                  subjectSYStoMQTT, "SYS: LED Brightness", (char*)getUniqueId("rgbb", "").c_str(),
                  will_Topic, "", "{{ (value_json.rgbb/2.55) | round(0) }}",
                  "{\"rgbb\":{{ (value*2.55) | round(0) }},\"save\":true}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSset,
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZdisplaySSD1306
  createDiscovery("switch",
                  subjectSSD1306toMQTT, "SSD1306: Control", (char*)getUniqueId("onstate", "").c_str(),
                  will_Topic, "", "{{ value_json.onstate }}",
                  "{\"onstate\":true,\"save\":true}", "{\"onstate\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSSD1306set,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
  createDiscovery("switch",
                  subjectWebUItoMQTT, "SSD1306: Display metric", (char*)getUniqueId("displayMetric", "").c_str(),
                  will_Topic, "", "{{ value_json.displayMetric }}",
                  "{\"displayMetric\":true,\"save\":true}", "{\"displayMetric\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoWebUIset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
  createDiscovery("number",
                  subjectSSD1306toMQTT, "SSD1306: Brightness", (char*)getUniqueId("brightness", "").c_str(),
                  will_Topic, "", "{{ value_json.brightness }}",
                  "{\"brightness\":{{value}},\"save\":true}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSSD1306set,
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifndef ESP32_ETHERNET
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: RSSI", (char*)getUniqueId("rssi", "").c_str(),
                  will_Topic, "signal_strength", "{{ value_json.rssi }}",
                  "", "", "dB",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif
# endif
# if defined(ESP32) && !defined(NO_INT_TEMP_READING)
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Internal temperature", (char*)getUniqueId("tempc", "").c_str(),
                  will_Topic, "temperature", "{{ value_json.tempc  | round(1)}}",
                  "", "", "°C",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassMeasurement
  );
# if defined(ZboardM5STICKC) || defined(ZboardM5STICKCP) || defined(ZboardM5TOUGH)
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Bat voltage", (char*)getUniqueId("m5batvoltage", "").c_str(),
                  will_Topic, "voltage", "{{ value_json.m5batvoltage }}",
                  "", "", "V",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Bat current", (char*)getUniqueId("m5batcurrent", "").c_str(),
                  will_Topic, "current", "{{ value_json.m5batcurrent }}",
                  "", "", "A",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Vin voltage", (char*)getUniqueId("m5vinvoltage", "").c_str(),
                  will_Topic, "voltage", "{{ value_json.m5vinvoltage }}",
                  "", "", "V",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Vin current", (char*)getUniqueId("m5vincurrent", "").c_str(),
                  will_Topic, "current", "{{ value_json.m5vincurrent }}",
                  "", "", "A",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif
# ifdef ZboardM5STACK
  createDiscovery("sensor",
                  subjectSYStoMQTT, "SYS: Batt level", (char*)getUniqueId("m5battlevel", "").c_str(),
                  will_Topic, "battery", "{{ value_json.m5battlevel }}",
                  "", "", "%",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("binary_sensor",
                  subjectSYStoMQTT, "SYS: Is Charging", (char*)getUniqueId("m5ischarging", "").c_str(),
                  will_Topic, "{{ value_json.m5ischarging }}", "",
                  "", "", "%",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("binary_sensor",
                  subjectSYStoMQTT, "SYS: Is Charge Full", (char*)getUniqueId("m5ischargefull", "").c_str(),
                  will_Topic, "{{ value_json.m5ischargefull }}", "",
                  "", "", "%",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif
# endif
  createDiscovery("button",
                  will_Topic, "SYS: Restart gateway", (char*)getUniqueId("restart", "").c_str(),
                  will_Topic, "restart", "",
                  "{\"cmd\":\"restart\"}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSset,
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("button",
                  will_Topic, "SYS: Erase credentials", (char*)getUniqueId("erase", "").c_str(),
                  will_Topic, "", "",
                  "{\"cmd\":\"erase\"}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSset,
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("update",
                  subjectRLStoMQTT, "SYS: Firmware Update", (char*)getUniqueId("update", "").c_str(),
                  will_Topic, "firmware", "",
                  LATEST_OR_DEV, "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSupdate,
                  "", "", "", "", false,
                  stateClassNone
  );
# ifdef ZsensorBME280
#define BMEparametersCount 5
  Log.trace(F("bme280Discovery" CR));
  char* BMEsensor[BMEparametersCount][8] = {
      {"sensor", "temp", "bme", "temperature", jsonTempc, "", "", "°C"},
      {"sensor", "pa", "bme", "pressure", jsonPa, "", "", "hPa"},
      {"sensor", "hum", "bme", "humidity", jsonHum, "", "", "%"},
      {"sensor", "altim", "bme", "", jsonAltim, "", "", "m"},
      {"sensor", "altift", "bme", "", jsonAltif, "", "", "ft"}

  };

  for (int i = 0; i < BMEparametersCount; i++) {
    createDiscovery(BMEsensor[i][0],
                    BMETOPIC, BMEsensor[i][1], (char*)getUniqueId(BMEsensor[i][1], BMEsensor[i][2]).c_str(),
                    will_Topic, BMEsensor[i][3], BMEsensor[i][4],
                    BMEsensor[i][5], BMEsensor[i][6], BMEsensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassNone
    );
  }
# endif

# ifdef ZsensorHTU21
#define HTUparametersCount 2
  Log.trace(F("htu21Discovery" CR));
  char* HTUsensor[HTUparametersCount][8] = {
      {"sensor", "temp", "htu", "temperature", jsonTempc, "", "", "°C"},
      {"sensor", "hum", "htu", "humidity", jsonHum, "", "", "%"}

  };

  for (int i = 0; i < HTUparametersCount; i++) {

    createDiscovery(HTUsensor[i][0],
                    HTUTOPIC, HTUsensor[i][1], (char*)getUniqueId(HTUsensor[i][1], HTUsensor[i][2]).c_str(),
                    will_Topic, HTUsensor[i][3], HTUsensor[i][4],
                    HTUsensor[i][5], HTUsensor[i][6], HTUsensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZsensorLM75
  Log.trace(F("LM75Discovery" CR));
  char* LM75sensor[8] = {"sensor", "temp", "htu", "temperature", jsonTempc, "", "", "°C"};


  createDiscovery(LM75sensor[0],
                  LM75TOPIC, LM75sensor[1], (char*)getUniqueId(LM75sensor[1], LM75sensor[2]).c_str(),
                  will_Topic, LM75sensor[3], LM75sensor[4],
                  LM75sensor[5], LM75sensor[6], LM75sensor[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassMeasurement
  );
# endif

# ifdef ZsensorAHTx0
#define AHTparametersCount 2
  Log.trace(F("AHTx0Discovery" CR));
  char* AHTsensor[AHTparametersCount][8] = {
      {"sensor", "temp", "aht", "temperature", jsonTempc, "", "", "°C"},
      {"sensor", "hum", "aht", "humidity", jsonHum, "", "", "%"}

  };

  for (int i = 0; i < AHTparametersCount; i++) {
    createDiscovery(AHTsensor[i][0],
                    AHTTOPIC, AHTsensor[i][1], (char*)getUniqueId(AHTsensor[i][1], AHTsensor[i][2]).c_str(),
                    will_Topic, AHTsensor[i][3], AHTsensor[i][4],
                    AHTsensor[i][5], AHTsensor[i][6], AHTsensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZsensorDHT
#define DHTparametersCount 2
  Log.trace(F("DHTDiscovery" CR));
  char* DHTsensor[DHTparametersCount][8] = {
      {"sensor", "temp", "dht", "temperature", jsonTempc, "", "", "°C"},
      {"sensor", "hum", "dht", "humidity", jsonHum, "", "", "%"}

  };

  for (int i = 0; i < DHTparametersCount; i++) {

    createDiscovery(DHTsensor[i][0],
                    DHTTOPIC, DHTsensor[i][1], (char*)getUniqueId(DHTsensor[i][1], DHTsensor[i][2]).c_str(),
                    will_Topic, DHTsensor[i][3], DHTsensor[i][4],
                    DHTsensor[i][5], DHTsensor[i][6], DHTsensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZsensorADC
  Log.trace(F("ADCDiscovery" CR));
  char* ADCsensor[8] = {"sensor", "adc", "", "", jsonAdc, "", "", ""};



  createDiscovery(ADCsensor[0],
                  ADCTOPIC, ADCsensor[1], (char*)getUniqueId(ADCsensor[1], ADCsensor[2]).c_str(),
                  will_Topic, ADCsensor[3], ADCsensor[4],
                  ADCsensor[5], ADCsensor[6], ADCsensor[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZsensorBH1750
#define BH1750parametersCount 3
  Log.trace(F("BH1750Discovery" CR));
  char* BH1750sensor[BH1750parametersCount][8] = {
      {"sensor", "lux", "BH1750", "illuminance", jsonLux, "", "", "lx"},
      {"sensor", "ftCd", "BH1750", "irradiance", jsonFtcd, "", "", ""},
      {"sensor", "wattsm2", "BH1750", "irradiance", jsonWm2, "", "", "wm²"}

  };

  for (int i = 0; i < BH1750parametersCount; i++) {

    createDiscovery(BH1750sensor[i][0],
                    subjectBH1750toMQTT, BH1750sensor[i][1], (char*)getUniqueId(BH1750sensor[i][1], BH1750sensor[i][2]).c_str(),
                    will_Topic, BH1750sensor[i][3], BH1750sensor[i][4],
                    BH1750sensor[i][5], BH1750sensor[i][6], BH1750sensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZsensorMQ2
#define MQ2parametersCount 2
  Log.trace(F("MQ2Discovery" CR));
  char* MQ2sensor[MQ2parametersCount][8] = {
      {"sensor", "gas", "MQ2", "gas", jsonVal, "", "", "ppm"},
      {"binary_sensor", "MQ2", "", "gas", jsonPresence, "true", "false", ""}

  };

  for (int i = 0; i < MQ2parametersCount; i++) {
    createDiscovery(MQ2sensor[i][0],
                    subjectMQ2toMQTT, MQ2sensor[i][1], (char*)getUniqueId(MQ2sensor[i][1], MQ2sensor[i][2]).c_str(),
                    will_Topic, MQ2sensor[i][3], MQ2sensor[i][4],
                    MQ2sensor[i][5], MQ2sensor[i][6], MQ2sensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassNone
    );
  }
# endif

# ifdef ZsensorTEMT6000
#define TEMT6000parametersCount 3
  Log.trace(F("TEMT6000Discovery" CR));
  char* TEMT6000sensor[TEMT6000parametersCount][8] = {
      {"sensor", "lux", "TEMT6000", "illuminance", jsonLux, "", "", "lx"},
      {"sensor", "ftcd", "TEMT6000", "irradiance", jsonFtcd, "", "", ""},
      {"sensor", "wattsm2", "TEMT6000", "irradiance", jsonWm2, "", "", "wm²"}

  };

  for (int i = 0; i < TEMT6000parametersCount; i++) {

    createDiscovery(TEMT6000sensor[i][0],
                    subjectTEMT6000toMQTT, TEMT6000sensor[i][1], (char*)getUniqueId(TEMT6000sensor[i][1], TEMT6000sensor[i][2]).c_str(),
                    will_Topic, TEMT6000sensor[i][3], TEMT6000sensor[i][4],
                    TEMT6000sensor[i][5], TEMT6000sensor[i][6], TEMT6000sensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZsensorTSL2561
#define TSL2561parametersCount 3
  Log.trace(F("TSL2561Discovery" CR));
  char* TSL2561sensor[TSL2561parametersCount][8] = {
      {"sensor", "lux", "TSL2561", "illuminance", jsonLux, "", "", "lx"},
      {"sensor", "ftcd", "TSL2561", "irradiance", jsonFtcd, "", "", ""},
      {"sensor", "wattsm2", "TSL2561", "irradiance", jsonWm2, "", "", "wm²"}

  };

  for (int i = 0; i < TSL2561parametersCount; i++) {

    createDiscovery(TSL2561sensor[i][0],
                    subjectTSL12561toMQTT, TSL2561sensor[i][1], (char*)getUniqueId(TSL2561sensor[i][1], TSL2561sensor[i][2]).c_str(),
                    will_Topic, TSL2561sensor[i][3], TSL2561sensor[i][4],
                    TSL2561sensor[i][5], TSL2561sensor[i][6], TSL2561sensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZsensorHCSR501
  Log.trace(F("HCSR501Discovery" CR));
  char* HCSR501sensor[8] = {"binary_sensor", "hcsr501", "", "motion", jsonPresence, "true", "false", ""};



  createDiscovery(HCSR501sensor[0],
                  subjectHCSR501toMQTT, HCSR501sensor[1], (char*)getUniqueId(HCSR501sensor[1], HCSR501sensor[2]).c_str(),
                  will_Topic, HCSR501sensor[3], HCSR501sensor[4],
                  HCSR501sensor[5], HCSR501sensor[6], HCSR501sensor[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZsensorGPIOInput
  Log.trace(F("GPIOInputDiscovery" CR));
  char* GPIOInputsensor[8] = {"binary_sensor", "GPIOInput", "", "", jsonGpio, INPUT_GPIO_ON_VALUE, INPUT_GPIO_OFF_VALUE, ""};



  createDiscovery(GPIOInputsensor[0],
                  subjectGPIOInputtoMQTT, GPIOInputsensor[1], (char*)getUniqueId(GPIOInputsensor[1], GPIOInputsensor[2]).c_str(),
                  will_Topic, GPIOInputsensor[3], GPIOInputsensor[4],
                  GPIOInputsensor[5], GPIOInputsensor[6], GPIOInputsensor[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZsensorINA226
#define INA226parametersCount 3
  Log.trace(F("INA226Discovery" CR));
  char* INA226sensor[INA226parametersCount][8] = {
      {"sensor", "volt", "INA226", "voltage", jsonVolt, "", "", "V"},
      {"sensor", "current", "INA226", "current", jsonCurrent, "", "", "A"},
      {"sensor", "power", "INA226", "power", jsonPower, "", "", "W"}

  };

  for (int i = 0; i < INA226parametersCount; i++) {

    createDiscovery(INA226sensor[i][0],
                    subjectINA226toMQTT, INA226sensor[i][1], (char*)getUniqueId(INA226sensor[i][1], INA226sensor[i][2]).c_str(),
                    will_Topic, INA226sensor[i][3], INA226sensor[i][4],
                    INA226sensor[i][5], INA226sensor[i][6], INA226sensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZsensorDS1820

  pubOneWire_HADiscovery();
# endif

# ifdef ZactuatorONOFF
  Log.trace(F("actuatorONOFFDiscovery" CR));
  char* actuatorONOFF[8] = {"switch", "actuatorONOFF", "", "", "{{ value_json.cmd }}", "{\"cmd\":1}", "{\"cmd\":0}", ""};



  createDiscovery(actuatorONOFF[0],
                  subjectGTWONOFFtoMQTT, actuatorONOFF[1], (char*)getUniqueId(actuatorONOFF[1], actuatorONOFF[2]).c_str(),
                  will_Topic, actuatorONOFF[3], actuatorONOFF[4],
                  actuatorONOFF[5], actuatorONOFF[6], actuatorONOFF[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoONOFF,
                  "", "", "", "", false,
                  stateClassNone,
                  "0", "1"
  );
# endif

# ifdef ZsensorRN8209
#define RN8209parametersCount 4
  Log.trace(F("RN8209Discovery" CR));
  char* RN8209sensor[RN8209parametersCount][8] = {
      {"sensor", "volt", "RN8209", "voltage", jsonVolt, "", "", "V"},
      {"sensor", "current", "RN8209", "current", jsonCurrent, "", "", "A"},
      {"sensor", "power", "RN8209", "power", jsonPower, "", "", "W"},
      {"binary_sensor", "inUse", "RN8209", "power", jsonInuseRN8209, "on", "off", ""}

  };

  for (int i = 0; i < RN8209parametersCount; i++) {
    String name = "NRG: " + String(RN8209sensor[i][1]);
    createDiscovery(RN8209sensor[i][0],
                    subjectRN8209toMQTT, (char*)name.c_str(), (char*)getUniqueId(RN8209sensor[i][1], RN8209sensor[i][2]).c_str(),
                    will_Topic, RN8209sensor[i][3], RN8209sensor[i][4],
                    RN8209sensor[i][5], RN8209sensor[i][6], RN8209sensor[i][7],
                    0, Gateway_AnnouncementMsg, will_Message, true, "",
                    "", "", "", "", false,
                    stateClassMeasurement
    );
  }
# endif

# ifdef ZgatewayRF

  Log.trace(F("gatewayRFDiscovery" CR));
  char* gatewayRF[8] = {"sensor", "gatewayRF", "", "", jsonVal, "", "", ""};



  createDiscovery(gatewayRF[0],
                  subjectRFtoMQTT, gatewayRF[1], (char*)getUniqueId(gatewayRF[1], gatewayRF[2]).c_str(),
                  will_Topic, gatewayRF[3], gatewayRF[4],
                  gatewayRF[5], gatewayRF[6], gatewayRF[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );

# endif

# ifdef ZgatewayRF2

  Log.trace(F("gatewayRF2Discovery" CR));
  char* gatewayRF2[8] = {"sensor", "gatewayRF2", "", "", jsonAddress, "", "", ""};



  createDiscovery(gatewayRF2[0],
                  subjectRF2toMQTT, gatewayRF2[1], (char*)getUniqueId(gatewayRF2[1], gatewayRF2[2]).c_str(),
                  will_Topic, gatewayRF2[3], gatewayRF2[4],
                  gatewayRF2[5], gatewayRF2[6], gatewayRF2[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZgatewayRFM69

  Log.trace(F("gatewayRFM69Discovery" CR));
  char* gatewayRFM69[8] = {"sensor", "gatewayRFM69", "", "", jsonVal, "", "", ""};



  createDiscovery(gatewayRFM69[0],
                  subjectRFM69toMQTT, gatewayRFM69[1], (char*)getUniqueId(gatewayRFM69[1], gatewayRFM69[2]).c_str(),
                  will_Topic, gatewayRFM69[3], gatewayRFM69[4],
                  gatewayRFM69[5], gatewayRFM69[6], gatewayRFM69[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZgatewayLORA

  Log.trace(F("gatewayLORADiscovery" CR));
  char* gatewayLORA[8] = {"sensor", "gatewayLORA", "", "", jsonMsg, "", "", ""};



  createDiscovery(gatewayLORA[0],
                  subjectLORAtoMQTT, gatewayLORA[1], (char*)getUniqueId(gatewayLORA[1], gatewayLORA[2]).c_str(),
                  will_Topic, gatewayLORA[3], gatewayLORA[4],
                  gatewayLORA[5], gatewayLORA[6], gatewayLORA[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );

  createDiscovery("switch",
                  subjectLORAtoMQTT, "LORA: CRC", (char*)getUniqueId("enablecrc", "").c_str(),
                  will_Topic, "", "{{ value_json.enablecrc }}",
                  "{\"enablecrc\":true,\"save\":true}", "{\"enablecrc\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoLORAset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );

  createDiscovery("switch",
                  subjectLORAtoMQTT, "LORA: Invert IQ", (char*)getUniqueId("invertiq", "").c_str(),
                  will_Topic, "", "{{ value_json.invertiq }}",
                  "{\"invertiq\":true,\"save\":true}", "{\"invertiq\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoLORAset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );

  createDiscovery("switch",
                  subjectLORAtoMQTT, "LORA: Only Known", (char*)getUniqueId("onlyknown", "").c_str(),
                  will_Topic, "", "{{ value_json.onlyknown }}",
                  "{\"onlyknown\":true,\"save\":true}", "{\"onlyknown\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoLORAset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
# endif

# ifdef ZgatewaySRFB

  Log.trace(F("gatewaySRFBDiscovery" CR));
  char* gatewaySRFB[8] = {"sensor", "gatewaySRFB", "", "", jsonVal, "", "", ""};



  createDiscovery(gatewaySRFB[0],
                  subjectSRFBtoMQTT, gatewaySRFB[1], (char*)getUniqueId(gatewaySRFB[1], gatewaySRFB[2]).c_str(),
                  will_Topic, gatewaySRFB[3], gatewaySRFB[4],
                  gatewaySRFB[5], gatewaySRFB[6], gatewaySRFB[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZgatewayPilight

  Log.trace(F("gatewayPilightDiscovery" CR));
  char* gatewayPilight[8] = {"sensor", "gatewayPilight", "", "", jsonMsg, "", "", ""};



  createDiscovery(gatewayPilight[0],
                  subjectPilighttoMQTT, gatewayPilight[1], (char*)getUniqueId(gatewayPilight[1], gatewayPilight[2]).c_str(),
                  will_Topic, gatewayPilight[3], gatewayPilight[4],
                  gatewayPilight[5], gatewayPilight[6], gatewayPilight[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZgatewayIR

  Log.trace(F("gatewayIRDiscovery" CR));
  char* gatewayIR[8] = {"sensor", "gatewayIR", "", "", jsonVal, "", "", ""};



  createDiscovery(gatewayIR[0],
                  subjectIRtoMQTT, gatewayIR[1], (char*)getUniqueId(gatewayIR[1], gatewayIR[2]).c_str(),
                  will_Topic, gatewayIR[3], gatewayIR[4],
                  gatewayIR[5], gatewayIR[6], gatewayIR[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef Zgateway2G

  Log.trace(F("gateway2GDiscovery" CR));
  char* gateway2G[8] = {"sensor", "gateway2G", "", "", jsonMsg, "", "", ""};



  createDiscovery(gateway2G[0],
                  subject2GtoMQTT, gateway2G[1], (char*)getUniqueId(gateway2G[1], gateway2G[2]).c_str(),
                  will_Topic, gateway2G[3], gateway2G[4],
                  gateway2G[5], gateway2G[6], gateway2G[7],
                  0, Gateway_AnnouncementMsg, will_Message, true, "",
                  "", "", "", "", false,
                  stateClassNone
  );
# endif

# ifdef ZgatewayBT
# ifdef ESP32

  createDiscovery("number",
                  subjectBTtoMQTT, "BT: Connect interval", (char*)getUniqueId("intervalcnct", "").c_str(),
                  will_Topic, "", "{{ value_json.intervalcnct/60000 }}",
                  "{\"intervalcnct\":{{value*60000}},\"save\":true}", "", "min",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("number",
                  subjectBTtoMQTT, "BT: Scan duration", (char*)getUniqueId("scanduration", "").c_str(),
                  will_Topic, "", "{{ value_json.scanduration/1000 }}",
                  "{\"scanduration\":{{value*1000}},\"save\":true}", "", "s",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("button",
                  will_Topic, "BT: Force scan", (char*)getUniqueId("force_scan", "").c_str(),
                  will_Topic, "", "",
                  "{\"interval\":0}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("button",
                  will_Topic, "BT: Erase config", (char*)getUniqueId("erase_bt_config", "").c_str(),
                  will_Topic, "", "",
                  "{\"erase\":true}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone
  );
  createDiscovery("switch",
                  subjectBTtoMQTT, "BT: Publish only sensors", (char*)getUniqueId("only_sensors", "").c_str(),
                  will_Topic, "", "{{ value_json.onlysensors }}",
                  "{\"onlysensors\":true,\"save\":true}", "{\"onlysensors\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
  createDiscovery("switch",
                  subjectBTtoMQTT, "BT: Adaptive scan", (char*)getUniqueId("adaptive_scan", "").c_str(),
                  will_Topic, "", "{{ value_json.adaptivescan }}",
                  "{\"adaptivescan\":true,\"save\":true}", "{\"adaptivescan\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
  createDiscovery("switch",
                  subjectBTtoMQTT, "BT: Enabled", (char*)getUniqueId("enabled", "").c_str(),
                  will_Topic, "", "{{ value_json.enabled }}",
                  "{\"enabled\":true,\"save\":true}", "{\"enabled\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );

#define EntitiesCount 9
  const char* obsoleteEntities[EntitiesCount][2] = {

      {"switch", "active_scan"},

      {"number", "scanbcnct"},

      {"switch", "restart"},
      {"switch", "erase"},
      {"switch", "force_scan"},
      {"sensor", "interval"},
      {"sensor", "scanbcnct"},
      {"switch", "ohdiscovery"},
      {"switch", "discovery"}};

  for (int i = 0; i < EntitiesCount; i++) {
    eraseTopic(obsoleteEntities[i][0], (char*)getUniqueId(obsoleteEntities[i][1], "").c_str());
  }

  btScanParametersDiscovery();

  btPresenceParametersDiscovery();

  createDiscovery("switch",
                  subjectBTtoMQTT, "BT: Publish HASS presence", (char*)getUniqueId("hasspresence", "").c_str(),
                  will_Topic, "", "{{ value_json.hasspresence }}",
                  "{\"hasspresence\":true,\"save\":true}", "{\"hasspresence\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
  createDiscovery("switch",
                  subjectBTtoMQTT, "BT: Publish Advertisement data", (char*)getUniqueId("pubadvdata", "").c_str(),
                  will_Topic, "", "{{ value_json.pubadvdata }}",
                  "{\"pubadvdata\":true,\"save\":true}", "{\"pubadvdata\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
  createDiscovery("switch",
                  subjectBTtoMQTT, "BT: Connect to devices", (char*)getUniqueId("bleconnect", "").c_str(),
                  will_Topic, "", "{{ value_json.bleconnect }}",
                  "{\"bleconnect\":true,\"save\":true}", "{\"bleconnect\":false,\"save\":true}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", false,
                  stateClassNone,
                  "false", "true"
  );
# if DEFAULT_LOW_POWER_MODE != -1
  createDiscovery("switch",
                  subjectSYStoMQTT, "SYS: Low Power Mode command", (char*)getUniqueId("lowpowermode", "").c_str(),
                  will_Topic, "", "{{ value_json.lowpowermode | bool }}",
                  "{\"lowpowermode\":2}", "{\"lowpowermode\":0}", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoBTset,
                  "", "", "", "", true,
                  stateClassNone,
                  "false", "true"
  );
# else

  eraseTopic("switch", (char*)getUniqueId("lowpowermode", "").c_str());
# endif
# endif
# endif
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorADC.ino"
# 29 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorADC.ino"
#include "User_config.h"

#ifdef ZsensorADC

# if defined(ESP8266)
ADC_MODE(ADC_TOUT);
# endif

static int persistedadc = -1024;


unsigned long timeadc = 0;
unsigned long timeadcpub = 0;
void setupADC() {
  Log.notice(F("ADC_GPIO: %d" CR), ADC_GPIO);
}

void MeasureADC() {
  if (millis() - timeadc > TimeBetweenReadingADC) {
# if defined(ESP8266)
    yield();
    analogRead(ADC_GPIO);
# endif
    timeadc = millis();
    int val = analogRead(ADC_GPIO);
    int sum_val = val;
    if (NumberOfReadingsADC > 1) {
      for (int i = 1; i < NumberOfReadingsADC; i++) {
        sum_val += analogRead(ADC_GPIO);
      }
      val = sum_val / NumberOfReadingsADC;
    }
    if (isnan(val)) {
      Log.error(F("Failed to read from ADC !" CR));
    } else {
      if (val >= persistedadc + ThresholdReadingADC || val <= persistedadc - ThresholdReadingADC || (MinTimeInSecBetweenPublishingADC > 0 && millis() - timeadcpub > (MinTimeInSecBetweenPublishingADC * 1000UL))) {
        timeadcpub = millis();
        Log.trace(F("Creating ADC buffer" CR));
        StaticJsonDocument<JSON_MSG_BUFFER> ADCdataBuffer;
        JsonObject ADCdata = ADCdataBuffer.to<JsonObject>();
        ADCdata["adc"] = (int)val;
        if (NumberOfReadingsADC > 1) {
          ADCdata["adc_reads"] = NumberOfReadingsADC;
        }
# if defined(ADC_DIVIDER)
        float volt = 0;
# if defined(ESP32)

        volt = val * (3.3 / 4096.0);
# elif defined(ESP8266)

        volt = val * (3.3 / 1024.0);
# else

        volt = val * (5.0 / 1024.0);
# endif
        volt *= ADC_DIVIDER;

        int v = (volt * 100);
        volt = (float)v / 100.0;
        ADCdata["volt"] = (float)volt;
# endif
# if defined(ADC_CALIBRATED_SCALE_FACTOR)
        float voltage = sum_val * ADC_CALIBRATED_SCALE_FACTOR;
        ADCdata["adc_sum"] = (int)sum_val;
        ADCdata["mvolt_scaled"] = (int)voltage;
# endif
        ADCdata["origin"] = ADCTOPIC;
        handleJsonEnqueue(ADCdata);
        persistedadc = val;
      }
    }
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorAHTx0.ino"
# 38 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorAHTx0.ino"
#include "User_config.h"

#ifdef ZsensorAHTx0
# include <stdint.h>

# include "Adafruit_AHTX0.h"
# include "Wire.h"
# include "config_AHTx0.h"


unsigned long timeAHTx0 = 0;


Adafruit_AHTX0 ahtSensor;

void setupZsensorAHTx0() {
  delay(10);
  Log.notice(F("AHTx0 Initialized - begin()" CR));

# if defined(ESP32)
  Wire.begin(AHT_I2C_SDA, AHT_I2C_SCL);
  if (!ahtSensor.begin(&Wire)) {
    Log.error(F("Failed to initialize AHTx0 sensor!" CR));
  }
# else
  if (!ahtSensor.begin()) {
    Log.error(F("Failed to initialize AHTx0 sensor!" CR));
  }
# endif
}

void MeasureAHTTempHum() {
  if (millis() > (timeAHTx0 + TimeBetweenReadingAHTx0)) {
    Log.trace(F("Read AHTx0 Sensor" CR));

    timeAHTx0 = millis();
    static float persisted_aht_tempc;
    static float persisted_aht_hum;

    sensors_event_t ahtTempC, ahtHum;
    if (!ahtSensor.getEvent(&ahtHum, &ahtTempC))
    {
      Log.error(F("Failed to read from sensor AHTx0!" CR));
      return;
    }


    if (isnan(ahtTempC.temperature) || isnan(ahtHum.relative_humidity)) {
      Log.error(F("Failed to read from sensor AHTx0!" CR));
    } else {
      Log.notice(F("Creating AHTx0 buffer" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> AHTx0dataBuffer;
      JsonObject AHTx0data = AHTx0dataBuffer.to<JsonObject>();

      if (ahtTempC.temperature != persisted_aht_tempc || AHTx0_always) {
        float ahtTempF = convertTemp_CtoF(ahtTempC.temperature);
        AHTx0data["tempc"] = (float)ahtTempC.temperature;
        AHTx0data["tempf"] = (float)ahtTempF;
      } else {
        Log.notice(F("Same Temp. Don't send it" CR));
      }


      if (ahtHum.relative_humidity != persisted_aht_hum || AHTx0_always) {
        AHTx0data["hum"] = (float)ahtHum.relative_humidity;
      } else {
        Log.notice(F("Same Humidity. Don't send it" CR));
      }
      AHTx0data["origin"] = AHTTOPIC;
      handleJsonEnqueue(AHTx0data);
    }
    persisted_aht_tempc = ahtTempC.temperature;
    persisted_aht_hum = ahtHum.relative_humidity;
  }
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorBH1750.ino"
# 39 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorBH1750.ino"
#include "User_config.h"

#ifdef ZsensorBH1750
# include "Wire.h"
# include "math.h"

void setupZsensorBH1750() {
  Log.notice(F("Setup BH1750 on adress: %H" CR), BH1750_i2c_addr);
  Wire.begin();
  Wire.beginTransmission(BH1750_i2c_addr);
  Wire.write(0x10);
  Wire.endTransmission();
  delay(300);
}

void MeasureLightIntensity() {
  if (millis() > (timebh1750 + TimeBetweenReadingBH1750)) {
    Log.trace(F("Creating BH1750 buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> BH1750dataBuffer;
    JsonObject BH1750data = BH1750dataBuffer.to<JsonObject>();

    timebh1750 = millis();
    unsigned int i = 0;
    static float persistedll;
    static float persistedlf;
    static float persistedlw;
    unsigned int Lux;
    float ftcd;
    float Wattsm2;


    Wire.requestFrom(BH1750_i2c_addr, 2);
    if (Wire.available() != 2) {
      Log.error(F("Failed to read from LightSensor BH1750!" CR));
    } else {
      i = Wire.read();
      i <<= 8;
      i |= Wire.read();


      Lux = i / 1.2;
      ftcd = Lux / 10.764;
      Wattsm2 = Lux / 683.0;


      if (Lux != persistedll || bh1750_always) {
        BH1750data["lux"] = (unsigned int)Lux;
      } else {
        Log.trace(F("Same lux don't send it" CR));
      }


      if (ftcd != persistedlf || bh1750_always) {
        BH1750data["ftcd"] = (unsigned int)ftcd;
      } else {
        Log.trace(F("Same ftcd don't send it" CR));
      }


      if (Wattsm2 != persistedlw || bh1750_always) {
        BH1750data["wattsm2"] = (unsigned int)Wattsm2;
      } else {
        Log.trace(F("Same wattsm2 don't send it" CR));
      }
      BH1750data["origin"] = subjectBH1750toMQTT;
      handleJsonEnqueue(BH1750data);
    }
    persistedll = Lux;
    persistedlf = ftcd;
    persistedlw = Wattsm2;
  }
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorBME280.ino"
# 40 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorBME280.ino"
#include "User_config.h"

#ifdef ZsensorBME280
# include <stdint.h>

# include "SparkFunBME280.h"
# include "Wire.h"


BME280 mySensor;

void setupZsensorBME280() {
# if defined(ESP8266) || defined(ESP32)

  Wire.begin(BME280_PIN_SDA, BME280_PIN_SCL);
# else
  Wire.begin();
# endif

  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = BME280_i2c_addr;
  Log.notice(F("Setup BME280/BMP280 on address: %X" CR), BME280_i2c_addr);







  mySensor.settings.runMode = 3;
# 81 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorBME280.ino"
  mySensor.settings.tStandby = 1;
# 90 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorBME280.ino"
  mySensor.settings.filter = 4;





  mySensor.settings.tempOverSample = BME280TemperatureOversample;





  mySensor.settings.pressOverSample = BME280PressureOversample;





  mySensor.settings.humidOverSample = BME280HumidityOversample;





  mySensor.settings.tempCorrection = BME280Correction;

  delay(10);

  int ret = mySensor.begin();
  if (ret == 0x60) {
    Log.notice(F("Bosch BME280 successfully initialized: %X" CR), ret);
  } else if (ret == 0x58) {
    Log.notice(F("Bosch BMP280 successfully initialized: %X" CR), ret);
  } else {
    Log.notice(F("Bosch BME280/BMP280 failed: %X" CR), ret);
  }
}

void MeasureTempHumAndPressure() {
  if (millis() > (timebme280 + TimeBetweenReadingbme280)) {
    timebme280 = millis();
    static float persisted_bme_tempc;
    static float persisted_bme_tempf;
    static float persisted_bme_hum;
    static float persisted_bme_pa;
    static float persisted_bme_altim;
    static float persisted_bme_altift;

    float BmeTempC = mySensor.readTempC();
    float BmeTempF = mySensor.readTempF();
    float BmeHum = mySensor.readFloatHumidity();
    float BmePa = mySensor.readFloatPressure();
    float BmeAltiM = mySensor.readFloatAltitudeMeters();
    float BmeAltiFt = mySensor.readFloatAltitudeFeet();


    if (isnan(BmeTempC) || isnan(BmeTempF) || isnan(BmeHum) || isnan(BmePa) || isnan(BmeAltiM) || isnan(BmeAltiFt)) {
      Log.error(F("Failed to read from BME280/BMP280!" CR));
    } else {
      Log.trace(F("Creating BME280/BMP280 buffer" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> BME280dataBuffer;
      JsonObject BME280data = BME280dataBuffer.to<JsonObject>();

      if (BmeTempC != persisted_bme_tempc || bme280_always) {
        BME280data["tempc"] = (float)BmeTempC;
      } else {
        Log.trace(F("Same Degrees C don't send it" CR));
      }


      if (BmeTempF != persisted_bme_tempf || bme280_always) {
        BME280data["tempf"] = (float)BmeTempF;
      } else {
        Log.trace(F("Same Degrees F don't send it" CR));
      }


      if (BmeHum != persisted_bme_hum || bme280_always) {
        BME280data["hum"] = (float)BmeHum;
      } else {
        Log.trace(F("Same Humidity don't send it" CR));
      }


      if (BmePa != persisted_bme_pa || bme280_always) {
        BME280data["pa"] = (float)BmePa;
      } else {
        Log.trace(F("Same Pressure don't send it" CR));
      }


      if (BmeAltiM != persisted_bme_altim || bme280_always) {
        Log.trace(F("Sending Altitude Meter to MQTT" CR));
        BME280data["altim"] = (float)BmeAltiM;
      } else {
        Log.trace(F("Same Altitude Meter don't send it" CR));
      }


      if (BmeAltiFt != persisted_bme_altift || bme280_always) {
        BME280data["altift"] = (float)BmeAltiFt;
      } else {
        Log.trace(F("Same Altitude Feet don't send it" CR));
      }
      BME280data["origin"] = BMETOPIC;
      handleJsonEnqueue(BME280data);
    }

    persisted_bme_tempc = BmeTempC;
    persisted_bme_tempf = BmeTempF;
    persisted_bme_hum = BmeHum;
    persisted_bme_pa = BmePa;
    persisted_bme_altim = BmeAltiM;
    persisted_bme_altift = BmeAltiFt;
  }
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorC37_YL83_HMRD.ino"
# 24 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorC37_YL83_HMRD.ino"
#include "User_config.h"

#ifdef ZsensorC37_YL83_HMRD


unsigned long timeC37YL83HMRD = 0;
unsigned int persistedanalog = 0;
unsigned int persisteddigital = 0;

void setupZsensorC37_YL83_HMRD() {
  pinMode(C37_YL83_HMRD_Digital_GPIO, INPUT);
  Log.trace(F("C37_YL83_HMRD: digital configured pin: %d" CR), C37_YL83_HMRD_Digital_GPIO);

  pinMode(C37_YL83_HMRD_Analog_GPIO, INPUT);
  Log.trace(F("C37_YL83_HMRD: Analog configured pin: %d" CR), C37_YL83_HMRD_Analog_GPIO);

# ifdef C37_YL83_HMRD_Analog_RESOLUTION
  analogReadResolution(C37_YL83_HMRD_Analog_RESOLUTION);
  Log.trace(F("C37_YL83_HMRD: resolution: %d" CR), C37_YL83_HMRD_Analog_RESOLUTION);
# endif
}

void MeasureC37_YL83_HMRDWater() {
  if (millis() > (timeC37YL83HMRD + C37_YL83_HMRD_INTERVAL_SEC)) {
    timeC37YL83HMRD = millis();
    static int persistedanalog;
    static int persisteddigital;

    int sensorDigitalValue = digitalRead(C37_YL83_HMRD_Digital_GPIO);
    int sensorAnalogValue = analogRead(C37_YL83_HMRD_Analog_GPIO);

    Log.trace(F("Creating C37_YL83_HMRD buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> C37_YL83_HMRDdataBuffer;
    JsonObject C37_YL83_HMRDdata = C37_YL83_HMRDdataBuffer.to<JsonObject>();
    if (sensorDigitalValue != persisteddigital || C37_YL83_HMRD_ALWAYS) {
      C37_YL83_HMRDdata["detected"] = (sensorDigitalValue == 1 ? "false" : "true");
    } else {
      Log.trace(F("Same digital don't send it" CR));
    }
    if (sensorAnalogValue != persistedanalog || C37_YL83_HMRD_ALWAYS) {
      C37_YL83_HMRDdata["reading"] = sensorAnalogValue;
    } else {
      Log.trace(F("Same analog don't send it" CR));
    }
    if (C37_YL83_HMRDdata.size() > 0) {
      C37_YL83_HMRDdata["origin"] = C37_YL83_HMRD_TOPIC;
      handleJsonEnqueue(C37_YL83_HMRDdata);
      delay(10);
# if defined(DEEP_SLEEP_IN_US) || defined(ESP32_EXT0_WAKE_PIN)
      if (sensorDigitalValue == 1)
        ready_to_sleep = true;
# endif
    }
    persistedanalog = sensorAnalogValue;
    persisteddigital = sensorDigitalValue;
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorDHT.ino"
# 30 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorDHT.ino"
#include "User_config.h"

#ifdef ZsensorDHT
# include <DHT.h>
# include <DHT_U.h>

DHT dht(DHT_RECEIVER_GPIO, DHT_SENSOR_TYPE);


unsigned long timedht = 0;

void setupDHT() {
  Log.notice(F("Reading DHT on pin: %d" CR), DHT_RECEIVER_GPIO);
}

void MeasureTempAndHum() {
  if (millis() > (timedht + TimeBetweenReadingDHT)) {
    timedht = millis();
    static float persistedh;
    static float persistedt;
    float h = dht.readHumidity();

    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Log.error(F("Failed to read from DHT sensor!" CR));
    } else {
      Log.trace(F("Creating DHT buffer" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> DHTdataBuffer;
      JsonObject DHTdata = DHTdataBuffer.to<JsonObject>();
      if (h != persistedh || dht_always) {
        DHTdata["hum"] = (float)h;
      } else {
        Log.trace(F("Same hum don't send it" CR));
      }
      if (t != persistedt || dht_always) {
        DHTdata["tempc"] = (float)t;
        DHTdata["tempf"] = dht.convertCtoF(t);
      } else {
        Log.trace(F("Same temp don't send it" CR));
      }
      DHTdata["origin"] = DHTTOPIC;
      handleJsonEnqueue(DHTdata);
    }
    persistedh = h;
    persistedt = t;
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorDS1820.ino"
# 26 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorDS1820.ino"
#include "User_config.h"

#ifdef ZsensorDS1820
# include <DallasTemperature.h>
# include <OneWire.h>

OneWire owbus(DS1820_OWBUS_GPIO);
DallasTemperature ds1820(&owbus);
DeviceAddress ds1820_address, ds1820_devices[OW_MAX_SENSORS];

static uint8_t ds1820_count = 0;
static uint8_t ds1820_resolution[OW_MAX_SENSORS];
static String ds1820_type[OW_MAX_SENSORS];
static String ds1820_addr[OW_MAX_SENSORS];

void setupZsensorDS1820() {
  Log.trace(F("DS1820: configured pin: %d for 1-wire bus" CR), DS1820_OWBUS_GPIO);
  ds1820.begin();


  uint8_t numDevicesOnBus = ds1820.getDeviceCount();


  Log.notice(F("DS1820: Found %d devices" CR), numDevicesOnBus);


  for (int deviceIndex = 0; deviceIndex < numDevicesOnBus && ds1820_count < OW_MAX_SENSORS; deviceIndex++) {

    if (ds1820.getAddress(ds1820_address, deviceIndex) && ds1820.validFamily(ds1820_address)) {
      ds1820_addr[ds1820_count] = String("0x");
      for (uint8_t i = 0; i < 8; i++) {
        if (ds1820_address[i] < 0x10) ds1820_addr[ds1820_count] += String("0");
        ds1820_devices[ds1820_count][i] = ds1820_address[i];
        ds1820_addr[ds1820_count] += String(ds1820_address[i], HEX);
      }


      if (ds1820_address[0] == 0x10) {





        ds1820_type[ds1820_count] = String("DS1820/DS18S20");
      } else if (ds1820_address[0] == 0x28) {

        ds1820_type[ds1820_count] = String("DS18B20");
        ds1820.setResolution(ds1820_address, DS1820_RESOLUTION);
      } else if (ds1820_address[0] == 0x22) {
        ds1820_type[ds1820_count] = String("DS1822");
        ds1820.setResolution(ds1820_address, DS1820_RESOLUTION);
      } else {
        ds1820_type[ds1820_count] = String("DS1825");
        ds1820.setResolution(ds1820_address, DS1820_RESOLUTION);
      }
      ds1820_resolution[ds1820_count] = ds1820.getResolution(ds1820_address);
      Log.trace(F("DS1820: Device %d, Type: %s, Address: %s, Resolution: %d" CR),
                ds1820_count,
                (char*)ds1820_type[ds1820_count].c_str(),
                (char*)ds1820_addr[ds1820_count].c_str(),
                ds1820_resolution[ds1820_count]);
      ds1820_count++;
    }
  }

  if (ds1820.getDS18Count() == 0) {
    Log.error(F("DS1820: Failed to enumerate sensors on 1-wire bus. Check your GPIO assignment!" CR));
  }



  ds1820.setWaitForConversion(false);
}

void pubOneWire_HADiscovery() {

# ifdef ZmqttDiscovery

  if (SYSConfig.discovery) {
    for (int index = 0; index < ds1820_count; index++) {
      createDiscovery("sensor",
                      (char*)(String(OW_TOPIC) + "/" + ds1820_addr[index]).c_str(),
                      (char*)("DS12B20_" + String(index + 1) + "_c").c_str(),
                      (char*)(ds1820_addr[index] + "_c").c_str(),
                      will_Topic,
                      "temperature",
                      jsonTempc,
                      "", "", "°C",
                      0, "", "", true, "",
                      "", "", "", "", false,
                      stateClassMeasurement
      );
    }
  }
# endif
}

void MeasureDS1820Temp() {
  static float persisted_temp[OW_MAX_SENSORS];
  static unsigned long timeDS1820 = 0;
  static boolean triggeredConversion = false;
  float current_temp[OW_MAX_SENSORS];



  if (!triggeredConversion && ((millis() - timeDS1820) > (DS1820_INTERVAL_SEC * 1000UL - DS1820_CONV_TIME))) {
    Log.trace(F("DS1820: Trigger temperature conversion..." CR));
    ds1820.requestTemperatures();
    triggeredConversion = true;
  } else if (triggeredConversion && ((millis() - timeDS1820) > DS1820_INTERVAL_SEC * 1000UL)) {
    timeDS1820 = millis();
    triggeredConversion = false;

    if (ds1820_count < 1) {
      Log.error(F("DS1820: Failed to identify any temperature sensors on 1-wire bus during setup!" CR));
    } else {
      Log.trace(F("DS1820: Reading temperature(s) from %d sensor(s)..." CR), ds1820_count);
      StaticJsonDocument<JSON_MSG_BUFFER> DS1820dataBuffer;
      JsonObject DS1820data = DS1820dataBuffer.to<JsonObject>();

      for (uint8_t i = 0; i < ds1820_count; i++) {
        current_temp[i] = round(ds1820.getTempC(ds1820_devices[i]) * 10) / 10.0;
        if (current_temp[i] == -127) {
          Log.error(F("DS1820: Device %s currently disconnected!" CR), (char*)ds1820_addr[i].c_str());
        } else if (DS1820_ALWAYS || current_temp[i] != persisted_temp[i]) {
          DS1820data["tempf"] = (float)DallasTemperature::toFahrenheit(current_temp[i]);
          DS1820data["tempc"] = (float)current_temp[i];

          if (DS1820_DETAILS) {
            DS1820data["type"] = ds1820_type[i];
            DS1820data["res"] = String(ds1820_resolution[i]) + String("bit");
            DS1820data["addr"] = ds1820_addr[i];
          }
          String origin = String(OW_TOPIC) + "/" + ds1820_addr[i];
          DS1820data["origin"] = origin;
          handleJsonEnqueue(DS1820data);
          delay(10);
# if defined(DEEP_SLEEP_IN_US) || defined(ESP32_EXT0_WAKE_PIN)
          ready_to_sleep = true;
# endif
        } else {
          Log.trace(F("DS1820: Temperature for device %s didn't change, don't publish it." CR), (char*)ds1820_addr[i].c_str());
        }
        persisted_temp[i] = current_temp[i];
      }
    }
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorGPIOInput.ino"
# 29 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorGPIOInput.ino"
#include "User_config.h"

#ifdef ZsensorGPIOInput
# if defined(TRIGGER_GPIO) && INPUT_GPIO == TRIGGER_GPIO
unsigned long resetTime = 0;
# endif
unsigned long lastDebounceTime = 0;
int InputState = 3;
int previousInputState = 3;

void setupGPIOInput() {
  Log.notice(F("Reading GPIO at pin: %d" CR), INPUT_GPIO);
  pinMode(INPUT_GPIO, GPIO_INPUT_TYPE);
}

void MeasureGPIOInput() {
  int reading = digitalRead(INPUT_GPIO);






  if (reading != previousInputState) {

    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > GPIOInputDebounceDelay) {


# if defined(ESP8266) || defined(ESP32)
    yield();
# endif
# if defined(TRIGGER_GPIO) && INPUT_GPIO == TRIGGER_GPIO && !defined(ESPWifiManualSetup)
    if (reading == LOW) {
      if (resetTime == 0) {
        resetTime = millis();
      } else if ((millis() - resetTime) > 3000) {
        Log.trace(F("Button Held" CR));
        InfoIndicatorOFF();
        SendReceiveIndicatorOFF();

# ifdef ZactuatorONOFF
        uint8_t level = digitalRead(ACTUATOR_ONOFF_GPIO);
        if (level == ACTUATOR_ON) {
          ActuatorTrigger();
        }
# endif
        Log.notice(F("Erasing ESP Config, restarting" CR));
        setup_wifimanager(true);
      }
    } else {
      resetTime = 0;
    }
# endif

    if (reading != InputState) {
      Log.trace(F("Creating GPIOInput buffer" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> GPIOdataBuffer;
      JsonObject GPIOdata = GPIOdataBuffer.to<JsonObject>();
      if (InputState == HIGH) {
        GPIOdata["gpio"] = "HIGH";
      }
      if (InputState == LOW) {
        GPIOdata["gpio"] = "LOW";
      }
      GPIOdata["origin"] = subjectGPIOInputtoMQTT;
      handleJsonEnqueue(GPIOdata);

# if defined(ZactuatorONOFF) && defined(ACTUATOR_TRIGGER)

      if (InputState != 3) {
# if defined(ACTUATOR_BUTTON_TRIGGER_LEVEL)
        if (InputState == ACTUATOR_BUTTON_TRIGGER_LEVEL)
          ActuatorTrigger();
# else
        ActuatorTrigger();
# endif
      }
# endif
      InputState = reading;
    }
  }


  previousInputState = reading;
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorGPIOKeyCode.ino"
# 26 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorGPIOKeyCode.ino"
#include "User_config.h"

#ifdef ZsensorGPIOKeyCode

int InputStateGPIOKeyCode = 0x0f;
int lastInputStateGPIOKeyCode = 0x0f;
int lastLatchStateGPIOKeyCode = 0;

void setupGPIOKeyCode() {
  pinMode(GPIOKeyCode_LATCH_GPIO, INPUT_PULLUP);
  pinMode(GPIOKeyCode_D0_GPIO, INPUT_PULLUP);
  pinMode(GPIOKeyCode_D1_GPIO, INPUT_PULLUP);
  pinMode(GPIOKeyCode_D2_GPIO, INPUT_PULLUP);

}

void MeasureGPIOKeyCode() {
  int latch = digitalRead(GPIOKeyCode_LATCH_GPIO);





  {


# if defined(ESP8266) || defined(ESP32)
    yield();
# endif

    if (latch > 0 && lastLatchStateGPIOKeyCode != latch) {
      int reading = digitalRead(GPIOKeyCode_D0_GPIO) | (digitalRead(GPIOKeyCode_D1_GPIO) << 1) | (digitalRead(GPIOKeyCode_D2_GPIO) << 2);


      char hex[3];

      InputStateGPIOKeyCode = reading;
      sprintf(hex, "%02x", InputStateGPIOKeyCode);
      hex[2] = 0;
      Log.notice(F("GPIOKeyCode %H" CR), hex);
      pub(subjectGPIOKeyCodetoMQTT, hex);
      lastLatchStateGPIOKeyCode = latch;
    }

    if (latch != lastLatchStateGPIOKeyCode) {
      lastLatchStateGPIOKeyCode = latch;
      Log.notice(F("GPIOKeyCode latch %d" CR), latch);
      if (latch == 0)
        pub(subjectGPIOKeyCodeStatetoMQTT, "done");
    }


    lastInputStateGPIOKeyCode = InputStateGPIOKeyCode;
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorHCSR04.ino"
# 30 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorHCSR04.ino"
#include "User_config.h"

#ifdef ZsensorHCSR04

unsigned long timeHCSR04 = 0;

void setupHCSR04() {
  Log.notice(F("HCSR04 trigger pin: %d" CR), HCSR04_TRI_GPIO);
  Log.notice(F("HCSR04 echo pin: %d" CR), HCSR04_ECH_GPIO);
  pinMode(HCSR04_TRI_GPIO, OUTPUT);
  pinMode(HCSR04_ECH_GPIO, INPUT);
}

void MeasureDistance() {
  if (millis() > (timeHCSR04 + TimeBetweenReadingHCSR04)) {
    timeHCSR04 = millis();
    Log.trace(F("Creating HCSR04 buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> HCSR04dataBuffer;
    JsonObject HCSR04data = HCSR04dataBuffer.to<JsonObject>();
    digitalWrite(HCSR04_TRI_GPIO, LOW);
    delayMicroseconds(2);
    digitalWrite(HCSR04_TRI_GPIO, HIGH);
    delayMicroseconds(10);
    digitalWrite(HCSR04_TRI_GPIO, LOW);
    unsigned long duration = pulseIn(HCSR04_ECH_GPIO, HIGH);
    if (isnan(duration)) {
      Log.error(F("Failed to read from HC SR04 sensor!" CR));
    } else {
      static unsigned int distance = 99999;
      unsigned int d = duration / 58.2;
      HCSR04data["distance"] = (int)d;
      if (d > distance) {
        HCSR04data["direction"] = "away";
        Log.trace(F("HC SR04 Distance changed" CR));
      } else if (d < distance) {
        HCSR04data["direction"] = "towards";
        Log.trace(F("HC SR04 Distance changed" CR));
      } else if (HCSR04_always) {
        HCSR04data["direction"] = "static";
        Log.trace(F("HC SR04 Distance hasn't changed" CR));
      }
      distance = d;
      handleJsonEnqueue(HCSR04data);
    }
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorHCSR501.ino"
# 29 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorHCSR501.ino"
#include "User_config.h"

#ifdef ZsensorHCSR501

void setupHCSR501() {
  Log.notice(F("HCSR501 pin: %d" CR), HCSR501_GPIO);
  pinMode(HCSR501_GPIO, INPUT);
# ifdef HCSR501_LED_NOTIFY_GPIO
  pinMode(HCSR501_LED_NOTIFY_GPIO, OUTPUT);
  digitalWrite(HCSR501_LED_NOTIFY_GPIO, LOW);
# endif
}

void MeasureHCSR501() {
  if (millis() > TimeBeforeStartHCSR501) {
    StaticJsonDocument<JSON_MSG_BUFFER> HCSR501dataBuffer;
    JsonObject HCSR501data = HCSR501dataBuffer.to<JsonObject>();
    static int pirState = LOW;
    int PresenceValue = digitalRead(HCSR501_GPIO);
# if defined(ESP8266) || defined(ESP32)
    yield();
# endif
    if (PresenceValue == HIGH) {
      if (pirState == LOW) {

        HCSR501data["presence"] = "true";
        pirState = HIGH;
      }
    } else {
      if (pirState == HIGH) {

        HCSR501data["presence"] = "false";
        pirState = LOW;
      }
    }
# ifdef HCSR501_LED_NOTIFY_GPIO
    digitalWrite(HCSR501_LED_NOTIFY_GPIO, pirState == HCSR501_LED_ON);
# endif
    if (HCSR501data.size() > 0) {
      HCSR501data["origin"] = subjectHCSR501toMQTT;
      handleJsonEnqueue(HCSR501data);
    }
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorHTU21.ino"
# 38 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorHTU21.ino"
#include "User_config.h"

#ifdef ZsensorHTU21
# include <stdint.h>

# include "SparkFunHTU21D.h"
# include "Wire.h"
# include "config_HTU21.h"


unsigned long timehtu21 = 0;


HTU21D htuSensor;

void setupZsensorHTU21() {
  delay(10);
  Log.notice(F("HTU21 Initialized - begin()" CR));

# if defined(ESP32)
  Wire.begin(I2C_SDA, I2C_SCL);
  htuSensor.begin(Wire);
# else
  htuSensor.begin();
# endif
}

void MeasureTempHum() {
  if (millis() > (timehtu21 + TimeBetweenReadinghtu21)) {
    Log.trace(F("Read HTU21 Sensor" CR));

    timehtu21 = millis();
    static float persisted_htu_tempc;
    static float persisted_htu_hum;

    float HtuTempC = htuSensor.readTemperature();
    float HtuHum = htuSensor.readHumidity();

    if (HtuTempC >= 998 || HtuHum >= 998) {
      Log.error(F("Failed to read from sensor HTU21!" CR));
      return;
    }


    if (isnan(HtuTempC) || isnan(HtuHum)) {
      Log.error(F("Failed to read from sensor HTU21!" CR));
    } else {
      Log.notice(F("Creating HTU21 buffer" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> HTU21dataBuffer;
      JsonObject HTU21data = HTU21dataBuffer.to<JsonObject>();

      if (HtuTempC != persisted_htu_tempc || htu21_always) {
        float HtuTempF = (HtuTempC * 1.8) + 32;
        HTU21data["tempc"] = (float)HtuTempC;
        HTU21data["tempf"] = (float)HtuTempF;
      } else {
        Log.notice(F("Same Temp. Don't send it" CR));
      }


      if (HtuHum != persisted_htu_hum || htu21_always) {
        HTU21data["hum"] = (float)HtuHum;
      } else {
        Log.notice(F("Same Humidity. Don't send it" CR));
      }
      HTU21data["origin"] = HTUTOPIC;
      handleJsonEnqueue(HTU21data);
    }
    persisted_htu_tempc = HtuTempC;
    persisted_htu_hum = HtuHum;
  }
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorINA226.ino"
# 35 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorINA226.ino"
#include "User_config.h"

#ifdef ZsensorINA226
# include <Wire.h>

float rShunt = 0.1;
const int INA226_ADDR = 0x40;


unsigned long timeINA226 = 0;

void setupINA226() {
  Wire.begin();

  writeRegister(0x00, 0x4427);
}

void MeasureINA226() {
  if (millis() > (timeINA226 + TimeBetweenReadingINA226)) {
    timeINA226 = millis();
    Log.trace(F("Creating INA226 buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> INA226dataBuffer;
    JsonObject INA226data = INA226dataBuffer.to<JsonObject>();

    Log.trace(F("Retrieving electrical data" CR));

    float volt = readRegister(0x02) * 0.00125;

    int shuntvolt = readRegister(0x01);
    if (shuntvolt && 0x8000) {
      shuntvolt = ~shuntvolt;
      shuntvolt += 1;
      shuntvolt *= -1;
    }
    float current = shuntvolt * 0.0000025 / rShunt;
    float power = abs(volt * current);

    INA226data["volt"] = volt;
    INA226data["current"] = current;
    INA226data["power"] = power;
    INA226data["origin"] = subjectINA226toMQTT;
    handleJsonEnqueue(INA226data);
  }
}

static void writeRegister(byte reg, word value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

static word readRegister(byte reg) {
  word res = 0x0000;
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() == 0) {
    if (Wire.requestFrom(INA226_ADDR, 2) >= 2) {
      res = Wire.read() * 256;
      res += Wire.read();
    }
  }
  return res;
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorLM75.ino"
# 38 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorLM75.ino"
#include "User_config.h"

#ifdef ZsensorLM75
# include <stdint.h>

# include "Temperature_LM75_Derived.h"
# include "Wire.h"
# include "config_LM75.h"


unsigned long timelm75 = 0;


Generic_LM75 lm75Sensor;

void setupZsensorLM75() {
  delay(10);
  Log.notice(F("LM75 Initialized - begin()" CR));

# if defined(ESP32)
  Wire.begin(I2C_SDA, I2C_SCL);
# elif defined(ESP8266)
  Wire.begin();
# endif
}

void MeasureTemp() {
  if (millis() > (timelm75 + TimeBetweenReadinglm75)) {
    Log.trace(F("Read LM75 Sensor" CR));

    timelm75 = millis();
    static float persisted_lm75_tempc;

    float lm75TempC = lm75Sensor.readTemperatureC();

    if (lm75TempC >= 998) {
      Log.error(F("Failed to read from sensor LM75!" CR));
      return;
    }


    if (isnan(lm75TempC)) {
      Log.error(F("Failed to read from sensor HLM75!" CR));
    } else {
      Log.notice(F("Creating LM75 buffer" CR));
      StaticJsonDocument<JSON_MSG_BUFFER> LM75dataBuffer;
      JsonObject LM75data = LM75dataBuffer.to<JsonObject>();

      if (lm75TempC != persisted_lm75_tempc || lm75_always) {
        float lm75TempF = (lm75TempC * 1.8) + 32;
        LM75data["tempc"] = (float)lm75TempC;
        LM75data["tempf"] = (float)lm75TempF;
        LM75data["origin"] = LM75TOPIC;
        handleJsonEnqueue(LM75data);
      } else {
        Log.notice(F("Same Temp. Don't send it" CR));
      }
    }
    persisted_lm75_tempc = lm75TempC;
  }
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorMQ2.ino"
# 36 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorMQ2.ino"
#include "User_config.h"

#ifdef ZsensorMQ2

# include "Wire.h"
# include "math.h"

void setupZsensorMQ2() {
  Log.notice(F("Setup MQ2 detection on pin: %d" CR), MQ2SENSORDETECTPIN);
  pinMode(MQ2SENSORDETECTPIN, INPUT);

  Log.notice(F("Starting MQ2 calibration on pin: %d" CR), MQ2SENSORADCPIN);


  float sensorValue;
  for (int x = 0; x < 1000; x++) {
    analogRead(MQ2SENSORADCPIN);
  }
  delay(1000);

  Log.trace(F("MQ2 Initialized." CR));
}

void MeasureGasMQ2() {
  if (millis() > (timemq2 + TimeBetweenReadingmq2)) {
    timemq2 = millis();

    Log.trace(F("Creating MQ2 buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> MQ2dataBuffer;
    JsonObject MQ2data = MQ2dataBuffer.to<JsonObject>();

    MQ2data["gas"] = analogRead(MQ2SENSORADCPIN);
    MQ2data["detected"] = digitalRead(MQ2SENSORDETECTPIN) == HIGH ? "false" : "true";
    MQ2data["origin"] = subjectMQ2toMQTT;
    handleJsonEnqueue(MQ2data);
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorRN8209.ino"
# 27 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorRN8209.ino"
#ifdef ZsensorRN8209

# include "ArduinoJson.h"
# include "driver/uart.h"
# include "rn8209_flash.h"
# include "rn8209c_user.h"

extern "C" bool init_8209c_interface();

float voltage = 0;
float current = 0;
float power = 0;

TaskHandle_t rn8209TaskHandle = nullptr;

unsigned long PublishingTimerRN8209 = 0;

void rn8209_loop(void* mode) {
  while (1) {
    uint32_t temp_voltage = 0;
    uint8_t retv = rn8209c_read_voltage(&temp_voltage);
    uint8_t ret = rn8209c_read_emu_status();
    uint8_t retc = 1;
    uint8_t retp = 1;
    static float previousCurrent = 0;
    static float previousVoltage = 0;
    if (ret) {
      uint32_t temp_current = 0;
      retc = rn8209c_read_current(phase_A, &temp_current);
      if (ret == 1) {
        current = temp_current;
      } else {
        current = (int32_t)temp_current * (-1);
      }
      if (retc == 0) {
        current = current / 10000.0;
        overLimitCurrent(current);
      }
      if (retv == 0) {
        voltage = (float)temp_voltage / 1000.0;
      }
    }
    unsigned long now = millis();
    if ((now > (PublishingTimerRN8209 + TimeBetweenPublishingRN8209) ||
         !PublishingTimerRN8209 ||
         (abs(current - previousCurrent) > MinCurrentThreshold) || (abs(voltage - previousVoltage) > MinVoltageThreshold)) &&
        !ProcessLock) {
      StaticJsonDocument<JSON_MSG_BUFFER> RN8209dataBuffer;
      JsonObject RN8209data = RN8209dataBuffer.to<JsonObject>();
      if (retc == 0) {
        previousCurrent = current;
        RN8209data["current"] = round2(current);
      }
      uint32_t temp_power = 0;
      retp = rn8209c_read_power(phase_A, &temp_power);
      if (retv == 0) {
        previousVoltage = voltage;
        RN8209data["volt"] = round2(voltage);
      }
      if (ret == 1) {
        power = temp_power;
      } else {
        power = (int32_t)temp_power * (-1);
      }
      if (retp == 0) {
        power = power / 10000.0;
        RN8209data["power"] = round2(power);
      }
      PublishingTimerRN8209 = now;
      if (RN8209data) {
        RN8209data["origin"] = subjectRN8209toMQTT;
        handleJsonEnqueue(RN8209data, QueueSemaphoreTimeOutTask);
      }
    }

    delay(TimeBetweenReadingRN8209);
  }
}

void setupRN8209() {
  STU_8209C cal = {0};
  cal.Ku = RN8209_KU;
  cal.Kia = RN8209_KIA;
  cal.EC = RN8209_EC;
  set_user_param(cal);
  init_8209c_interface();
  xTaskCreate(rn8209_loop, "rn8209_loop", 5500, NULL, 10, &rn8209TaskHandle);

  Log.trace(F("ZsensorRN8209 setup done " CR));
}

#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorSHTC3.ino"
#include "User_config.h"

#ifdef ZsensorSHTC3
# include <SparkFun_SHTC3.h>

SHTC3 mySHTC3;


unsigned long timedht = 0;
void errorDecoder(SHTC3_Status_TypeDef message);

void errorDecoder(SHTC3_Status_TypeDef message)
{
  switch (message) {
    case SHTC3_Status_Nominal:
      Log.notice(F("Nominal"));
      break;
    case SHTC3_Status_Error:
      Log.error(F("Error"));
      break;
    case SHTC3_Status_CRC_Fail:
      Log.error(F("CRC Fail"));
      break;
    default:
      Log.error(F("Unknown return code"));
      break;
  }
}

void setupSHTC3() {
  Wire.begin();
  errorDecoder(mySHTC3.begin());
}

void MeasureTempAndHum() {
  if (millis() > (timedht + TimeBetweenReadingSHTC3)) {
    timedht = millis();
    static float persistedh;
    static float persistedt;
    SHTC3_Status_TypeDef result = mySHTC3.update();
    if (mySHTC3.lastStatus == SHTC3_Status_Nominal) {

      float t = mySHTC3.toDegC();
      float h = mySHTC3.toPercent();

      if (isnan(h) || isnan(t)) {
        Log.error(F("Failed to read from SHTC3 sensor!" CR));
      } else {
        Log.trace(F("Creating SHTC3 buffer" CR));
        StaticJsonDocument<JSON_MSG_BUFFER> SHTC3dataBuffer;
        JsonObject SHTC3data = SHTC3dataBuffer.to<JsonObject>();
        if (h != persistedh || shtc3_always) {
          SHTC3data["hum"] = (float)h;
        } else {
          Log.trace(F("Same hum don't send it" CR));
        }
        if (t != persistedt || shtc3_always) {
          SHTC3data["tempc"] = (float)t;
          SHTC3data["tempf"] = mySHTC3.toDegF();
        } else {
          Log.trace(F("Same temp don't send it" CR));
        }
        SHTC3data["origin"] = SHTC3TOPIC;
        handleJsonEnqueue(SHTC3data);
      }
      persistedh = h;
      persistedt = t;
    } else {
      errorDecoder(mySHTC3.lastStatus);
      Log.error(F("Failed to read from SHTC3 sensor!" CR));
    }
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorTEMT6000.ino"
# 38 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorTEMT6000.ino"
#include "User_config.h"

#ifdef ZsensorTEMT6000

# include "Wire.h"
# include "math.h"

void setupZsensorTEMT6000() {
  Log.notice(F("Setup TEMT6000 on pin: %i" CR), TEMT6000LIGHTSENSORPIN);
  pinMode(TEMT6000LIGHTSENSORPIN, INPUT);
}

void MeasureLightIntensityTEMT6000() {
  if (millis() > (timetemt6000 + TimeBetweenReadingtemt6000)) {
    static uint32_t persisted_lux;
    timetemt6000 = millis();

    Log.trace(F("Creating TEMT6000 buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> TEMT6000dataBuffer;
    JsonObject TEMT6000data = TEMT6000dataBuffer.to<JsonObject>();

    float volts = analogRead(TEMT6000LIGHTSENSORPIN) * 5.0 / 1024.0;
    float amps = volts / 10000.0;
    float microamps = amps * 1000000;
    float lux = microamps * 2.0;

    if (persisted_lux != lux || temt6000_always) {
      persisted_lux = lux;

      TEMT6000data["lux"] = (float)lux;
      TEMT6000data["ftcd"] = (float)(lux) / 10.764;
      TEMT6000data["wattsm2"] = (float)(lux) / 683.0;
      TEMT6000data["origin"] = subjectTEMT6000toMQTT;
      handleJsonEnqueue(TEMT6000data);
    } else {
      Log.trace(F("Same lux value, do not send" CR));
    }
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorTSL2561.ino"
# 40 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorTSL2561.ino"
#include "User_config.h"

#ifdef ZsensorTSL2561
# include <Adafruit_Sensor.h>
# include <Adafruit_TSL2561_U.h>

# include "Wire.h"
# include "math.h"

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void displaySensorDetails(void) {
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Log.trace(F("------------------------------------" CR));
  Log.trace(("Sensor: %s" CR), sensor.name);
  Log.trace(("Driver Ver: %s" CR), sensor.version);
  Log.trace(("Unique ID: %s" CR), sensor.sensor_id);
  Log.trace(("Max Value: %s lux" CR), sensor.max_value);
  Log.trace(("Min Value: %s lux" CR), sensor.min_value);
  Log.trace(("Resolution: %s lux" CR), sensor.resolution);
  Log.trace(F("------------------------------------" CR));
  delay(500);
}

void setupZsensorTSL2561() {
  Log.notice(F("Setup TSL2561 on adress: %H" CR), TSL2561_ADDR_FLOAT);
  Wire.begin();
  Wire.beginTransmission(TSL2561_ADDR_FLOAT);

  if (!tsl.begin()) {
    Log.error(F("No TSL2561 detected" CR));
  }




  tsl.enableAutoRange(true);



  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);

  Log.trace(F("TSL2561 Initialized. Printing detials now." CR));
  displaySensorDetails();
}

void MeasureLightIntensityTSL2561() {
  if (millis() > (timetsl2561 + TimeBetweenReadingtsl2561)) {
    static uint32_t persisted_lux;
    timetsl2561 = millis();

    Log.trace(F("Creating TSL2561 buffer" CR));
    StaticJsonDocument<JSON_MSG_BUFFER> TSL2561dataBuffer;
    JsonObject TSL2561data = TSL2561dataBuffer.to<JsonObject>();

    sensors_event_t event;
    tsl.getEvent(&event);
    if (event.light)

    {
      if (persisted_lux != event.light || tsl2561_always) {
        persisted_lux = event.light;

        TSL2561data["lux"] = (float)event.light;
        TSL2561data["ftcd"] = (float)(event.light) / 10.764;
        TSL2561data["wattsm2"] = (float)(event.light) / 683.0;
        TSL2561data["origin"] = subjectTSL12561toMQTT;
        handleJsonEnqueue(TSL2561data);
      } else {
        Log.trace(F("Same lux value, do not send" CR));
      }
    } else {
      Log.error(F("Failed to read from TSL2561" CR));
    }
  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorTouch.ino"
# 26 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZsensorTouch.ino"
#include <HardwareSerial.h>

#include "ArduinoJson.h"
#include "User_config.h"
#include "config_Touch.h"

#ifdef ZsensorTouch

# ifndef ESP32
# error Only supported on esp32
# endif


static unsigned long touchTimeRead = 0;


struct TouchStatus {
  uint8_t pin;
  uint16_t timePublished = 0;
  uint16_t timeStatusChanged = 0;
  uint16_t avg = 10 * 256;
  bool isOn = false;
  bool wasOn = false;
  int onCount = 0;
  unsigned long onTime;
};

static TouchStatus status[TOUCH_SENSORS];

void setupTouch() {
# if TOUCH_SENSORS > 0
  status[0].pin = TOUCH_GPIO_0;
# endif
# if TOUCH_SENSORS > 1
  status[1].pin = TOUCH_GPIO_1;
# endif
# if TOUCH_SENSORS > 2
  status[2].pin = TOUCH_GPIO_2;
# endif
# if TOUCH_SENSORS > 3
  status[3].pin = TOUCH_GPIO_3;
# endif
# if TOUCH_SENSORS > 4
  status[4].pin = TOUCH_GPIO_4;
# endif
# if TOUCH_SENSORS > 5
  status[5].pin = TOUCH_GPIO_5;
# endif
# if TOUCH_SENSORS > 6
  status[6].pin = TOUCH_GPIO_6;
# endif
# if TOUCH_SENSORS > 7
  status[7].pin = TOUCH_GPIO_7;
# endif
# if TOUCH_SENSORS > 8
  status[8].pin = TOUCH_GPIO_8;
# endif
# if TOUCH_SENSORS > 9
  status[9].pin = TOUCH_GPIO_9;
# endif
  for (int i = 0; i < TOUCH_SENSORS; i++) {
    Log.notice(F("TOUCH_GPIO_%d: %d" CR), i, status[i].pin);
  }
  touchTimeRead = millis() - TOUCH_TIME_BETWEEN_READINGS;
}

void MeasureTouch() {
  unsigned long now = millis();
  if (now - touchTimeRead < TOUCH_TIME_BETWEEN_READINGS) {
    return;
  }
  touchTimeRead = now;
  for (int i = 0; i < TOUCH_SENSORS; i++) {
    TouchStatus* button = &status[i];

    uint16_t val = touchRead(button->pin);
    if (val == 0) {

      continue;
    }




    uint16_t curValue = val * 256;
    bool isOn = (curValue < button->avg * (TOUCH_THRESHOLD * 256 / 100) / 256);
    if (!isOn) {
      button->avg = (button->avg * 127 + val * 256 * 1) / 128;
    }
    int lastOnCount = button->onCount;
    if (isOn) {
      ++button->onCount;
    } else {
      button->onCount = 0;
    }

    bool isStatusChanged = false;
    if (((button->wasOn && !isOn) || (!button->wasOn && isOn && button->onCount >= TOUCH_MIN_DURATION / TOUCH_TIME_BETWEEN_READINGS)) && now - button->timeStatusChanged > TOUCH_DEBOUNCE_TIME) {
      isStatusChanged = true;
      button->wasOn = isOn;
      button->timeStatusChanged = now;
      if (isOn) {
        button->onTime = now;
      }
    }


    if (now - button->timeStatusChanged > 100000) {
      button->timeStatusChanged = now - 100000;
    }
    if (isStatusChanged) {
      const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(4);
      StaticJsonDocument<JSON_MSG_CALC_BUFFER> jsonDoc;
      JsonObject touchData = jsonDoc.to<JsonObject>();
      touchData["id"] = i;
      touchData["on"] = (isOn ? 1 : 0);
      touchData["value"] = (int)(curValue / 256);
      if (!isOn) {
        touchData["onDuration"] = now - button->onTime + TOUCH_MIN_DURATION;
      }
      pub(TOUCHTOPIC, touchData);
      button->timePublished = now;
    }



  }
}
#endif
# 1 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
# 24 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
#include "User_config.h"
#if defined(ZwebUI) && defined(ESP32)
# include <ArduinoJson.h>
# include <SPIFFS.h>
# include <WebServer.h>

# include "ArduinoLog.h"
# include "config_WebContent.h"
# include "config_WebUI.h"

# if defined(ZgatewayCloud)
# include "config_Cloud.h"
# endif

# if defined(ZdisplaySSD1306)
# include "config_SSD1306.h"
# endif

uint32_t requestToken = 0;

QueueHandle_t webUIQueue;

WebServer server(80);


extern void eraseAndRestart();
extern unsigned long uptime();



#define ROW_LENGTH 1024

const uint16_t LOG_BUFFER_SIZE = 6096;
uint32_t log_buffer_pointer;
void* log_buffer_mutex;
char log_buffer[LOG_BUFFER_SIZE];

const uint16_t MAX_LOGSZ = LOG_BUFFER_SIZE - 96;
const uint16_t TOPSZ = 151;
uint8_t masterlog_level;
bool reset_web_log_flag = false;

const char* www_username = WEBUI_LOGIN;
String authFailResponse = "Authentication Failed";
bool webUISecure = WEBUI_AUTH;
boolean displayMetric = DISPLAY_METRIC;
# 89 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
class TasAutoMutex {
  SemaphoreHandle_t mutex;
  bool taken;
  int maxWait;
  const char* name;

public:
  TasAutoMutex(SemaphoreHandle_t* mutex, const char* name = "", int maxWait = 40, bool take = true);
  ~TasAutoMutex();
  void give();
  void take();
  static void init(SemaphoreHandle_t* ptr);
};


TasAutoMutex::TasAutoMutex(SemaphoreHandle_t* mutex, const char* name, int maxWait, bool take) {
  if (mutex) {
    if (!(*mutex)) {
      TasAutoMutex::init(mutex);
    }
    this->mutex = *mutex;
    this->maxWait = maxWait;
    this->name = name;
    if (take) {
      this->taken = xSemaphoreTakeRecursive(this->mutex, this->maxWait);



    }
  } else {
    this->mutex = (SemaphoreHandle_t) nullptr;
  }
}

TasAutoMutex::~TasAutoMutex() {
  if (this->mutex) {
    if (this->taken) {
      xSemaphoreGiveRecursive(this->mutex);
      this->taken = false;
    }
  }
}

void TasAutoMutex::init(SemaphoreHandle_t* ptr) {
  SemaphoreHandle_t mutex = xSemaphoreCreateRecursiveMutex();
  (*ptr) = mutex;


}

void TasAutoMutex::give() {
  if (this->mutex) {
    if (this->taken) {
      xSemaphoreGiveRecursive(this->mutex);
      this->taken = false;
    }
  }
}

void TasAutoMutex::take() {
  if (this->mutex) {
    if (!this->taken) {
      this->taken = xSemaphoreTakeRecursive(this->mutex, this->maxWait);



    }
  }
}


size_t strchrspn(const char* str1, int character) {
  size_t ret = 0;
  char* start = (char*)str1;
  char* end = strchr(str1, character);
  if (end) ret = end - start;
  return ret;
}

int WifiGetRssiAsQuality(int rssi) {
  int quality = 0;

  if (rssi <= -100) {
    quality = 0;
  } else if (rssi >= -50) {
    quality = 100;
  } else {
    quality = 2 * (rssi + 100);
  }
  return quality;
}

char* GetTextIndexed(char* destination, size_t destination_size, uint32_t index, const char* haystack) {


  char* write = destination;
  const char* read = haystack;

  index++;
  while (index--) {
    size_t size = destination_size - 1;
    write = destination;
    char ch = '.';
    while ((ch != '\0') && (ch != '|')) {
      ch = pgm_read_byte(read++);
      if (size && (ch != '|')) {
        *write++ = ch;
        size--;
      }
    }
    if (0 == ch) {
      if (index) {
        write = destination;
      }
      break;
    }
  }
  *write = '\0';
  return destination;
}

const char kUnescapeCode[] = "&><\"\'\\";
const char kEscapeCode[] PROGMEM = "&amp;|&gt;|&lt;|&quot;|&apos;|&#92;";

String HtmlEscape(const String unescaped) {
  char escaped[10];
  size_t ulen = unescaped.length();
  String result;
  result.reserve(ulen);
  for (size_t i = 0; i < ulen; i++) {
    char c = unescaped[i];
    char* p = strchr(kUnescapeCode, c);
    if (p != nullptr) {
      result += GetTextIndexed(escaped, sizeof(escaped), p - kUnescapeCode, kEscapeCode);
    } else {
      result += c;
    }
  }
  return result;
}

void AddLogData(uint32_t loglevel, const char* log_data, const char* log_data_payload = nullptr, const char* log_data_retained = nullptr) {



# ifdef ESP32


  TasAutoMutex mutex((SemaphoreHandle_t*)&log_buffer_mutex);
# endif

  char empty[2] = {0};
  if (!log_data_payload) {
    log_data_payload = empty;
  }
  if (!log_data_retained) {
    log_data_retained = empty;
  }

  if (!log_buffer) {
    return;
  }





  uint32_t log_data_len = strlen(log_data) + strlen(log_data_payload) + strlen(log_data_retained);
  char too_long[TOPSZ];
  if (log_data_len > MAX_LOGSZ) {
    snprintf_P(too_long, sizeof(too_long) - 20, PSTR("%s%s"), log_data, log_data_payload);
    snprintf_P(too_long, sizeof(too_long), PSTR("%s... %d truncated"), too_long, log_data_len);
    log_data = too_long;
    log_data_payload = empty;
    log_data_retained = empty;
  }

  log_buffer_pointer &= 0xFF;
  if (!log_buffer_pointer) {
    log_buffer_pointer++;
  }
  while (log_buffer_pointer == log_buffer[0] ||
         strlen(log_buffer) + strlen(log_data) + strlen(log_data_payload) + strlen(log_data_retained) + 4 > LOG_BUFFER_SIZE)
  {
    char* it = log_buffer;
    it++;
    it += strchrspn(it, '\1');
    it++;
    memmove(log_buffer, it, LOG_BUFFER_SIZE - (it - log_buffer));
  }
  snprintf_P(log_buffer, LOG_BUFFER_SIZE, PSTR("%s%c%c%s%s%s%s\1"),
             log_buffer, log_buffer_pointer++, '0' + loglevel, "", log_data, log_data_payload, log_data_retained);
  log_buffer_pointer &= 0xFF;
  if (!log_buffer_pointer) {
    log_buffer_pointer++;
  }
}

bool NeedLogRefresh(uint32_t req_loglevel, uint32_t index) {
  if (!log_buffer) {
    return false;
  }

# ifdef ESP32


  TasAutoMutex mutex((SemaphoreHandle_t*)&log_buffer_mutex);
# endif


  if (strlen(log_buffer) < LOG_BUFFER_SIZE / 2) {
    return false;
  }

  char* line;
  size_t len;
  if (!GetLog(req_loglevel, &index, &line, &len)) {
    return false;
  }
  return ((line - log_buffer) < LOG_BUFFER_SIZE / 4);
}

bool GetLog(uint32_t req_loglevel, uint32_t* index_p, char** entry_pp, size_t* len_p) {
  if (!log_buffer) {
    return false;
  }
  if (uptime() < 3) {
    return false;
  }

  uint32_t index = *index_p;
  if (!req_loglevel || (index == log_buffer_pointer)) {
    return false;
  }

# ifdef ESP32


  TasAutoMutex mutex((SemaphoreHandle_t*)&log_buffer_mutex);
# endif

  if (!index) {
    index = log_buffer[0];
  }

  do {
    size_t len = 0;
    uint32_t loglevel = 0;
    char* entry_p = log_buffer;
    do {
      uint32_t cur_idx = *entry_p;
      entry_p++;
      size_t tmp = strchrspn(entry_p, '\1');
      tmp++;
      if (cur_idx == index) {
        loglevel = *entry_p - '0';
        entry_p++;
        len = tmp - 1;
        break;
      }
      entry_p += tmp;
    } while (entry_p < log_buffer + LOG_BUFFER_SIZE && *entry_p != '\0');
    index++;
    if (index > 255) {
      index = 1;
    }
    *index_p = index;
    if ((len > 0) &&
        (loglevel <= req_loglevel) &&
        (masterlog_level <= req_loglevel)) {
      *entry_pp = entry_p;
      *len_p = len;
      return true;
    }
    delay(0);
  } while (index != log_buffer_pointer);
  return false;
}



# ifdef WEBUI_DEVELOPMENT

String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

bool exists(String path) {
  bool yes = false;
  File file = FILESYSTEM.open(path, "r");
  if (!file.isDirectory()) {
    yes = true;
  }
  file.close();
  return yes;
}
# endif





void handleRoot() {
  WEBUI_TRACE_LOG(F("handleRoot: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("m")) {
      if (currentWebUIMessage) {
        server.send(200, "application/json", "{t}{s}<b>" + String(currentWebUIMessage->title) + "</b>{e}{s}" + String(currentWebUIMessage->line1) + "{e}{s}" + String(currentWebUIMessage->line2) + "{e}{s}" + String(currentWebUIMessage->line3) + "{e}{s}" + String(currentWebUIMessage->line4) + "{e}</table>");
      } else {
        server.send(200, "application/json", "{t}{s}Uptime:{m}" + String(uptime()) + "{e}</table>");
      }
    } else if (server.hasArg("rst")) {
      Log.warning(F("[WebUI] Restart" CR));
      char jsonChar[100];
      serializeJson(modules, jsonChar, measureJson(modules) + 1);
      char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Restart").c_str());
      String response = String(buffer);
      response += String(restart_script);
      response += String(script);
      response += String(style);
      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, reset_body, jsonChar, gateway_name, "Restart");
      response += String(buffer);
      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
      response += String(buffer);
      server.send(200, "text/html", response);

      delay(2000);

      ESPRestart(5);
    } else {

      server.send(200, "text/plain", "00:14:36.767 RSL: RESULT = {\"Topic\":\"topic\"}");
    }
  } else {
    char jsonChar[100];
    serializeJson(modules, jsonChar, measureJson(modules) + 1);

    char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Main Menu").c_str());
    String response = String(buffer);
    response += String(root_script);
    response += String(script);
    response += String(style);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, root_body, jsonChar, gateway_name);
    response += String(buffer);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
    response += String(buffer);
    server.send(200, "text/html", response);
  }
}





void handleCN() {
  WEBUI_SECURE
  WEBUI_TRACE_LOG(F("handleCN: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleCN Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
  } else {
    char jsonChar[100];
    serializeJson(modules, jsonChar, measureJson(modules) + 1);

    char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configuration").c_str());
    String response = String(buffer);
    response += String(script);
    response += String(style);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_body, jsonChar, gateway_name);
    response += String(buffer);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
    response += String(buffer);
    server.send(200, "text/html", response);
  }
}
# 491 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
void handleWU() {
  WEBUI_TRACE_LOG(F("handleWU: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleWU Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    bool update = false;

    if (displayMetric != server.hasArg("dm")) {
      update = true;
    }
    displayMetric = server.hasArg("dm");

    if (webUISecure != server.hasArg("sw")) {
      update = true;
    }
    webUISecure = server.hasArg("sw");

    if (server.hasArg("save") && update) {
      WebUIConfig_save();
    }
  }

  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure WebUI").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);
  int logLevel = Log.getLevel();
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_webui_body, jsonChar, gateway_name, (displayMetric ? "checked" : ""), (webUISecure ? "checked" : ""));
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}
# 539 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
void handleWI() {
  WEBUI_TRACE_LOG(F("handleWI: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  String WiFiScan = "";
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleWI Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("scan")) {
      bool limitScannedNetworks = true;
      int n = WiFi.scanNetworks();

      WEBUI_TRACE_LOG(F("handleWI scan: found %d" CR), n);
      if (0 == n) {


      } else {

        int indices[n];
        for (uint32_t i = 0; i < n; i++) {
          indices[i] = i;
        }


        for (uint32_t i = 0; i < n; i++) {
          for (uint32_t j = i + 1; j < n; j++) {
            if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i])) {
              std::swap(indices[i], indices[j]);
            }
          }
        }

        uint32_t networksToShow = n;
        if ((limitScannedNetworks) && (networksToShow > MAX_WIFI_NETWORKS_TO_SHOW)) {
          networksToShow = MAX_WIFI_NETWORKS_TO_SHOW;
        }

        for (uint32_t i = 0; i < n; i++) {
          if (-1 == indices[i]) {
            continue;
          }
          String cssid = WiFi.SSID(indices[i]);
          uint32_t cschn = WiFi.channel(indices[i]);
          for (uint32_t j = i + 1; j < n; j++) {
            if ((cssid == WiFi.SSID(indices[j])) && (cschn == WiFi.channel(indices[j]))) {
              WEBUI_TRACE_LOG(F("handleWI scan: duplicate %s" CR), WiFi.SSID(indices[j]).c_str());
              indices[j] = -1;
            }
          }
        }


        for (uint32_t i = 0; i < networksToShow; i++) {
          if (-1 == indices[i]) {
            continue;
          }
          int32_t rssi = WiFi.RSSI(indices[i]);
          WEBUI_TRACE_LOG(F("D_LOG_WIFI D_SSID  %s, D_BSSID  %s,  D_CHANNEL  %d,  D_RSSI  %d" CR),
                          WiFi.SSID(indices[i]).c_str(), WiFi.BSSIDstr(indices[i]).c_str(), WiFi.channel(indices[i]), rssi);
          int quality = WifiGetRssiAsQuality(rssi);
          String ssid_copy = WiFi.SSID(indices[i]);
          if (!ssid_copy.length()) {
            ssid_copy = F("no_name");
          }

          WiFiScan += "<div><a href='#p' onclick='c(this)'>" + HtmlEscape(ssid_copy) + "</a>&nbsp;(" + WiFi.channel(indices[i]) + ")&nbsp<span class='q'>" + quality + "% (" + rssi + " dBm)</span></div>";
        }
      }
      WEBUI_TRACE_LOG(F("handleWI scan: results %s" CR), WiFiScan.c_str());

      char jsonChar[100];
      serializeJson(modules, jsonChar, measureJson(modules) + 1);

      char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure WiFi").c_str());
      String response = String(buffer);
      response += String(wifi_script);
      response += String(script);
      response += String(style);
      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_wifi_body, jsonChar, gateway_name, WiFiScan.c_str(), WiFi.SSID());
      response += String(buffer);
      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
      response += String(buffer);
      server.send(200, "text/html", response);
      return;

    } else if (server.hasArg("save")) {
      StaticJsonDocument<JSON_MSG_BUFFER> WEBtoSYSBuffer;
      JsonObject WEBtoSYS = WEBtoSYSBuffer.to<JsonObject>();
      bool update = false;
      if (server.hasArg("s1")) {
        WEBtoSYS["wifi_ssid"] = server.arg("s1");
        if (strncmp((char*)WiFi.SSID().c_str(), server.arg("s1").c_str(), parameters_size)) {
          update = true;
        }
      }
      if (server.hasArg("p1")) {
        WEBtoSYS["wifi_pass"] = server.arg("p1");
        if (strncmp((char*)WiFi.psk().c_str(), server.arg("p1").c_str(), parameters_size)) {
          update = true;
        }
      }
      if (update) {
        String topic = String(mqtt_topic) + String(gateway_name) + String(subjectMQTTtoSYSset);
        Log.warning(F("[WebUI] Save WiFi and Restart" CR));
        char jsonChar[100];
        serializeJson(modules, jsonChar, measureJson(modules) + 1);
        char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

        snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Save WiFi and Restart").c_str());
        String response = String(buffer);
        response += String(restart_script);
        response += String(script);
        response += String(style);
        snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, reset_body, jsonChar, gateway_name, "Save WiFi and Restart");
        response += String(buffer);
        snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
        response += String(buffer);
        server.send(200, "text/html", response);

        delay(2000);
        MQTTtoSYS((char*)topic.c_str(), WEBtoSYS);
        return;
      } else {
        Log.warning(F("[WebUI] No changes" CR));
      }
    }
  }
  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure WiFi").c_str());
  String response = String(buffer);
  response += String(wifi_script);
  response += String(script);
  response += String(style);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_wifi_body, jsonChar, gateway_name, WiFiScan.c_str(), WiFi.SSID());
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}
# 697 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
void handleMQ() {
  WEBUI_TRACE_LOG(F("handleMQ: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleMQ Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("save")) {
      StaticJsonDocument<JSON_MSG_BUFFER> WEBtoSYSBuffer;
      JsonObject WEBtoSYS = WEBtoSYSBuffer.to<JsonObject>();
      bool update = false;

      if (server.hasArg("mh")) {
        WEBtoSYS["mqtt_server"] = server.arg("mh");
        if (strncmp(mqtt_server, server.arg("mh").c_str(), parameters_size)) {
          update = true;
        }
      }

      if (server.hasArg("ml")) {
        WEBtoSYS["mqtt_port"] = server.arg("ml");
        if (strncmp(mqtt_port, server.arg("ml").c_str(), 6)) {
          update = true;
        }
      }

      if (server.hasArg("mu")) {
        WEBtoSYS["mqtt_user"] = server.arg("mu");
        if (strncmp(mqtt_user, server.arg("mu").c_str(), parameters_size)) {
          update = true;
        }
      }

      if (server.hasArg("mp")) {
        WEBtoSYS["mqtt_pass"] = server.arg("mp");
        if (strncmp(mqtt_pass, server.arg("mp").c_str(), parameters_size)) {
          update = true;
        }
      }


      if (mqtt_secure != server.hasArg("sc")) {
        update = true;
      }
      WEBtoSYS["mqtt_secure"] = server.hasArg("sc");

      if (!update) {
        Log.warning(F("[WebUI] clearing" CR));
        for (JsonObject::iterator it = WEBtoSYS.begin(); it != WEBtoSYS.end(); ++it) {
          WEBtoSYS.remove(it);
        }
      }

      if (server.hasArg("h")) {
        WEBtoSYS["gateway_name"] = server.arg("h");
        if (strncmp(gateway_name, server.arg("h").c_str(), parameters_size)) {
          update = true;
        }
      }

      if (server.hasArg("mt")) {
        WEBtoSYS["mqtt_topic"] = server.arg("mt");
        if (strncmp(mqtt_topic, server.arg("mt").c_str(), parameters_size)) {
          update = true;
        }
      }

# ifndef ESPWifiManualSetup
      if (update) {
        Log.warning(F("[WebUI] Save MQTT and Reconnect" CR));

        char jsonChar[100];
        serializeJson(modules, jsonChar, measureJson(modules) + 1);
        char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

        snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Save MQTT and Reconnect").c_str());
        String response = String(buffer);
        response += String(restart_script);
        response += String(script);
        response += String(style);
        snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, reset_body, jsonChar, gateway_name, "Save MQTT and Reconnect");
        response += String(buffer);
        snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
        response += String(buffer);
        server.send(200, "text/html", response);

        delay(2000);
        String topic = String(mqtt_topic) + String(gateway_name) + String(subjectMQTTtoSYSset);
        MQTTtoSYS((char*)topic.c_str(), WEBtoSYS);
        return;
      } else {
        Log.warning(F("[WebUI] No changes" CR));
      }
# endif
    }
  }

  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure MQTT").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_mqtt_body, jsonChar, gateway_name, mqtt_server, mqtt_port, mqtt_user, (mqtt_secure ? "checked" : ""), gateway_name, mqtt_topic);
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}

# ifndef ESPWifiManualSetup







void handleCG() {
  WEBUI_TRACE_LOG(F("handleCG: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  bool update = false;
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject WEBtoSYS = jsonBuffer.to<JsonObject>();

  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleCG Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("save") && server.hasArg("gp") && strcmp(ota_pass, server.arg("gp").c_str())) {
      strncpy(ota_pass, server.arg("gp").c_str(), parameters_size);
      WEBtoSYS["gw_pass"] = ota_pass;
      update = true;
    }
  }

  if (update) {
    Log.warning(F("[WebUI] Save Password and Restart" CR));

    char jsonChar[100];
    serializeJson(modules, jsonChar, measureJson(modules) + 1);
    char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Save Password and Restart").c_str());
    String response = String(buffer);
    response += String(restart_script);
    response += String(script);
    response += String(style);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, reset_body, jsonChar, gateway_name, "Save Password and Restart");
    response += String(buffer);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
    response += String(buffer);
    server.send(200, "text/html", response);

    delay(2000);
    String topic = String(mqtt_topic) + String(gateway_name) + String(subjectMQTTtoSYSset);
    MQTTtoSYS((char*)topic.c_str(), WEBtoSYS);
  } else {
    Log.warning(F("[WebUI] No changes" CR));
  }

  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure gateway").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_gateway_body, jsonChar, gateway_name, ota_pass);
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}
# endif







void handleLO() {
  WEBUI_TRACE_LOG(F("handleLO: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleLO Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("save") && server.hasArg("lo") && server.arg("lo").toInt() != Log.getLevel()) {
      Log.fatal(F("[WebUI] Log level changed to: %d" CR), server.arg("lo").toInt());
      Log.setLevel(server.arg("lo").toInt());
    }
  }

  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure Logging").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);
  int logLevel = Log.getLevel();
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_logging_body, jsonChar, gateway_name, (logLevel == 0 ? "selected" : ""), (logLevel == 1 ? "selected" : ""), (logLevel == 2 ? "selected" : ""), (logLevel == 3 ? "selected" : ""), (logLevel == 4 ? "selected" : ""), (logLevel == 5 ? "selected" : ""), (logLevel == 6 ? "selected" : ""));
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}

# ifdef ZgatewayLORA
# 931 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
void handleLA() {
  WEBUI_TRACE_LOG(F("handleLA: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleLA Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("save")) {
      StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
      JsonObject WEBtoLORA = jsonBuffer.to<JsonObject>();
      bool update = false;
      if (server.hasArg("lf")) {
        WEBtoLORA["frequency"] = server.arg("lf");
        update = true;
      }

      if (server.hasArg("lt")) {
        WEBtoLORA["txpower"] = server.arg("lt");
        update = true;
      }

      if (server.hasArg("ls")) {
        WEBtoLORA["spreadingfactor"] = server.arg("ls");
        update = true;
      }

      if (server.hasArg("lb")) {
        WEBtoLORA["signalbandwidth"] = server.arg("lb");
        update = true;
      }

      if (server.hasArg("lc")) {
        WEBtoLORA["codingrate"] = server.arg("lc");
        update = true;
      }

      if (server.hasArg("ll")) {
        WEBtoLORA["preamblelength"] = server.arg("ll");
        update = true;
      }

      if (server.hasArg("lw")) {
        WEBtoLORA["syncword"] = server.arg("lw");
        update = true;
      }

      if (server.hasArg("lr")) {
        WEBtoLORA["enablecrc"] = server.arg("lr");
        update = true;
      } else {
        WEBtoLORA["enablecrc"] = false;
        update = true;
      }

      if (server.hasArg("li")) {
        WEBtoLORA["invertiq"] = server.arg("li");
        update = true;
      } else {
        WEBtoLORA["invertiq"] = false;
        update = true;
      }

      if (server.hasArg("ok")) {
        WEBtoLORA["onlyknown"] = server.arg("ok");
        update = true;
      } else {
        WEBtoLORA["onlyknown"] = false;
        update = true;
      }
      if (update) {
        Log.notice(F("[WebUI] Save data" CR));
        WEBtoLORA["save"] = true;
        LORAConfig_fromJson(WEBtoLORA);
        stateLORAMeasures();
        Log.trace(F("[WebUI] LORAConfig end" CR));
      }
    }
  }
  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure LORA").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_lora_body,
           jsonChar,
           gateway_name,
           LORAConfig.frequency == 868000000 ? "selected" : "",
           LORAConfig.frequency == 915000000 ? "selected" : "",
           LORAConfig.frequency == 433000000 ? "selected" : "",
           LORAConfig.txPower == 0 ? "selected" : "",
           LORAConfig.txPower == 1 ? "selected" : "",
           LORAConfig.txPower == 2 ? "selected" : "",
           LORAConfig.txPower == 3 ? "selected" : "",
           LORAConfig.txPower == 4 ? "selected" : "",
           LORAConfig.txPower == 5 ? "selected" : "",
           LORAConfig.txPower == 6 ? "selected" : "",
           LORAConfig.txPower == 7 ? "selected" : "",
           LORAConfig.txPower == 8 ? "selected" : "",
           LORAConfig.txPower == 9 ? "selected" : "",
           LORAConfig.txPower == 10 ? "selected" : "",
           LORAConfig.txPower == 11 ? "selected" : "",
           LORAConfig.txPower == 12 ? "selected" : "",
           LORAConfig.txPower == 13 ? "selected" : "",
           LORAConfig.txPower == 14 ? "selected" : "",
           LORAConfig.spreadingFactor == 7 ? "selected" : "",
           LORAConfig.spreadingFactor == 8 ? "selected" : "",
           LORAConfig.spreadingFactor == 9 ? "selected" : "",
           LORAConfig.spreadingFactor == 10 ? "selected" : "",
           LORAConfig.spreadingFactor == 11 ? "selected" : "",
           LORAConfig.spreadingFactor == 12 ? "selected" : "",
           LORAConfig.signalBandwidth == 7800 ? "selected" : "",
           LORAConfig.signalBandwidth == 10400 ? "selected" : "",
           LORAConfig.signalBandwidth == 15600 ? "selected" : "",
           LORAConfig.signalBandwidth == 20800 ? "selected" : "",
           LORAConfig.signalBandwidth == 31250 ? "selected" : "",
           LORAConfig.signalBandwidth == 41700 ? "selected" : "",
           LORAConfig.signalBandwidth == 62500 ? "selected" : "",
           LORAConfig.signalBandwidth == 125000 ? "selected" : "",
           LORAConfig.signalBandwidth == 250000 ? "selected" : "",
           LORAConfig.signalBandwidth == 500000 ? "selected" : "",
           LORAConfig.codingRateDenominator == 5 ? "selected" : "",
           LORAConfig.codingRateDenominator == 6 ? "selected" : "",
           LORAConfig.codingRateDenominator == 7 ? "selected" : "",
           LORAConfig.codingRateDenominator == 8 ? "selected" : "",
           LORAConfig.preambleLength,
           LORAConfig.syncWord,
           LORAConfig.crc ? "checked" : "",
           LORAConfig.invertIQ ? "checked" : "",
           LORAConfig.onlyKnown ? "checked" : "");

  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}
# elif defined(ZgatewayRTL_433) || defined(ZgatewayPilight) || defined(ZgatewayRF) || defined(ZgatewayRF2) || defined(ZactuatorSomfy)
# include <map>
std::map<int, String> activeReceiverOptions = {
    {0, "Inactive"},
# if defined(ZgatewayPilight) && !defined(ZradioSX127x)
    {1, "PiLight"},
# endif
# if defined(ZgatewayRF) && !defined(ZradioSX127x)
    {2, "RF"},
# endif
# ifdef ZgatewayRTL_433
    {3, "RTL_433"},
# endif
# if defined(ZgatewayRF2) && !defined(ZradioSX127x)
    {4, "RF2 (restart required)"}
# endif
};

bool isValidReceiver(int receiverId) {

  return activeReceiverOptions.find(receiverId) != activeReceiverOptions.end();
}

String generateActiveReceiverOptions(int currentSelection) {
  String optionsHtml = "";
  for (const auto& option : activeReceiverOptions) {
    optionsHtml += "<option value='" + String(option.first) + "'";
    if (currentSelection == option.first) {
      optionsHtml += " selected";
    }
    optionsHtml += ">" + option.second + "</option>";
  }
  return optionsHtml;
}
# 1115 "C:/Users/Matt Jayne/Desktop/VS Code Project files/OpenMQTTMulti/OpenMQTTGatewayMulti/main/ZwebUI.ino"
void handleRF() {
  WEBUI_TRACE_LOG(F("handleRF: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  bool update = false;
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject WEBtoRF = jsonBuffer.to<JsonObject>();

  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleRF Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("save")) {
      if (server.hasArg("rf")) {
        String freqStr = server.arg("rf");
        RFConfig.frequency = freqStr.toFloat();
        if (validFrequency(RFConfig.frequency)) {
          WEBtoRF["frequency"] = RFConfig.frequency;
          update = true;
        } else {
          Log.warning(F("[WebUI] Invalid Frequency" CR));
        }
      }
      if (server.hasArg("ar")) {
        int selectedReceiver = server.arg("ar").toInt();
        if (isValidReceiver(selectedReceiver)) {
          RFConfig.activeReceiver = selectedReceiver;
          WEBtoRF["activereceiver"] = RFConfig.activeReceiver;
          update = true;
        } else {
          Log.warning(F("[WebUI] Invalid Active Receiver" CR));
        }
      }
      if (server.hasArg("oo")) {
        RFConfig.newOokThreshold = server.arg("oo").toInt();
        WEBtoRF["ookthreshold"] = RFConfig.newOokThreshold;
        update = true;
      }
      if (server.hasArg("rs")) {
        RFConfig.rssiThreshold = server.arg("rs").toInt();
        WEBtoRF["rssithreshold"] = RFConfig.rssiThreshold;
        update = true;
      }
      if (update) {
        Log.notice(F("[WebUI] Save data" CR));
        WEBtoRF["save"] = true;
        RFConfig_fromJson(WEBtoRF);
        stateRFMeasures();
        Log.trace(F("[WebUI] RFConfig end" CR));
      }
    }
  }

  String activeReceiverHtml = generateActiveReceiverOptions(RFConfig.activeReceiver);

  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);
  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure RF").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_rf_body, jsonChar, gateway_name, RFConfig.frequency, activeReceiverHtml.c_str());
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}
# endif





void handleRT() {
  WEBUI_TRACE_LOG(F("handleRT: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleRT Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
  }
  if (server.hasArg("non")) {
    char jsonChar[100];
    serializeJson(modules, jsonChar, measureJson(modules) + 1);
    Log.warning(F("[WebUI] Erase and Restart" CR));

    char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Erase and Restart").c_str());
    String response = String(buffer);
    response += String(restart_script);
    response += String(script);
    response += String(style);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, reset_body, jsonChar, gateway_name, "Erase and Restart");
    response += String(buffer);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
    response += String(buffer);
    server.send(200, "text/html", response);

    eraseAndRestart();
  } else {
    handleCN();
  }
}

# if defined(ZgatewayCloud)




void handleCL() {
  WEBUI_TRACE_LOG(F("handleCL: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleCL Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
  }

  if (server.hasArg("save")) {



    if (server.hasArg("save") && server.method() == 1) {
      setCloudEnabled(server.hasArg("cl-en"));
    }
  }

  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Configure Cloud").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);

  char cloudEnabled[8] = {0};
  if (isCloudEnabled()) {
    strncpy(cloudEnabled, "checked", 8);
  }

  char deviceToken[5] = {0};
  if (!isCloudDeviceTokenSupplied()) {
    strncpy(deviceToken, " Not", 4);
  }

  requestToken = esp_random();
# ifdef ESP32_ETHERNET
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_cloud_body, jsonChar, gateway_name, " cloud checked", " Not", (String(CLOUDGATEWAY) + "token/start").c_str(), (char*)ETH.macAddress().c_str(), ("http://" + String(ip2CharArray(ETH.localIP())) + "/").c_str(), gateway_name, uptime(), requestToken);
# else
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, config_cloud_body, jsonChar, gateway_name, cloudEnabled, deviceToken, (String(CLOUDGATEWAY) + "token/start").c_str(), (char*)WiFi.macAddress().c_str(), ("http://" + String(ip2CharArray(WiFi.localIP())) + "/").c_str(), gateway_name, uptime(), requestToken);
# endif
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}





void handleTK() {
  WEBUI_TRACE_LOG(F("handleTK: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleTK Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
  }

  if (server.hasArg("deviceToken") && server.hasArg("uptime") && server.hasArg("RT")) {
    String deviceToken = server.arg("deviceToken");

    if (setCloudDeviceToken(deviceToken) && server.arg("RT").toInt() == requestToken && server.arg("uptime").toInt() + 600 > uptime()) {
      setCloudEnabled(true);
      char jsonChar[100];
      serializeJson(modules, jsonChar, measureJson(modules) + 1);

      char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Received Device Token").c_str());
      String response = String(buffer);
      response += String(script);
      response += String(style);
      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, token_body, jsonChar, gateway_name);
      response += String(buffer);
      snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
      response += String(buffer);
      server.send(200, "text/html", response);
    } else {
      WEBUI_TRACE_LOG(F("handleTK: uptime: %u, uptime: %u, ok: %T" CR), server.arg("uptime").toInt(), uptime(), server.arg("uptime").toInt() + 600 > uptime());
      WEBUI_TRACE_LOG(F("handleTK: RT: %d, RT: %d, ok: %T " CR), server.arg("RT").toInt(), requestToken, server.arg("RT").toInt() == requestToken);
      Log.error(F("[WebUI] Invalid Token Response: RT: %T, uptime: %T" CR), server.arg("RT").toInt() == requestToken, server.arg("uptime").toInt() + 600 > uptime());
      server.send(500, "text/html", "Internal ERROR - Invalid Token");
    }
  }
}

# endif





void handleIN() {
  WEBUI_TRACE_LOG(F("handleCN: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleIN Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
  } else {
    char jsonChar[100];
    serializeJson(modules, jsonChar, measureJson(modules) + 1);

    String informationDisplay = stateMeasures();


# if defined(ZgatewayBT)
    informationDisplay += "1<BR>BT}2}1";
    informationDisplay += stateBTMeasures(false);
# endif
# if defined(ZdisplaySSD1306)
    informationDisplay += "1<BR>SSD1306}2}1";
    informationDisplay += stateSSD1306Display();
# endif
# if defined(ZgatewayCloud)
    informationDisplay += "1<BR>Cloud}2}1";
    informationDisplay += stateCLOUDStatus();
# endif
# if defined(ZgatewayLORA)
    informationDisplay += "1<BR>LORA}2}1";
    informationDisplay += stateLORAMeasures();
# endif
# if defined(ZgatewayRF)
    informationDisplay += "1<BR>RF}2}1";
    informationDisplay += stateRFMeasures();
# endif
    informationDisplay += "1<BR>WebUI}2}1";
    informationDisplay += stateWebUIStatus();






    informationDisplay += "1}2";
    informationDisplay.replace(",\"", "}1");
    informationDisplay.replace("\":", "}2");
    informationDisplay.replace("{\"", "");
    informationDisplay.replace("\"", "\\\"");



    if (informationDisplay.length() > WEB_TEMPLATE_BUFFER_MAX_SIZE) {
      Log.warning(F("[WebUI] informationDisplay content length ( %d ) greater than WEB_TEMPLATE_BUFFER_MAX_SIZE.  Display truncated" CR), informationDisplay.length());
    }

    char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Information").c_str());
    String response = String(buffer);

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, information_script, informationDisplay.c_str());
    response += String(buffer);

    response += String(script);
    response += String(style);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, information_body, jsonChar, gateway_name);
    response += String(buffer);

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
    response += String(buffer);

    server.send(200, "text/html", response);
  }
}





void handleFavicon() {
  WEBUI_TRACE_LOG(F("handleCN: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  server.sendHeader("Content-Type", "image/x-icon");
  server.send_P(200, "image/x-icon", reinterpret_cast<const char*>(Openmqttgateway_logo_mini_ico), sizeof(Openmqttgateway_logo_mini_ico));
}

# if defined(ESP32) && defined(MQTT_HTTPS_FW_UPDATE)




void handleUP() {
  WEBUI_TRACE_LOG(F("handleUP: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args()) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleUP Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    DynamicJsonDocument jsonBuffer(JSON_MSG_BUFFER);
    JsonObject WEBtoSYS = jsonBuffer.to<JsonObject>();

    if (server.hasArg("o")) {
      WEBtoSYS["url"] = server.arg("o");
      WEBtoSYS["version"] = "test";
      WEBtoSYS["password"] = ota_pass;

      {
        sendRestartPage();

        String output;
        serializeJson(WEBtoSYS, output);
        Log.notice(F("[WebUI] MQTTtoSYSupdate %s" CR), output.c_str());
      }

      String topic = String(mqtt_topic) + String(gateway_name) + String(subjectMQTTtoSYSupdate);
      MQTTHttpsFWUpdate((char*)topic.c_str(), WEBtoSYS);
      return;
    } else if (server.hasArg("le")) {
      uint32_t le = server.arg("le").toInt();
      if (le != 0) {
        WEBtoSYS["version"] = (le == 1 ? "latest" : (le == 2 ? "dev" : "unknown"));
        WEBtoSYS["password"] = ota_pass;
        {
          sendRestartPage();

          String output;
          serializeJson(WEBtoSYS, output);
          Log.notice(F("[WebUI] MQTTtoSYSupdate %s" CR), output.c_str());
        }

        String topic = String(mqtt_topic) + String(gateway_name) + String(subjectMQTTtoSYSupdate);
        MQTTHttpsFWUpdate((char*)topic.c_str(), WEBtoSYS);
        return;
      }
    }
  }
  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);

  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Firmware Upgrade").c_str());
  String response = String(buffer);
  response += String(script);
  response += String(style);
  String systemUrl = RELEASE_LINK + latestVersion + "/" + ENV_NAME + "-firmware.bin";
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, upgrade_body, jsonChar, gateway_name, systemUrl.c_str());
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);
}
# endif

void sendRestartPage() {
  char jsonChar[100];
  serializeJson(modules, jsonChar, measureJson(modules) + 1);
  char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Updating Firmware and Restart").c_str());
  String response = String(buffer);
  response += String(restart_script);
  response += String(script);
  response += String(style);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, reset_body, jsonChar, gateway_name, "Updating Firmware and Restart");
  response += String(buffer);
  snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
  response += String(buffer);
  server.send(200, "text/html", response);

  delay(2000);
}





void handleCS() {
  WEBUI_TRACE_LOG(F("handleCS: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
  WEBUI_SECURE
  if (server.args() && server.hasArg("c2")) {
    for (uint8_t i = 0; i < server.args(); i++) {
      WEBUI_TRACE_LOG(F("handleCS Arg: %d, %s=%s" CR), i, server.argName(i).c_str(), server.arg(i).c_str());
    }
    if (server.hasArg("c1")) {
      String c1 = server.arg("c1");

      String cmdTopic = String(mqtt_topic) + String(gateway_name) + "/" + c1.substring(0, c1.indexOf(' '));
      String command = c1.substring(c1.indexOf(' ') + 1);
      if (command.length()) {
        WEBUI_TRACE_LOG(F("[WebUI] handleCS inject MQTT Command topic: '%s', command: '%s'" CR), cmdTopic.c_str(), command.c_str());
        receivingMQTT((char*)cmdTopic.c_str(), (char*)command.c_str());
      } else {
        Log.warning(F("[WebUI] Missing command: '%s', command: '%s'" CR), cmdTopic.c_str(), command.c_str());
      }
    }

    uint32_t index = server.arg("c2").toInt();

    String message = String(log_buffer_pointer) + "}1" + String(reset_web_log_flag) + "}1";
    if (!reset_web_log_flag) {
      index = 0;
      reset_web_log_flag = true;
    }

    bool cflg = (index);
    char* line;
    size_t len;
    while (GetLog(1, &index, &line, &len)) {
      if (cflg) {
        message += "\n";
      }
      for (int x = 0; x < len - 1; x++) {
        message += line[x];
      }
      cflg = true;
    }
    message += "}1";
    server.send(200, "text/plain", message);
  } else {
    char jsonChar[100];
    serializeJson(modules, jsonChar, measureJson(modules) + 1);

    char buffer[WEB_TEMPLATE_BUFFER_MAX_SIZE];

    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, header_html, (String(gateway_name) + " - Console").c_str());
    String response = String(buffer);
    response += String(console_script);
    response += String(script);
    response += String(style);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, console_body, jsonChar, gateway_name);
    response += String(buffer);
    snprintf(buffer, WEB_TEMPLATE_BUFFER_MAX_SIZE, footer, OMG_VERSION);
    response += String(buffer);
    server.send(200, "text/html", response);
  }
}





void notFound() {
  WEBUI_SECURE
# ifdef WEBUI_DEVELOPMENT
  String path = server.uri();
  if (!exists(path)) {
    if (exists(path + ".html")) {
      path += ".html";
    } else {
# endif
      Log.warning(F("[WebUI] notFound: uri: %s, args: %d, method: %d" CR), server.uri(), server.args(), server.method());
      server.send(404, "text/plain", "Not found");
      return;
# ifdef WEBUI_DEVELOPMENT
    }
  }
  WEBUI_TRACE_LOG(F("notFound returning: actual uri: %s, args: %d, method: %d" CR), path, server.args(), server.method());
  File file = FILESYSTEM.open(path, "r");
  server.streamFile(file, "text/html");
  file.close();
# endif
}

void WebUISetup() {
  WEBUI_TRACE_LOG(F("ZwebUI setup start" CR));

  WebUIConfig_load();
  webUIQueue = xQueueCreate(5, sizeof(webUIQueueMessage*));

# ifdef WEBUI_DEVELOPMENT
  FILESYSTEM.begin();
  {
    File root = FILESYSTEM.open("/");
    File file = root.openNextFile();
    while (file) {
      String fileName = file.name();
      size_t fileSize = file.size();
      WEBUI_TRACE_LOG(F("FS File: %s, size: %s" CR), fileName.c_str(), formatBytes(fileSize).c_str());
      file = root.openNextFile();
    }
  }
# endif
  server.onNotFound(notFound);

  server.on("/", handleRoot);

  server.on("/in", handleIN);
  server.on("/cs", handleCS);
# if defined(ESP32) && defined(MQTT_HTTPS_FW_UPDATE)
  server.on("/up", handleUP);
# endif
  server.on("/cn", handleCN);
  server.on("/wi", handleWI);
  server.on("/mq", handleMQ);
# ifndef ESPWifiManualSetup
  server.on("/cg", handleCG);
# endif
  server.on("/wu", handleWU);
# ifdef ZgatewayLORA
  server.on("/la", handleLA);
# elif defined(ZgatewayRTL_433) || defined(ZgatewayPilight) || defined(ZgatewayRF) || defined(ZgatewayRF2) || defined(ZactuatorSomfy)
  server.on("/rf", handleRF);
# endif
# if defined(ZgatewayCloud)
  server.on("/cl", handleCL);
  server.on("/tk", handleTK);
# endif
  server.on("/lo", handleLO);

  server.on("/rt", handleRT);
  server.on("/favicon.ico", handleFavicon);
  server.begin();

  Log.begin(LOG_LEVEL, &WebLog);

  Log.trace(F("[WebUI] displayMetric %T" CR), displayMetric);
  Log.trace(F("[WebUI] WebUI Secure %T" CR), webUISecure);
  Log.notice(F("OpenMQTTGateway URL: http://%s/" CR), WiFi.localIP().toString().c_str());
  displayPrint("URL: http://", (char*)WiFi.localIP().toString().c_str());
  Log.notice(F("ZwebUI setup done" CR));
}

unsigned long nextWebUIMessage = uptime() + DISPLAY_WEBUI_INTERVAL;

void WebUILoop() {
  server.handleClient();

  if (uptime() >= nextWebUIMessage && uxQueueMessagesWaiting(webUIQueue)) {
    webUIQueueMessage* message = nullptr;
    xQueueReceive(webUIQueue, &message, portMAX_DELAY);
    newSSD1306Message = true;

    if (currentWebUIMessage) {
      free(currentWebUIMessage);
    }
    currentWebUIMessage = message;
    nextWebUIMessage = uptime() + DISPLAY_WEBUI_INTERVAL;
  }
}

void MQTTtoWebUI(char* topicOri, JsonObject& WebUIdata) {
  bool success = false;
  if (cmpToMainTopic(topicOri, subjectMQTTtoWebUIset)) {
    WEBUI_TRACE_LOG(F("MQTTtoWebUI json set" CR));

    if (WebUIdata.containsKey("displayMetric")) {
      displayMetric = WebUIdata["displayMetric"].as<bool>();
      Log.notice(F("Set displayMetric: %T" CR), displayMetric);
      success = true;
    }

    if (WebUIdata.containsKey("save") && WebUIdata["save"]) {
      success = WebUIConfig_save();
      if (success) {
        Log.notice(F("WebUI config saved" CR));
      }
    } else if (WebUIdata.containsKey("load") && WebUIdata["load"]) {
      success = WebUIConfig_load();
      if (success) {
        Log.notice(F("WebUI config loaded" CR));
      }
    } else if (WebUIdata.containsKey("init") && WebUIdata["init"]) {
      WebUIConfig_init();
      success = true;
      if (success) {
        Log.notice(F("WebUI config initialised" CR));
      }
    } else if (WebUIdata.containsKey("erase") && WebUIdata["erase"]) {

      preferences.begin(Gateway_Short_Name, false);
      success = preferences.remove("WebUIConfig");
      preferences.end();
      if (success) {
        Log.notice(F("WebUI config erased" CR));
      }
    }
    if (success) {
      stateWebUIStatus();
    } else {
      Log.error(F("[ WebUI ] MQTTtoWebUI Fail json" CR), WebUIdata);
    }
  }
}

String stateWebUIStatus() {

  StaticJsonDocument<JSON_MSG_BUFFER> WebUIdataBuffer;
  JsonObject WebUIdata = WebUIdataBuffer.to<JsonObject>();
  WebUIdata["displayMetric"] = (bool)displayMetric;
  WebUIdata["webUISecure"] = (bool)webUISecure;
  WebUIdata["displayQueue"] = uxQueueMessagesWaiting(webUIQueue);

  String output;
  serializeJson(WebUIdata, output);


  WebUIdata["origin"] = subjectWebUItoMQTT;
  handleJsonEnqueue(WebUIdata);
  return output;
}

bool WebUIConfig_save() {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject jo = jsonBuffer.to<JsonObject>();
  jo["displayMetric"] = (bool)displayMetric;
  jo["webUISecure"] = (bool)webUISecure;

  String conf = "";
  serializeJson(jsonBuffer, conf);
  preferences.begin(Gateway_Short_Name, false);
  int result = preferences.putString("WebUIConfig", conf);
  preferences.end();
  Log.trace(F("[WebUI] WebUIConfig_save: %s, result: %d" CR), conf.c_str(), result);
  return true;
}

void WebUIConfig_init() {
  displayMetric = DISPLAY_METRIC;
  webUISecure = WEBUI_AUTH;
  Log.notice(F("WebUI config initialised" CR));
}

bool WebUIConfig_load() {
  StaticJsonDocument<JSON_MSG_BUFFER> jsonBuffer;
  preferences.begin(Gateway_Short_Name, true);
  if (preferences.isKey("WebUIConfig")) {
    auto error = deserializeJson(jsonBuffer, preferences.getString("WebUIConfig", "{}"));
    preferences.end();
    if (error) {
      Log.error(F("WebUI config deserialization failed: %s, buffer capacity: %u" CR), error.c_str(), jsonBuffer.capacity());
      return false;
    }
    if (jsonBuffer.isNull()) {
      Log.warning(F("WebUI config is null" CR));
      return false;
    }
    JsonObject jo = jsonBuffer.as<JsonObject>();
    displayMetric = jo["displayMetric"].as<bool>();
    webUISecure = jo["webUISecure"].as<bool>();
    return true;
  } else {
    preferences.end();
    Log.notice(F("No WebUI config to load" CR));
    return false;
  }
}




constexpr unsigned int webUIHash(const char* s, int off = 0) {
  return !s[off] ? 5381 : (webUIHash(s, off + 1) * 33) ^ s[off];
}




void webUIPubPrint(const char* topicori, JsonObject& data) {
  WEBUI_TRACE_LOG(F("[ webUIPubPrint ] pub %s " CR), topicori);
  if (webUIQueue) {
    webUIQueueMessage* message = (webUIQueueMessage*)heap_caps_calloc(1, sizeof(webUIQueueMessage), MALLOC_CAP_8BIT);
    if (message != NULL) {

      strlcpy(message->line1, "", WEBUI_TEXT_WIDTH);
      strlcpy(message->line2, "", WEBUI_TEXT_WIDTH);
      strlcpy(message->line3, "", WEBUI_TEXT_WIDTH);
      strlcpy(message->line4, "", WEBUI_TEXT_WIDTH);
      char* topic = strdup(topicori);
      strlcpy(message->title, strtok(topic, "/"), WEBUI_TEXT_WIDTH);
      free(topic);


      switch (webUIHash(message->title)) {
        case webUIHash("SYStoMQTT"): {


          if (data["version"]) {
            strlcpy(message->line1, data["version"], WEBUI_TEXT_WIDTH);
          } else {
            strlcpy(message->line1, "", WEBUI_TEXT_WIDTH);
          }



          String uptime = data["uptime"];
          String line = "uptime: " + uptime;
          line.toCharArray(message->line2, WEBUI_TEXT_WIDTH);



          String freemem = data["freemem"];
          line = "freemem: " + freemem;
          line.toCharArray(message->line3, WEBUI_TEXT_WIDTH);



          String ip = data["ip"];
          line = "ip: " + ip;
          line.toCharArray(message->line4, WEBUI_TEXT_WIDTH);



          if (xQueueSend(webUIQueue, (void*)&message, 0) != pdTRUE) {
            Log.warning(F("[ WebUI ] ERROR: webUIQueue full, discarding %s" CR), message->title);
            free(message);
          } else {

          }
          break;
        }

# ifdef ZgatewayRTL_433
        case webUIHash("RTL_433toMQTT"): {
          if (data["model"] && strncmp(data["model"], "status", 6)) {




            strlcpy(message->line1, data["model"], WEBUI_TEXT_WIDTH);



            String line2 = "";
            if (data["id"]) {
              String id = data["id"];
              line2 += "id: " + id + " ";
            }

            if (data["channel"]) {
              String channel = data["channel"];
              line2 += "channel: " + channel;
            }
            line2.toCharArray(message->line2, WEBUI_TEXT_WIDTH);


            String line3 = "";

            if (data.containsKey("temperature_C")) {
              float temperature_C = data["temperature_C"];
              char temp[5];

              if (displayMetric) {
                dtostrf(temperature_C, 3, 1, temp);
                line3 = "temp: " + (String)temp + "°C ";
              } else {
                dtostrf(convertTemp_CtoF(temperature_C), 3, 1, temp);
                line3 = "temp: " + (String)temp + "°F ";
              }
            }

            float humidity = data["humidity"];
            if (data.containsKey("humidity") && humidity <= 100 && humidity >= 0) {
              char hum[5];
              dtostrf(humidity, 3, 1, hum);
              line3 += "hum: " + (String)hum + "% ";
            }
            if (data.containsKey("wind_avg_km_h")) {
              float wind_avg_km_h = data["wind_avg_km_h"];
              char wind[6];

              if (displayMetric) {
                dtostrf(wind_avg_km_h, 3, 1, wind);
                line3 += "wind: " + (String)wind + "km/h ";
              } else {
                dtostrf(convert_kmph2mph(wind_avg_km_h), 3, 1, wind);
                line3 += "wind: " + (String)wind + "mp/h ";
              }
            }

            float moisture = data["moisture"];
            if (data.containsKey("moisture") && moisture <= 100 && moisture >= 0) {
              char moist[5];
              dtostrf(moisture, 3, 1, moist);
              line3 += "moist: " + (String)moist + "% ";
            }

            line3.toCharArray(message->line3, WEBUI_TEXT_WIDTH);



            String line4 = "";
            if (data["battery_ok"]) {
              line4 = "batt: " + data["battery_ok"].as<String>();
            } else {
              line4 = "pulses: " + data["pulses"].as<String>();
            }

            line4 += " rssi: " + data["rssi"].as<String>();
            line4.toCharArray(message->line4, WEBUI_TEXT_WIDTH);



            if (xQueueSend(webUIQueue, (void*)&message, 0) != pdTRUE) {
              Log.error(F("[ WebUI ] webUIQueue full, discarding signal %s" CR), message->title);
              free(message);
            } else {

            }
          } else {
            Log.error(F("[ WebUI ] rtl_433 not displaying %s" CR), message->title);
            free(message);
          }
          break;
        }
# endif
# ifdef ZsensorBME280
        case webUIHash("CLIMAtoMQTT"): {




          strlcpy(message->line1, "bme280", WEBUI_TEXT_WIDTH);



          String line2 = "";
          if (data.containsKey("tempc")) {
            char temp[5];
            float temperature_C = data["tempc"];

            if (displayMetric) {
              dtostrf(temperature_C, 3, 1, temp);
              line2 = "temp: " + (String)temp + "°C ";
            } else {
              dtostrf(convertTemp_CtoF(temperature_C), 3, 1, temp);
              line2 = "temp: " + (String)temp + "°F ";
            }
          }
          line2.toCharArray(message->line2, WEBUI_TEXT_WIDTH);



          String line3 = "";
          float humidity = data["hum"];
          if (data.containsKey("hum") && humidity <= 100 && humidity >= 0) {
            char hum[5];
            dtostrf(humidity, 3, 1, hum);
            line3 += "hum: " + (String)hum + "% ";
          }
          line3.toCharArray(message->line3, WEBUI_TEXT_WIDTH);



          float pa = (int)data["pa"] / 100;
          char pressure[6];

          String line4 = "";
          if (displayMetric) {
            dtostrf(pa, 3, 1, pressure);
            line4 = "pressure: " + (String)pressure + " hPa";
          } else {
            dtostrf(convert_hpa2inhg(pa), 3, 1, pressure);
            line4 = "pressure: " + (String)pressure + " inHg";
          }
          line4.toCharArray(message->line4, WEBUI_TEXT_WIDTH);



          if (xQueueSend(webUIQueue, (void*)&message, 0) != pdTRUE) {
            Log.error(F("[ WebUI ] webUIQueue full, discarding signal %s" CR), message->title);
            free(message);
          } else {

          }
          break;
        }
# endif
# ifdef ZgatewayBT
        case webUIHash("BTtoMQTT"): {


          if (data["model_id"] != "MS-CDP" && data["model_id"] != "GAEN" && data["model_id"] != "APPLE_CONT" && data["model_id"] != "IBEACON") {

            String line2 = "";
            String line3 = "";
            String line4 = "";


            String properties[6] = {"", "", "", "", "", ""};
            int property = -1;

            if (data["type"] == "THB" || data["type"] == "THBX" || data["type"] == "PLANT" || data["type"] == "AIR" || data["type"] == "BATT" || data["type"] == "ACEL" || (data["type"] == "UNIQ" && data["model_id"] == "SDLS")) {
              if (data.containsKey("tempc")) {
                property++;
                char temp[5];
                if (displayMetric) {
                  float temperature = data["tempc"];
                  dtostrf(temperature, 3, 1, temp);
                  properties[property] = "temp: " + (String)temp + "°C ";
                } else {
                  float temperature = data["tempf"];
                  dtostrf(temperature, 3, 1, temp);
                  properties[property] = "temp: " + (String)temp + "°F ";
                }
              }

              if (data.containsKey("tempc2_dp")) {
                property++;
                char tempdp[5];
                if (displayMetric) {
                  float temperature = data["tempc2_dp"];
                  dtostrf(temperature, 3, 1, tempdp);
                  properties[property] = "dewp: " + (String)tempdp + "°C ";
                } else {
                  float temperature = data["tempf2_dp"];
                  dtostrf(temperature, 3, 1, tempdp);
                  properties[property] = "dewp: " + (String)tempdp + "°F ";
                }
              }

              if (data.containsKey("extprobe")) {
                property++;
                properties[property] = " ext. probe";
              }

              if (data.containsKey("hum")) {
                property++;
                float humidity = data["hum"];
                char hum[5];

                dtostrf(humidity, 3, 1, hum);
                properties[property] = "hum: " + (String)hum + "% ";
              }

              if (data.containsKey("pm25")) {
                property++;
                int pm25int = data["pm25"];
                char pm25[3];
                itoa(pm25int, pm25, 10);
                if ((data.containsKey("pm10"))) {
                  properties[property] = "PM 2.5: " + (String)pm25 + " ";

                } else {
                  properties[property] = "pm2.5: " + (String)pm25 + "μg/m³ ";
                }
              }

              if (data.containsKey("pm10")) {
                property++;
                int pm10int = data["pm10"];
                char pm10[3];
                itoa(pm10int, pm10, 10);
                if ((data.containsKey("pm25"))) {
                  properties[property] = "/ 10: " + (String)pm10 + "μg/m³ ";

                } else {
                  properties[property] = "pm10: " + (String)pm10 + "μg/m³ ";
                }
              }

              if (data.containsKey("for")) {
                property++;
                int formint = data["for"];
                char form[3];
                itoa(formint, form, 10);
                properties[property] = "CH₂O: " + (String)form + "mg/m³ ";
              }

              if (data.containsKey("co2")) {
                property++;
                int co2int = data["co2"];
                char co2[4];
                itoa(co2int, co2, 10);
                properties[property] = "co2: " + (String)co2 + "ppm ";
              }

              if (data.containsKey("moi")) {
                property++;
                int moiint = data["moi"];
                char moi[4];
                itoa(moiint, moi, 10);
                properties[property] = "moi: " + (String)moi + "% ";
              }

              if (data.containsKey("lux")) {
                property++;
                int luxint = data["lux"];
                char lux[5];
                itoa(luxint, lux, 10);
                properties[property] = "lux: " + (String)lux + "lx ";
              }

              if (data.containsKey("fer")) {
                property++;
                int ferint = data["fer"];
                char fer[7];
                itoa(ferint, fer, 10);
                properties[property] = "fer: " + (String)fer + "µS/cm ";
              }

              if (data.containsKey("pres")) {
                property++;
                int presint = data["pres"];
                char pres[4];
                itoa(presint, pres, 10);
                properties[property] = "pres: " + (String)pres + "hPa ";
              }

              if (data.containsKey("batt")) {
                property++;
                int battery = data["batt"];
                char batt[5];
                itoa(battery, batt, 10);
                properties[property] = "batt: " + (String)batt + "% ";
              }

              if (data.containsKey("shake")) {
                property++;
                int shakeint = data["shake"];
                char shake[3];
                itoa(shakeint, shake, 10);
                properties[property] = "shake: " + (String)shake + " ";
              }

              if (data.containsKey("volt")) {
                property++;
                float voltf = data["volt"];
                char volt[5];
                dtostrf(voltf, 3, 1, volt);
                properties[property] = "volt: " + (String)volt + "V ";
              }

              if (data.containsKey("wake")) {
                property++;
                String wakestr = data["wake"];
                properties[property] = "wake: " + wakestr + " ";
              }

            } else if (data["type"] == "BBQ") {
              String tempcstr = "";
              int j = 7;
              if (data["model_id"] == "IBT-2X(S)") {
                j = 3;
              } else if (data["model_id"] == "IBT-4X(S/C)") {
                j = 5;
              }

              for (int i = 0; i < j; i++) {
                if (i == 0) {
                  if (displayMetric) {
                    tempcstr = "tempc";
                  } else {
                    tempcstr = "tempf";
                  }
                  i++;
                } else {
                  if (displayMetric) {
                    tempcstr = "tempc" + (String)i;
                  } else {
                    tempcstr = "tempf" + (String)i;
                  }
                }

                if (data.containsKey(tempcstr)) {
                  char temp[5];
                  float temperature = data[tempcstr];
                  dtostrf(temperature, 3, 1, temp);
                  properties[i - 1] = "tp" + (String)i + ": " + (String)temp;
                  if (displayMetric) {
                    properties[i - 1] += "°C ";
                  } else {
                    properties[i - 1] += "°F ";
                  }
                } else {
                  properties[i - 1] = "tp" + (String)i + ": " + "off ";
                }
              }
            } else if (data["type"] == "BODY") {
              if (data.containsKey("steps")) {
                property++;
                int stepsint = data["steps"];
                char steps[5];
                itoa(stepsint, steps, 10);
                properties[property] = "steps: " + (String)steps + " ";

                property++;
              }

              if (data.containsKey("act_bpm")) {
                property++;
                int actbpmint = data["act_bpm"];
                char actbpm[3];
                itoa(actbpmint, actbpm, 10);
                properties[property] = "activity bpm: " + (String)actbpm + " ";
              }

              if (data.containsKey("bpm")) {
                property++;
                int bpmint = data["bpm"];
                char bpm[3];
                itoa(bpmint, bpm, 10);
                properties[property] = "bpm: " + (String)bpm + " ";
              }
            } else if (data["type"] == "SCALE") {
              if (data.containsKey("weighing_mode")) {
                property++;
                String mode = data["weighing_mode"];
                properties[property] = mode + " ";

                property++;
              }

              if (data.containsKey("weight")) {
                property++;
                float weightf = data["weight"];
                char weight[7];
                dtostrf(weightf, 3, 1, weight);
                if (data.containsKey("unit")) {
                  String unit = data["unit"];
                  properties[property] = "weight: " + (String)weight + unit + " ";
                } else {
                  properties[property] = "weight: " + (String)weight;
                }

                property++;
              }

              if (data.containsKey("impedance")) {
                property++;
                int impint = data["impedance"];
                char imp[3];
                itoa(impint, imp, 10);
                properties[property] = "impedance: " + (String)imp + "ohm ";
              }
            } else if (data["type"] == "UNIQ") {
              if (data["model_id"] == "M1017") {
                if (data.containsKey("lvl_cm")) {
                  property++;
                  char lvl[5];
                  if (displayMetric) {
                    float lvlf = data["lvl_cm"];
                    dtostrf(lvlf, 3, 1, lvl);
                    properties[property] = "level: " + (String)lvl + "cm ";
                  } else {
                    float lvlf = data["lvl_in"];
                    dtostrf(lvlf, 3, 1, lvl);
                    properties[property] = "level: " + (String)lvl + "\" ";
                  }
                }

                if (data.containsKey("quality")) {
                  property++;
                  int qualint = data["quality"];
                  char qual[3];
                  itoa(qualint, qual, 10);
                  properties[property] = "qy: " + (String)qual + " ";
                }

                if (data.containsKey("batt")) {
                  property++;
                  int battery = data["batt"];
                  char batt[5];
                  itoa(battery, batt, 10);
                  properties[property] = "batt: " + (String)batt + "% ";
                }
              }
            }

            line2 = properties[0] + properties[1];
            line3 = properties[2] + properties[3];
            line4 = properties[4] + properties[5];

            if (!(line2 == "" && line3 == "" && line4 == "")) {

              char* topic = strdup(topicori);
              String heading = strtok(topic, "/");
              String line0 = heading + "           " + data["id"].as<String>().substring(9, 17);
              line0.toCharArray(message->title, WEBUI_TEXT_WIDTH);
              free(topic);


              strlcpy(message->line1, data["model"], WEBUI_TEXT_WIDTH);

              line2.toCharArray(message->line2, WEBUI_TEXT_WIDTH);
              line3.toCharArray(message->line3, WEBUI_TEXT_WIDTH);
              line4.toCharArray(message->line4, WEBUI_TEXT_WIDTH);

              if (xQueueSend(webUIQueue, (void*)&message, 0) != pdTRUE) {
                Log.error(F("[ WebUI ] webUIQueue full, discarding signal %s" CR), message->title);
                free(message);
              } else {

              }
            } else {
              WEBUI_TRACE_LOG(F("[ WebUI ] incomplete messaage %s" CR), topicori);
              free(message);
            }

            break;
          } else {
            WEBUI_TRACE_LOG(F("[ WebUI ] incorrect model_id %s" CR), topicori);
            free(message);
            break;
          }
        }
# endif
# ifdef ZsensorRN8209
        case webUIHash("RN8209toMQTT"): {




          String line1 = "";
          if (data.containsKey("volt")) {
            char volt[5];
            float voltage = data["volt"];
            dtostrf(voltage, 3, 1, volt);
            line1 = "volt: " + (String)volt;
          }
          line1.toCharArray(message->line1, WEBUI_TEXT_WIDTH);



          String line2 = "";
          if (data.containsKey("current")) {
            char curr[5];
            float current = data["current"];
            dtostrf(current, 3, 1, curr);
            line2 = "current: " + (String)curr + " A";
          }
          line2.toCharArray(message->line2, WEBUI_TEXT_WIDTH);



          String line3 = "";
          if (data.containsKey("power")) {
            char pow[5];
            float power = data["power"];
            dtostrf(power, 3, 1, pow);
            line3 = "power: " + (String)pow + " W";
          }
          line3.toCharArray(message->line3, WEBUI_TEXT_WIDTH);



          if (xQueueSend(webUIQueue, (void*)&message, 0) != pdTRUE) {
            Log.error(F("[ WebUI ] webUIQueue full, discarding signal %s" CR), message->title);
            free(message);
          } else {

          }
          break;
        }
# endif
# ifdef ZgatewayLORA
        case webUIHash("LORAtoMQTT"): {


          String line1 = "";
          if (data.containsKey("tempc")) {
            char temp[5];
            float temperature_C = data["tempc"];

            if (displayMetric) {
              dtostrf(temperature_C, 3, 1, temp);
              line1 = "temp: " + (String)temp + "°C ";
            } else {
              dtostrf(convertTemp_CtoF(temperature_C), 3, 1, temp);
              line1 = "temp: " + (String)temp + "°F ";
            }
          }
          line1.toCharArray(message->line1, WEBUI_TEXT_WIDTH);



          String line2 = "";
          float humidity = data["hum"];
          if (data.containsKey("hum") && humidity <= 100 && humidity >= 0) {
            char hum[5];
            dtostrf(humidity, 3, 1, hum);
            line2 += "hum: " + (String)hum + "% ";
          }
          line2.toCharArray(message->line2, WEBUI_TEXT_WIDTH);



          String line3 = "";
          float adc = data["adc"];
          if (data.containsKey("adc") && adc <= 100 && adc >= 0) {
            char cAdc[5];
            dtostrf(adc, 3, 1, cAdc);
            line3 += "adc: " + (String)cAdc + "µS/cm ";
          }
          line3.toCharArray(message->line2, WEBUI_TEXT_WIDTH);



          if (xQueueSend(webUIQueue, (void*)&message, 0) != pdTRUE) {
            Log.error(F("[ WebUI ] webUIQueue full, discarding signal %s" CR), message->title);
            free(message);
          } else {

          }
          break;
        }
# endif
        default:
          Log.verbose(F("[ WebUI ] unhandled topic %s" CR), message->title);
          free(message);
      }
    } else {
      Log.error(F("[ WebUI ] insufficent memory " CR));
    }
  } else {
    Log.error(F("[ WebUI ] not initalized " CR));
  }
}





SerialWeb WebLog(0);
SerialWeb::SerialWeb(int x) {
}




void SerialWeb::begin() {

}




int SerialWeb::available(void) {
}




int SerialWeb::peek(void) {
}




int SerialWeb::read(void) {
}




void SerialWeb::flush(void) {
}




size_t SerialWeb::write(const uint8_t* buffer, size_t size) {

  addLog(buffer, size);
  return Serial.write(buffer, size);
}

char line[ROW_LENGTH];
int lineIndex = 0;
void addLog(const uint8_t* buffer, size_t size) {
  for (int i = 0; i < size; i++) {
    if (char(buffer[i]) == 10 | lineIndex > ROW_LENGTH - 2) {
      if (char(buffer[i]) != 10) {
        line[lineIndex++] = char(buffer[i]);
      }
      line[lineIndex++] = char(0);
      AddLogData(1, (const char*)&line[0]);
      lineIndex = 0;
    } else {
      line[lineIndex++] = char(buffer[i]);
    }
  }
}

#endif