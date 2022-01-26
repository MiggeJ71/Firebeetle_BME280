// ToDo:
// - Move WiFi cridentials to separate .h file - done
// - Only read and report battery once every hour
// - Remove or reduce 2000ms delay at startup
// - put BME in sleep mode?
// - Send less data over MQTT (only needed data)
// - test power consumption with debug OFF

// Log all to Serial, comment this line to disable logging
#define LOG Serial
// Include must be placed after LOG definition to work
#include "log.h"

// WiFi access
#include "wifi_access.h"

#include "esp_adc_cal.h"
#include <DFRobot_BME280.h>
#include <Wire.h>
#include <WiFi.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <MQTT.h>

// ******** use abbreviations instead of full names ********
typedef DFRobot_BME280_IIC    BME;    

/**IIC address is 0x76 when pin SDO is low  (SIM7000)*/
BME   bme(&Wire, 0x76);   // select TwoWire peripheral and set sensor address

#define SEA_LEVEL_PRESSURE    1015.0f

// Define alarm levels
#define NO_ALARM 0
#define ALARM_LOW 1
#define ALARM_MEDIUM 2
#define ALARM_HIGH 3


// Define sleep time
#define INITIAL_SLEEP_TIME 5*1000000ULL  /* initail sleep time at start-up, 5s */
#define SLEEP_TIME 20*1000000ULL   /* sleep time between wake-ups, 900s (15min) */

// Define battery voltage levels
#define LOW_BATTERY_VOLTAGE 3.00
#define VERY_LOW_BATTERY_VOLTAGE 2.90
#define CRITICALLY_LOW_BATTERY_VOLTAGE 2.80

// MQTT Topics
#define MQTT_PUB_TEMP "esp32/bme280/IoT_3/temperature"
#define MQTT_PUB_HUM "esp32/bme280/IoT_3/humidity"
#define MQTT_PUB_PRES "esp32/bme280/IoT_3/pressure"
#define MQTT_PUB_BATT "esp32/bme280/IoT_3/battery"
#define MQTT_PUB_RESTARTS "esp32/bme280/IoT_3/restarts"
#define MQTT_PUB_ALARM "esp32/bme280/IoT_3/alarm"

String MQTTDeviceName = "Firebeetle";
String MQTTServerName = "192.168.1.50";
uint16_t MQTTPort = 1883;


// Define parameters to store in RTC memory to survive deep sleep
RTC_NOINIT_ATTR struct {
  uint8_t bssid[6];
  uint8_t channel;
  float BatteryVoltage;      //battery voltage in V
  uint64_t NumberOfRestarts; //number of restarts
  uint8_t alarm;             //last alarm
} cache;


// show last sensor operate status
void printLastOperateStatus(BME::eStatus_t eStatus)
{
  switch(eStatus) {
  case BME::eStatusOK:
    log_println("BME280 everything ok");
    break;
  case BME::eStatusErr:
    log_println("BME280 unknow error");
    break;
  case BME::eStatusErrDeviceNotDetected:
    log_println("BME280 device not detected");
    break;
  case BME::eStatusErrParameter:
    log_println("BME280 parameter error");
    break;
  default:
    log_println("BME280 unknow status");
    break;
  }
}

/******************************************************************************
Description.: reads the battery voltage through the voltage divider at GPIO36 (A0)
              if the ESP32-E has calibration eFused those will be used.
              In comparison with a regular voltmeter the values of ESP32 and
              multimeter differ only about 0.05V
Input Value.: -
Return Value: battery voltage in volts
******************************************************************************/
float readBattery() {
  uint32_t value = 0;
  int rounds = 11;
  esp_adc_cal_characteristics_t adc_chars;

  //battery voltage divided by 2 can be measured at GPIO34, which equals ADC1_CHANNEL6
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  switch(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars)) {
    case ESP_ADC_CAL_VAL_EFUSE_TP:
      log_println("Characterized using Two Point Value");
      break;
    case ESP_ADC_CAL_VAL_EFUSE_VREF:
      log_printf("Characterized using eFuse Vref (%d mV)\r\n", adc_chars.vref);
      break;
    default:
      log_printf("Characterized using Default Vref (%d mV)\r\n", 1100);
  }

  //to avoid noise, sample the pin several times and average the result
  for(int i=1; i<=rounds; i++) {
    value += adc1_get_raw(ADC1_CHANNEL_0);
    log_printf("ADC value : %d\n", value);
  }
  value /= (uint32_t)rounds;
  
  //due to the voltage divider (1M+1M) values must be multiplied by 2
  //and convert mV to V
  return esp_adc_cal_raw_to_voltage(value, &adc_chars)*2/1000.0;
}


/******************************************************************************
Description.: brings the WiFi up
Input Value.: When "tryCachedValuesFirst" is true the function tries to use
              cached values before attempting a scan + association
Return Value: true if WiFi is up, false if it timed out
******************************************************************************/
bool WiFiUP(bool tryCachedValuesFirst) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  
  if(tryCachedValuesFirst && cache.channel > 0) {
    log_printf("Cached values as follows:\r\n");
    log_printf(" Channel....: %d\r\n", cache.channel);
    log_printf(" BSSID......: %x:%x:%x:%x:%x:%x\r\n", cache.bssid[0], \
                                                         cache.bssid[1], \
                                                         cache.bssid[2], \
                                                         cache.bssid[3], \
                                                         cache.bssid[4], \
                                                         cache.bssid[5]);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, cache.channel, cache.bssid);

    for (unsigned long i=millis(); millis()-i < 10000;) {
      delay(10);

      if (WiFi.status() == WL_CONNECTED) {
        log_printf("WiFi connected with cached values (%lu)\r\n", millis()-i);
        return true;
      } 
    }
  }

  cache.channel = 0;
  for (uint32_t i = 0; i < sizeof(cache.bssid); i++)
    cache.bssid[i] = 0;

  // try it with the slower process
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  for (unsigned long i=millis(); millis()-i < 60000;) {
    delay(10);
  
    if (WiFi.status() == WL_CONNECTED) {
      log_printf("WiFi connected (%lu)\r\n", millis()-i);
  
      uint8_t *bssid = WiFi.BSSID();
      for (uint32_t i = 0; i < sizeof(cache.bssid); i++)
        cache.bssid[i] = bssid[i];
      cache.channel = WiFi.channel();
    
      return true;
    }
  }

  log_printf("WiFi NOT connected\r\n");
  return false;
}


/******************************************************************************
Description.: Reads sensor values and sends all data as MQTT messages
Return Value: true if OK, false if errors occured
******************************************************************************/
bool ReadAndPushSensors() {
  char buf[256] = {0};
   
  log_printf("Reading sensors\n");

  //read RTC
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  log_printf("Start reading sensor values\n");
   // New BME280 sensor readings

  float   temp = bme.getTemperature();
  uint32_t    press = bme.getPressure()/100;
  //float   alti = bme.calAltitude(SEA_LEVEL_PRESSURE, press);
  float   humi = bme.getHumidity();

  log_printf("Temp = %f\n", temp);
  log_printf("Press = %d\n", press);
  log_printf("Humi = %f\n", humi);

  //establish connection to MQTT server
  WiFiClient net;
  MQTTClient MQTTClient;
  MQTTClient.setTimeout(5000);
  MQTTClient.begin(MQTTServerName.c_str(), MQTTPort, net);

  if( MQTTClient.connect(MQTTDeviceName.c_str())) { 
    MQTTClient.publish(MQTT_PUB_TEMP, String(temp).c_str(), false, 2);
    MQTTClient.publish(MQTT_PUB_HUM, String(humi).c_str(), false, 2);
    MQTTClient.publish(MQTT_PUB_PRES, String(press).c_str(), false, 2);
    MQTTClient.publish(MQTT_PUB_BATT, String(cache.BatteryVoltage, 3), false, 2);
    snprintf(buf, sizeof(buf)-1, "%llu", cache.NumberOfRestarts);
    MQTTClient.publish(MQTT_PUB_RESTARTS, buf, false, 2);
    MQTTClient.publish(MQTT_PUB_ALARM, String(cache.alarm).c_str(), false, 2);
    
    return true;
  }

  return false;
}


/******************************************************************************
Description.: since this is a battery sensor, everything happens in setup
              and when the tonguing' is done the device enters deep-sleep
Input Value.: -
Return Value: -
******************************************************************************/
void setup() {
  bool status = false;
  bool reportVoltage = false;
  esp_reset_reason_t resetReason = esp_reset_reason();
  int countFail = 0;
  cache.alarm = NO_ALARM;

  //visual feedback when we are active, turn on onboard LED
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  cache.NumberOfRestarts++;

  Serial.begin(115200);

  log_printf("===================================================\r\n");
  log_printf("FireBeetle active\r\n" \
                " Compiled at: " __DATE__ " - " __TIME__ "\r\n" \
                " ESP-IDF: %s\r\n", esp_get_idf_version());

  log_printf("ESP_RST_POWERON = %d\n", ESP_RST_POWERON);
  log_printf("ESP_RST_DEEPSLEEP = %d\n", ESP_RST_DEEPSLEEP);

  //read battery voltage
  //cache.BatteryVoltage = readBattery();  
  // To be used while connected with USB and no battery
  cache.BatteryVoltage = 3.4;  
  log_printf("Voltage: %4.3f V\r\n", cache.BatteryVoltage);
    
  //a reset is required to wakeup again from below CRITICALLY_LOW_BATTERY_VOLTAGE
  //this is to prevent damaging the empty battery by saving as much power as possible
  //if battery is below VER_LOW_BATTERY_VOLTAGE but still above CRITICALLY_LOW_BATTERY_VOLTAGE, 
  //stop doing the regular work
  //this way the battery can run low, put still wakeup without physical interaction
  if (cache.BatteryVoltage < VERY_LOW_BATTERY_VOLTAGE) {
    if (cache.BatteryVoltage < CRITICALLY_LOW_BATTERY_VOLTAGE) {
      log_println("Battery critically low, hibernating...");
      cache.alarm = ALARM_HIGH;
      //switch off everything that might consume power
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
      //esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
      //esp_sleep_pd_config(ESP_PD_DOMAIN_CPU, ESP_PD_OPTION_OFF);

      //disable all wakeup sources
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    } else {
      log_println("Battery very low, deep sleeping...");
      cache.alarm = ALARM_MEDIUM;
      //sleep ~60 minutes if battery is between CRITICALLY_LOW_BATTERY_VOLTAGE to VERY_LOW_BATTERY_VOLTAGE
      uint64_t sleeptime = 60*60*1000000ULL;
      esp_sleep_enable_timer_wakeup(sleeptime);
    }

    digitalWrite(2, LOW);
    esp_deep_sleep_start();

    log_println("This should never get printed");
    return;
  }

   if (cache.BatteryVoltage < LOW_BATTERY_VOLTAGE)
     cache.alarm = ALARM_LOW;

  log_printf("Reset value = %d\n", resetReason);
  //check if a reset/power-on occured
  if (resetReason == ESP_RST_POWERON || resetReason == ESP_RST_WDT) {
      cache.NumberOfRestarts = 0;

      //set RTC to 0
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 0;
      settimeofday(&tv, NULL);

    if (resetReason == ESP_RST_POWERON) {
      log_printf("ESP was just switched ON\r\n");
    } else {
      log_printf("ESP was just reset\r\n");
      log_println("bme read data test");
      
      while(bme.begin() != BME::eStatusOK) {
        log_println("bme begin faild");
        countFail++;
        printLastOperateStatus(bme.lastOperateStatus);
        delay(1000);
        if (countFail > 20)
          break;
      }

      bme.setConfigFilter(BME::eConfigFilter_off);        // set config filterto OFF as we do weather measurements
      bme.setConfigTStandby(BME::eConfigTStandby_1000);   // set standby time
      bme.setCtrlMeasSamplingTemp(BME::eSampling_X1);     // set temperature over sampling
      bme.setCtrlMeasSamplingPress(BME::eSampling_X1);    // set pressure over sampling
      bme.setCtrlHumiSampling(BME::eSampling_X1);         // set humidity over sampling
      bme.setCtrlMeasMode(BME::eCtrlMeasMode_forced);     // set control measurement mode to measure once and then return to sleep, needs to be repeated for each measurment
    }

    //default is to have WiFi off
    if (WiFi.getMode() != WIFI_OFF) {
      log_printf("WiFi wasn't off!\r\n");
      WiFi.persistent(true);
      WiFi.mode(WIFI_OFF);
    }

    //try to connect
    WiFiUP(false);
    WiFi.disconnect(true, true);

    esp_sleep_enable_timer_wakeup(INITIAL_SLEEP_TIME);
  }

  // check if ESP returns from deepsleep
  if (resetReason == ESP_RST_DEEPSLEEP) {
    switch(esp_sleep_get_wakeup_cause()) {
      case ESP_SLEEP_WAKEUP_TIMER:
        log_printf("ESP woke up due to timer\r\n");

        while(bme.begin() != BME::eStatusOK) {
          log_println("bme begin faild");
          countFail++;
          printLastOperateStatus(bme.lastOperateStatus);
          delay(1000);
          if (countFail > 20)
            break;
        }

        // set control measurement mode to measure once and then return to sleep, needs to be repeated for each measurment
        bme.setCtrlMeasMode(BME::eCtrlMeasMode_forced);
        
        WiFiUP(true);
        ReadAndPushSensors();
        WiFi.disconnect(true, true);
        
        esp_sleep_enable_timer_wakeup(SLEEP_TIME);
        break;
    
      default:
        log_printf("ESP woke due to an unknown reason\r\n");
    }
  }

  log_printf("=== entering deepsleep after %d ms ===\r\n", millis());
  digitalWrite(2, LOW);
  esp_deep_sleep_start();
  
  log_println("This should never get printed");  
}

void loop() {
  // loop not used as all code is running in setup()
  log_println("This should never get printed");

}
