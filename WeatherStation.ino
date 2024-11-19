/**
 * @note      Arduino Setting
 *            Tools ->
 *                  Boards Manager : "esp32 by Espressif Systems -> 2.0.17"
 *                  Board:"ESP32S3 Dev Module"
 *                  USB CDC On Boot:"Enable"
 *                  USB DFU On Boot:"Disable"
 *                  Flash Size : "16MB(128Mb)"
 *                  Flash Mode"QIO 80MHz
 *                  Partition Scheme:"16M Flash(3M APP/9.9MB FATFS)"
 *                  PSRAM:"OPI PSRAM"
 *                  Upload Mode:"UART0/Hardware CDC"
 *                  USB Mode:"Hardware CDC and JTAG"
 *  
 */
#include <Arduino.h>            // In-built
#include <esp_task_wdt.h>       // In-built
#include "freertos/FreeRTOS.h"  // In-built
#include "freertos/task.h"      // In-built
#include "epd_driver.h"         // https://github.com/Xinyuan-LilyGO/LilyGo-EPD47
#include "esp_adc_cal.h"        // In-built

#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson
#include <HTTPClient.h>         // In-built

#include "WiFi.h"               // In-built
#include <WiFiClientSecure.h>
#include <SPI.h>                // In-built
#include <time.h>               // In-built

#include "owm_credentials.h"
#include "forecast_record.h"
#include "lang.h"
#include "googleTaskFromHA.h"

#define SCREEN_WIDTH   EPD_WIDTH
#define SCREEN_HEIGHT  EPD_HEIGHT

//################  VERSION  ##################################################
String version = "3.0 / 4.7in";  // Programme version, see change log at end
//################ VARIABLES ##################################################

enum alignment {LEFT, RIGHT, CENTER};
#define White         0xFF
#define LightGrey     0xBB
#define Grey          0x88
#define DarkGrey      0x44
#define Black         0x00

#define autoscale_on  true
#define autoscale_off false
#define barchart_on   true
#define barchart_off  false

boolean LargeIcon   = true;
boolean SmallIcon   = false;
#define Large  20           // For icon drawing
#define Small  10           // For icon drawing
String  Time_str = "-- --";
String  Date_str = "-- --- ----";
int     wifi_signal, CurrentHour = 9, CurrentMin = 0, CurrentSec = 0, EventCnt = 0, vref = 1100;
//################ PROGRAM VARIABLES and OBJECTS ##########################################
#define max_readings 24 // Limited to 3-days here, but could go to 5-days = 40 as the data is issued  

Forecast_record_type  WxConditions[1];
Forecast_record_type  WxForecast[max_readings];

float pressure_readings[max_readings]    = {0};
float temperature_readings[max_readings] = {0};
float humidity_readings[max_readings]    = {0};
float rain_readings[max_readings]        = {0};
float snow_readings[max_readings]        = {0};
float agileTariff[48];

long SleepDuration   = 5; // Sleep time in minutes, aligned to the nearest minute boundary, so if 30 will always update at 00 or 30 past the hour
int  WakeupHour      = 7;  // Wakeup after 07:00 to save battery power
int  SleepHour       = 23; // Sleep  after 23:00 to save battery power
long StartTime       = 0;
long SleepTimer      = 0;
long Delta           = 5; // ESP32 rtc speed compensation, prevents display at xx:59:yy and then xx:00:yy (one minute later) to save power

//fonts
#include "opensans8b.h"
//#include "opensans10b.h"
#include "opensans12b.h"
#include "opensans18b.h"
//#include "opensans24b.h"
#include "chinese20.h"

#include "baseimg.h"

#include "digital7.h"
#include "inverted.h"
#include "agency20.h"
#include "agency25.h"

GFXfont  currentFont;
uint8_t *framebuffer;

void BeginSleep() {
  epd_poweroff_all();
  UpdateLocalTime();
  SleepTimer = (SleepDuration * 60 - ((CurrentMin % SleepDuration) * 60 + CurrentSec)) + Delta; //Some ESP32 have a RTC that is too fast to maintain accurate time, so add an offset
  esp_sleep_enable_timer_wakeup(SleepTimer * 1000000LL); // in Secs, 1000000LL converts to Secs as unit = 1uSec
  Serial.println("Awake for : " + String((millis() - StartTime) / 1000.0, 3) + "-secs");
  Serial.println("Entering " + String(SleepTimer) + " (secs) of sleep time");
  Serial.println("Starting deep-sleep period...");
  Serial.end();
  esp_deep_sleep_start();  // Sleep for e.g. 30 minutes
}

boolean SetupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov"); //(gmtOffset_sec, daylightOffset_sec, ntpServer)
  setenv("TZ", Timezone, 1);  //setenv()adds the "TZ" variable to the environment with a value TimeZone, only used if set to 1, 0 means no change
  tzset(); // Set the TZ environment variable
  delay(100);
  return UpdateLocalTime();
}

uint8_t StartWiFi() {
  Serial.println("\r\nConnecting to: " + String(ssid));
  IPAddress dns(8, 8, 8, 8); // Use Google DNS
  IPAddress ip(192,168,50,86);
  IPAddress gateway(192,168,50,1);
  IPAddress subnet(255,255,255,0);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA); // switch off AP
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.config(ip, gateway, subnet, dns);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("STA: Failed!\n");
    WiFi.disconnect(false);
    delay(500);
    WiFi.begin(ssid, password);
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifi_signal = WiFi.RSSI(); // Get Wifi Signal strength now, because the WiFi will be turned off to save power!
    Serial.println("WiFi connected at: " + WiFi.localIP().toString());
    Serial.println("WiFi mac address: " + WiFi.macAddress());
  }
  else Serial.println("WiFi connection *** FAILED ***");
  return WiFi.status();
}

void StopWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  //esp_wifi_disconnect();
  Serial.println("WiFi switched Off");
}

void InitialiseSystem() {
  StartTime = millis();
  Serial.begin(115200);
  //while (!Serial);
  Serial.println(String(__FILE__) + "\nStarting...");
  epd_init();
  framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
  if (!framebuffer) Serial.println("Memory alloc failed!");
  memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
}

void loop() {
  // Nothing to do here
}

void IRAM_ATTR isr(){
  detachInterrupt(digitalPinToInterrupt(21)); //because later used for wake up
  Serial.println("Going to sleep now");
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_21,1); //1 = High, 0 = Low connected to GPIO32
  esp_deep_sleep_start();
  
}

void setup() {
  InitialiseSystem();

  if (StartWiFi() == WL_CONNECTED && SetupTime() == true) {

    bool WakeUp = true;
    /*
    if (WakeupHour > SleepHour)
      WakeUp = (CurrentHour >= WakeupHour || CurrentHour <= SleepHour);
    else
      WakeUp = (CurrentHour >= WakeupHour && CurrentHour <= SleepHour);
      */
    if (WakeUp) {
      byte Attempts = 1;
      bool RxWeather  = false;
      bool RxForecast = false;
      bool RxWaterFloat = false;
      bool RxOctopusAgileRate = false;
      bool RxGoogleTaskFromHA = false;
      WiFiClient client;   // wifi client object
      while ((RxWeather == false || RxForecast == false || RxWaterFloat == false) && Attempts <= 2) { // Try up-to 2 time for Weather and Forecast data
        if (RxWeather  == false) RxWeather  = obtainWeatherData(client, "weather");
        if (RxForecast == false) RxForecast = obtainWeatherData(client, "forecast");
        if (RxWaterFloat == false) RxWaterFloat = obtainWaterFloatData(client, WFStation); //get water float rate from station
        if (RxOctopusAgileRate == false) RxOctopusAgileRate = obtainOctpusAgileRate(client, agileTariffLocation);
        if (RxGoogleTaskFromHA == false) RxGoogleTaskFromHA = getTodoItemsFromHA(client);
        Attempts++;
      }
      Serial.println("Received all weather data...");
      if (RxWeather && RxForecast ) { // Only if received both Weather or Forecast proceed
        StopWiFi();         // Reduces power consumption
        epd_poweron();      // Switch on EPD display
        epd_clear();        // Clear the screen
        DrawBaseImage();
        DisplayWeather();   // Display the weather data
        edp_update();       // Update the display to show the information
        epd_poweroff_all(); // Switch off all power to EPD
      }
    }
  }
  BeginSleep();
}

void Convert_Readings_to_Imperial() { // Only the first 3-hours are used
  WxConditions[0].Pressure = hPa_to_inHg(WxConditions[0].Pressure);
  WxForecast[0].Rainfall   = mm_to_inches(WxForecast[0].Rainfall);
  WxForecast[0].Snowfall   = mm_to_inches(WxForecast[0].Snowfall);
}

bool DecodeWaterFloat(WiFiClient& json) {
  Serial.print(F("\nCreating WF object...and "));
  DynamicJsonDocument doc1(64 * 1024);                      // allocate the JsonDocument
  DeserializationError error = deserializeJson(doc1, json); // Deserialize the JSON document
  if (error) {                                             // Test if parsing succeeds.
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return false;
  }
  JsonObject root = doc1.as<JsonObject>();
  WxConditions[0].WaterFloat = root["items"]["latestReading"]["value"]; Serial.println("WtrFlt: " + String(WxConditions[0].WaterFloat));
  return true;
}

bool DecodeWeather(WiFiClient& json, String Type) {
  Serial.print(F("\nCreating object...and "));
  DynamicJsonDocument doc(64 * 1024);                      // allocate the JsonDocument
  DeserializationError error = deserializeJson(doc, json); // Deserialize the JSON document
  if (error) {                                             // Test if parsing succeeds.
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return false;
  }
  // convert it to a JsonObject
  JsonObject root = doc.as<JsonObject>();
  Serial.println(" Decoding " + Type + " data");
  if (Type == "weather") {
    // All Serial.println statements are for diagnostic purposes and some are not required, remove if not needed with //
    WxConditions[0].High        = -50; // Minimum forecast low
    WxConditions[0].Low         = 50;  // Maximum Forecast High
    WxConditions[0].FTimezone   = doc["timezone_offset"]; // "0"
    //JsonObject current = doc["current"];
    WxConditions[0].Sunrise     = doc["sys"]["sunrise"];                              Serial.println("SRis: " + String(WxConditions[0].Sunrise));
    WxConditions[0].Sunset      = doc["sys"]["sunset"];                               Serial.println("SSet: " + String(WxConditions[0].Sunset));
    WxConditions[0].Temperature = doc["main"]["temp"];                                 Serial.println("Temp: " + String(WxConditions[0].Temperature));
    WxConditions[0].FeelsLike   = doc["main"]["feels_like"];                           Serial.println("FLik: " + String(WxConditions[0].FeelsLike));
    WxConditions[0].Pressure    = doc["main"]["pressure"];                             Serial.println("Pres: " + String(WxConditions[0].Pressure));
    WxConditions[0].Humidity    = doc["main"]["humidity"];                             Serial.println("Humi: " + String(WxConditions[0].Humidity));
    WxConditions[0].DewPoint    = doc["dew_point"];                            Serial.println("DPoi: " + String(WxConditions[0].DewPoint));
    WxConditions[0].UVI         = doc["uvi"];                                  Serial.println("UVin: " + String(WxConditions[0].UVI));
    WxConditions[0].Cloudcover  = doc["clouds"]["all"];                               Serial.println("CCov: " + String(WxConditions[0].Cloudcover));
    WxConditions[0].Visibility  = doc["visibility"];                           Serial.println("Visi: " + String(WxConditions[0].Visibility));
    WxConditions[0].Windspeed   = doc["wind"]["speed"];                           Serial.println("WSpd: " + String(WxConditions[0].Windspeed));
    WxConditions[0].Winddir     = doc["wind"]["deg"];                             Serial.println("WDir: " + String(WxConditions[0].Winddir));
    JsonObject current_weather  = doc["weather"][0];
    String Description = current_weather["description"];                           // "scattered clouds"
    String Icon        = current_weather["icon"];                                  // "01n"
    WxConditions[0].Forecast0   = Description;                                     Serial.println("Fore: " + String(WxConditions[0].Forecast0));
    WxConditions[0].Icon        = Icon;                                            Serial.println("Icon: " + String(WxConditions[0].Icon));
  }
  if (Type == "forecast") {
    //Serial.println(json);
    Serial.print(F("\nReceiving Forecast period - ")); //------------------------------------------------
    JsonArray list                    = root["list"];
    for (byte r = 0; r < max_readings; r++) {
      Serial.println("\nPeriod-" + String(r) + "--------------");
      WxForecast[r].Dt                = list[r]["dt"].as<int>();
      WxForecast[r].Temperature       = list[r]["main"]["temp"].as<float>();       Serial.println("Temp: " + String(WxForecast[r].Temperature));
      WxForecast[r].Low               = list[r]["main"]["temp_min"].as<float>();   Serial.println("TLow: " + String(WxForecast[r].Low));
      WxForecast[r].High              = list[r]["main"]["temp_max"].as<float>();   Serial.println("THig: " + String(WxForecast[r].High));
      WxForecast[r].Pressure          = list[r]["main"]["pressure"].as<float>();   Serial.println("Pres: " + String(WxForecast[r].Pressure));
      WxForecast[r].Humidity          = list[r]["main"]["humidity"].as<float>();   Serial.println("Humi: " + String(WxForecast[r].Humidity));
      WxForecast[r].Icon              = (const char*)list[r]["weather"][0]["icon"]; Serial.println("Icon: " + String(WxForecast[r].Icon));
      WxForecast[r].Rainfall          = list[r]["rain"]["3h"].as<float>();         Serial.println("Rain: " + String(WxForecast[r].Rainfall));
      WxForecast[r].Snowfall          = list[r]["snow"]["3h"].as<float>();         Serial.println("Snow: " + String(WxForecast[r].Snowfall));
      if (r < 8) { // Check next 3 x 8 Hours = 1 day
        if (WxForecast[r].High > WxConditions[0].High) WxConditions[0].High = WxForecast[r].High; // Get Highest temperature for next 24Hrs
        if (WxForecast[r].Low  < WxConditions[0].Low)  WxConditions[0].Low  = WxForecast[r].Low;  // Get Lowest  temperature for next 24Hrs
      }
    }
    //------------------------------------------
    float pressure_trend = WxForecast[0].Pressure - WxForecast[2].Pressure; // Measure pressure slope between ~now and later
    pressure_trend = ((int)(pressure_trend * 10)) / 10.0; // Remove any small variations less than 0.1
    WxConditions[0].Trend = "=";
    if (pressure_trend > 0)  WxConditions[0].Trend = "+";
    if (pressure_trend < 0)  WxConditions[0].Trend = "-";
    if (pressure_trend == 0) WxConditions[0].Trend = "0";

    if (Units == "I") Convert_Readings_to_Imperial();
  }
  return true;
}
//#########################################################################################
String ConvertUnixTime(int unix_time) {
  // Returns either '21:12  ' or ' 09:12pm' depending on Units mode
  time_t tm = unix_time;
  struct tm *now_tm = localtime(&tm);
  char output[40];
  if (Units == "M") {
    strftime(output, sizeof(output), "%H:%M %d/%m/%y", now_tm);
  }
  else {
    strftime(output, sizeof(output), "%I:%M%P %m/%d/%y", now_tm);
  }
  return output;
}
//#########################################################################################
bool obtainWeatherData(WiFiClient & client, const String & RequestType) {
  const String units = (Units == "M" ? "metric" : "imperial");
  client.stop(); // close connection before sending a new request
  HTTPClient http;
  //api.openweathermap.org/data/2.5/RequestType?lat={lat}&lon={lon}&appid={API key}
  String uri = "http://api.openweathermap.org/data/2.5/" + RequestType + "?lat=" + Latitude + "&lon=" + Longitude + "&appid=" + apikey + "&mode=json&units=" + units + "&lang=" + Language;
  if (RequestType == "weather") uri += "&exclude=minutely,hourly,alerts,daily";
  http.begin(client, server, 80, uri); //http.begin(uri,test_root_ca); //HTTPS example connection
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    if (!DecodeWeather(http.getStream(), RequestType)) return false;
    client.stop();
  }
  else
  {
    Serial.printf("connection failed to get weather, error: %s", http.errorToString(httpCode).c_str());
    client.stop();
    http.end();
    return false;
  }
  http.end();
  return true;
}

//#########################################################################################
bool obtainWaterFloatData(WiFiClient & client, String & station) {
  WiFiClientSecure wfclient;
  //wfclient.setInsecure();


  HTTPClient http;
  //api.openweathermap.org/data/2.5/RequestType?lat={lat}&lon={lon}&appid={API key}
  //http://environment.data.gov.uk/flood-monitoring/id/measures/3400TH-flow-water-i-15_min-m3_s
  String uri;
  uri = "https://environment.data.gov.uk/flood-monitoring/id/measures/" + station + "-flow-water-i-15_min-m3_s";

  
  const char* root_ca =\
  "-----BEGIN CERTIFICATE-----\n" \
  "MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n" \
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
  "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
  "MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
  "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
  "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n" \
  "9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n" \
  "2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n" \
  "1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n" \
  "q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n" \
  "tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n" \
  "vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n" \
  "BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n" \
  "5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n" \
  "1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n" \
  "NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n" \
  "Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n" \
  "8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n" \
  "pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n" \
  "MrY=\n" \
  "-----END CERTIFICATE-----\n";

  wfclient.setCACert(root_ca);

  http.begin(wfclient, uri); //http.begin(uri,test_root_ca); //HTTPS example connection
  int httpResponseCode = http.GET();
  Serial.println("FLOW HTTP Return code : "+String(httpResponseCode));

  if (httpResponseCode == 301 || httpResponseCode == 302) {
      Serial.println("Redirection occurred...");
      String newLocation = http.getLocation();  // Get the new URL from the Location header
      Serial.print("Redirecting to: ");
      Serial.println(newLocation);
      
      // Close the current connection
      http.end();
      
      // Begin a new connection with the new URL
      http.begin(newLocation);
      httpResponseCode = http.GET();  // Resend the GET request to the new URL

      // Check for successful response
      if (httpResponseCode > 0) {
        String response = http.getString();  // Get response as String
        Serial.println("Response:");
        Serial.println(response);            // Print the response
      } else {
        Serial.print("Error on HTTP request: ");
        Serial.println(httpResponseCode);    // Print error code
        http.end();
        return false;
      }
  } else if (httpResponseCode == HTTP_CODE_OK) {
    if (!DecodeWaterFloat(http.getStream())) return false;
    client.stop();
  } else {
      Serial.printf("connection failed to get water float, error: %s", http.errorToString(httpResponseCode).c_str());
      client.stop();
      http.end();
      return false;
  }
  http.end();
  return true;
}

bool DecodeOctpusAgileRate(WiFiClient& json) {
  int reversedIndex = 48;
  Serial.print(F("\nCreating AgileRate object...and "));
  DynamicJsonDocument aDoc(96 * 1024);                      // allocate the JsonDocument
  DeserializationError error = deserializeJson(aDoc, json); // Deserialize the JSON document
  if (error) {                                             // Test if parsing succeeds.
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return false;
  }
  JsonObject root = aDoc.as<JsonObject>();
  JsonArray results = root["results"];

  //today midnight to next day midnight
  int arraySize = results.size();
  int aPointer = 0;
  int saveAt = 0;
  for ( byte r = 0; r < reversedIndex; r++) {
    aPointer = arraySize - r - 1;
    saveAt = r;
    if ( aPointer >= 0 ) {
      agileTariff[saveAt] = results[aPointer]["value_inc_vat"];
    } else {
      agileTariff[saveAt] = 0.0;
    }
    //Serial.println("Agile Tariff at " + String(saveAt) +" : " + String(agileTariff[saveAt]) +" " + String( aPointer ));
  }
  return true;
}


//###########################################################################################
bool obtainOctpusAgileRate(WiFiClient & client, String & theLocation) {
  client.stop(); // close connection before sending a new request
  //Serial.println(theLocation);

  String newuri = "";
  // char time_string[20];

  // get_time_four_hours_ago_truncated(time_string, sizeof(time_string));

  if ( CurrentHour >= 12 ) {
    newuri = "https://api.octopus.energy/v1/products/" + theLocation + "/electricity-tariffs/E-1R-" + theLocation + "-J/standard-unit-rates/?page_size=100&period_from=!!T12:00";
  } else {
    newuri = "https://api.octopus.energy/v1/products/" + theLocation + "/electricity-tariffs/E-1R-" + theLocation + "-J/standard-unit-rates/?page_size=100&period_from=!!T00:00&period_to=!!T23:59";
  }
  newuri.replace("!!",Date_str);
  Serial.println(newuri);


  HTTPClient octopusAPI;

  octopusAPI.begin(newuri.c_str()); //http.begin(uri,test_root_ca); //HTTPS example connection
  int httpCode = octopusAPI.GET();
  Serial.println("http response code from Octopus : " + String(httpCode));
  if (httpCode == HTTP_CODE_OK) {
    if (!DecodeOctpusAgileRate(octopusAPI.getStream())) return false;
    client.stop();
  }
  else
  {
    Serial.printf("connection failed to get Octopus Agile rate, error: %s", octopusAPI.errorToString(httpCode).c_str());
    client.stop();
    octopusAPI.end();
    return false;
  }
  octopusAPI.end();
  return true;
}


//Here start the Home assistant part



//##-____###___#_#_#_#_#_#_#_#

float mm_to_inches(float value_mm) {
  return 0.0393701 * value_mm;
}

float hPa_to_inHg(float value_hPa) {
  return 0.02953 * value_hPa;
}

int JulianDate(int d, int m, int y) {
  int mm, yy, k1, k2, k3, j;
  yy = y - (int)((12 - m) / 10);
  mm = m + 9;
  if (mm >= 12) mm = mm - 12;
  k1 = (int)(365.25 * (yy + 4712));
  k2 = (int)(30.6001 * mm + 0.5);
  k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
  // 'j' for dates in Julian calendar:
  j = k1 + k2 + d + 59 + 1;
  if (j > 2299160) j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
  return j;
}

float SumOfPrecip(float DataArray[], int readings) {
  float sum = 0;
  for (int i = 0; i <= readings; i++) sum += DataArray[i];
  return sum;
}

String TitleCase(String text) {
  if (text.length() > 0) {
    String temp_text = text.substring(0, 1);
    temp_text.toUpperCase();
    return temp_text + text.substring(1); // Title-case the string
  }
  else return text;
}

void DisplayWeather() {                          // 4.7" e-paper display is 960x540 resolution
  DisplayStatusSection(30, 510, wifi_signal);    // Wi-Fi signal strength and Battery voltage
  DisplayGeneralInfoSection();                   // Top line of the display
  //DisplayAstronomySection(5, 252);               // Astronomy section Sun rise/set, Moon phase and Moon icon
  DisplayMainWeatherSection(375, 178);           // Centre section of display for Location, temperature, Weather report, current Wx Symbol
  //DisplayWeatherIcon(188, 356);                  // Display weather icon scale = Large;
  DisplayWeatherIcon(435, 66);                  // Display weather icon scale = Large;
  DisplayForecastSection(494, 0);              // 3hr forecast boxes
  DisplayGraphAgileTariff(410, 310);
  //DisplayGraphSection(320, 220);                 // Graphs of pressure, temperature, humidity and rain or snowfall
  DisplayGoogleTaskItems(47,249);
}

String truncateSummary(const String& summary, float charLimit) {

  float charCount = 0;
  //int byteCount = 0;  // Tracks the total byte count
  int charIndex = 0;  // Tracks the index in the string

  // Iterate over each character to determine byte size
  while (charIndex < summary.length()) {
    unsigned char c = summary[charIndex];
    int charByteSize = 1;

    // Determine byte size based on UTF-8 character encoding
    if (c <= 0x7F) {
      charByteSize = 1;  // 1-byte character (ASCII)
      charCount += 1.3;
    } else if ((c & 0xE0) == 0xC0) {
      charByteSize = 2;  // 2-byte character
      charCount += 2;
    } else if ((c & 0xF0) == 0xE0) {
      charByteSize = 3;  // 3-byte character (Chinese, Japanese, Korean)
      charCount += 2;      
    } else if ((c & 0xF8) == 0xF0) {
      charByteSize = 4;  // 4-byte character (rare, usually emojis)
      charCount += 2;
    }

    // Check if adding this character would exceed the byte limit
    // if (byteCount + charByteSize > byteLimit) {
    //   break;
    // }
    if ( charCount > charLimit ){
      break;
    }

    // Increment byte count and move to the next character
   //byteCount += charByteSize;
    charIndex += charByteSize;
  }

  // Return truncated string up to the calculated byte limit
  return summary.substring(0, charIndex);
}

// Function to check if the due date has passed
bool isDatePassed(const String& dueDate) {
  int dueYear = dueDate.substring(0, 4).toInt();
  int dueMonth = dueDate.substring(5, 7).toInt();
  int dueDay = dueDate.substring(8, 10).toInt();


  int currentYear = Date_str.substring(0, 4).toInt();
  int currentMonth = Date_str.substring(5, 7).toInt();
  int currentDay = Date_str.substring(8, 10).toInt();

  // Compare years, months, and days to determine if due date is in the past
  if (dueYear < currentYear) return true;
  if (dueYear == currentYear && dueMonth < currentMonth) return true;
  if (dueYear == currentYear && dueMonth == currentMonth && dueDay < currentDay) return true;
  
  return false;
}

void DisplayGoogleTaskItems(int x, int y ) {
  int chineseFontSize = 20;
  int yPos = y;
  String notDue = "[]";
  String showDue = "©";

  for ( int i = 0; i < savedItemCount && i < MAX_TASKS; i++ ) {
    setFont(chinese20);
    String truncatedSummary = truncateSummary(googleTaskItems[i].summary, 20);
    Serial.println("truncated to :"+truncatedSummary);
    drawString(x, yPos, truncatedSummary, LEFT);
    yPos += 39;
    setFont(OpenSans8B);
    String dueText = googleTaskItems[i].due;
    if (isDatePassed(dueText)) {
      dueText = showDue + " " + dueText;  // Add clock icon if due date has passed
    } else {
      dueText = notDue + " " + dueText;
    }
    drawString(x, yPos, dueText, LEFT);
    yPos += 17;
    if (yPos >= SCREEN_HEIGHT - 50) {
      break;
    }
  }
}

void DisplayGeneralInfoSection() {
  setFont(OpenSans8B);
  drawString(345, 495, City, RIGHT);
  drawString(340, 30, Date_str, RIGHT);
  //display time
  setFont(digital7);
  drawString(21, 72, Time_str, LEFT);
}

void DisplayWeatherIcon(int x, int y) {
  DisplayConditionsSection(x, y, WxConditions[0].Icon, SmallIcon);
  setFont(OpenSans8B);
  drawString(x + 5, y - 39, "Now", CENTER);
}

void DisplayMainWeatherSection(int x, int y) {
  //setFont(inverted);
  DisplayTempHumiPressSection(x + 20, y - 20);
  //DisplayForecastTextSection(x - 55, y + 45);
  //DisplayVisiCCoverUVISection(x - 10, y + 95);
}

void DisplayTempHumiPressSection(int x, int y) {
  setFont(inverted);
  fillRect( x , y, 113, 65, Black);
  fillRect( x + 195, y, 115, 65, Black);
  fillRect( x + 365, y, 122, 65, Black);
  drawString(x + 110, y - 10 , String(WxConditions[0].Temperature, 1), RIGHT);
  drawString(x + 80 + 30 + 195, y -10 , String(WxConditions[0].Humidity, 0), RIGHT);
  drawString(x + 80 + 32 + 365, y -10 , String(WxConditions[0].WaterFloat, 0), RIGHT);
  /*
  DrawPressureAndTrend(x + 195, y + 15, WxConditions[0].Pressure, WxConditions[0].Trend);
  int Yoffset = 42;
  if (WxConditions[0].Windspeed > 0) {
    drawString(x - 30, y + Yoffset, String(WxConditions[0].FeelsLike, 1) + "° FL", LEFT);   // Show FeelsLike temperature if windspeed > 0
    Yoffset += 30;
  }
  */
  setFont(agency25);
  drawString(920, y + 75, String(WxConditions[0].High, 0) + "° | " + String(WxConditions[0].Low, 0) + "° Hi/Lo", RIGHT); // Show forecast high and Low
}

void DisplayForecastTextSection(int x, int y) {
#define lineWidth 34
  setFont(OpenSans12B);
  String Wx_Description = WxConditions[0].Forecast0;
  Wx_Description.replace(".", ""); // remove any '.'
  int spaceRemaining = 0, p = 0, charCount = 0, Width = lineWidth;
  while (p < Wx_Description.length()) {
    if (Wx_Description.substring(p, p + 1) == " ") spaceRemaining = p;
    if (charCount > Width - 1) { // '~' is the end of line marker
      Wx_Description = Wx_Description.substring(0, spaceRemaining) + "~" + Wx_Description.substring(spaceRemaining + 1);
      charCount = 0;
    }
    p++;
    charCount++;
  }
  if (WxForecast[0].Rainfall > 0) Wx_Description += " (" + String(WxForecast[0].Rainfall, 1) + String((Units == "M" ? "mm" : "in")) + ")";
  String Line1 = Wx_Description.substring(0, Wx_Description.indexOf("~"));
  String Line2 = Wx_Description.substring(Wx_Description.indexOf("~") + 1);
  drawString(x + 30, y + 5, TitleCase(Line1), LEFT);
  if (Line1 != Line2) drawString(x + 30, y + 30, Line2, LEFT);
}

void Display_WaterFloatLevel(int x, int y, float WFR) {
  String Level = "";
  drawString(x + 20, y - 5, String(WFR, (WFR < 0 ? 1 : 0)), LEFT);
}

void DisplayForecastWeather(int x, int y, int index, int fwidth) {
  x = x + fwidth * index;
  DisplayConditionsSection(x + fwidth / 2 - 5, y + 65, WxForecast[index].Icon, SmallIcon);
  setFont(OpenSans8B);
  drawString(x + (fwidth / 2) - 4, y + 30, String(ConvertUnixTime(WxForecast[index].Dt + WxConditions[0].FTimezone).substring(0, 5)), CENTER);
  //drawString(x + fwidth / 2, y + 130, String(WxForecast[index].High, 0) + "°/" + String(WxForecast[index].Low, 0) + "°", CENTER);
}

void DisplayForecastSection(int x, int y) {
  int f = 0;
  do {
    DisplayForecastWeather(x, y, f, 75); // x,y cordinates, forecatsr number, spacing width (82)
    f++;
  } while (f < 6);
}

void DisplayGraphAgileTariff(int x, int y) {
  int gwidth = (SCREEN_WIDTH - x - 30 ), gheight = ( SCREEN_HEIGHT - y - 50);
  int gx = x, gy = y;
  DrawGraph( gx, gy, gwidth, gheight, -10, 80, "Agile Octopus", agileTariff, 48, autoscale_off, barchart_on);
}

void DisplayGraphSection(int x, int y) {

}

void DisplayConditionsSection(int x, int y, String IconName, bool IconSize) {
  Serial.println("Icon name: " + IconName);
  if      (IconName == "01d" || IconName == "01n") ClearSky(x, y, IconSize, IconName);
  else if (IconName == "02d" || IconName == "02n") FewClouds(x, y, IconSize, IconName);
  else if (IconName == "03d" || IconName == "03n") ScatteredClouds(x, y, IconSize, IconName);
  else if (IconName == "04d" || IconName == "04n") BrokenClouds(x, y, IconSize, IconName);
  else if (IconName == "09d" || IconName == "09n") ChanceRain(x, y, IconSize, IconName);
  else if (IconName == "10d" || IconName == "10n") Rain(x, y, IconSize, IconName);
  else if (IconName == "11d" || IconName == "11n") Thunderstorms(x, y, IconSize, IconName);
  else if (IconName == "13d" || IconName == "13n") Snow(x, y, IconSize, IconName);
  else if (IconName == "50d" || IconName == "50n") Mist(x, y, IconSize, IconName);
  else                                             Nodata(x, y, IconSize, IconName);
}

void arrow(int x, int y, int asize, float aangle, int pwidth, int plength) {
  float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x; // calculate X position
  float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y; // calculate Y position
  float x1 = 0;         float y1 = plength;
  float x2 = pwidth / 2;  float y2 = pwidth / 2;
  float x3 = -pwidth / 2; float y3 = pwidth / 2;
  float angle = aangle * PI / 180 - 135;
  float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
  float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
  float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
  float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
  float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
  float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
  fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, Black);
}

void DrawSegment(int x, int y, int o1, int o2, int o3, int o4, int o11, int o12, int o13, int o14) {
  drawLine(x + o1,  y + o2,  x + o3,  y + o4,  Black);
  drawLine(x + o11, y + o12, x + o13, y + o14, Black);
}

void DrawPressureAndTrend(int x, int y, float pressure, String slope) {
  drawString(x + 25, y - 10, String(pressure, (Units == "M" ? 0 : 1)) + (Units == "M" ? "hPa" : "in"), LEFT);
  if      (slope == "+") {
    DrawSegment(x, y, 0, 0, 8, -8, 8, -8, 16, 0);
    DrawSegment(x - 1, y, 0, 0, 8, -8, 8, -8, 16, 0);
  }
  else if (slope == "0") {
    DrawSegment(x, y, 8, -8, 16, 0, 8, 8, 16, 0);
    DrawSegment(x - 1, y, 8, -8, 16, 0, 8, 8, 16, 0);
  }
  else if (slope == "-") {
    DrawSegment(x, y, 0, 0, 8, 8, 8, 8, 16, 0);
    DrawSegment(x - 1, y, 0, 0, 8, 8, 8, 8, 16, 0);
  }
}

void DisplayStatusSection(int x, int y, int rssi) {
  setFont(OpenSans8B);
  //DrawRSSI(x + 305, y + 15, rssi);
  DrawRSSI(x, y, rssi);
  DrawBattery(x + 30, y);
}

void DrawRSSI(int x, int y, int rssi) {
  int WIFIsignal = 0;
  int xpos = 1;
  for (int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20) {
    if (_rssi <= -20)  WIFIsignal = 30; //            <-20dbm displays 5-bars
    if (_rssi <= -40)  WIFIsignal = 24; //  -40dbm to  -21dbm displays 4-bars
    if (_rssi <= -60)  WIFIsignal = 18; //  -60dbm to  -41dbm displays 3-bars
    if (_rssi <= -80)  WIFIsignal = 12; //  -80dbm to  -61dbm displays 2-bars
    if (_rssi <= -100) WIFIsignal = 6;  // -100dbm to  -81dbm displays 1-bar
    fillRect(x + xpos * 8, y - WIFIsignal, 6, WIFIsignal, Black);
    xpos++;
  }
}

boolean UpdateLocalTime() {
  struct tm timeinfo;
  char   time_output[30], day_output[30], update_time[30], update_date[30];
  while (!getLocalTime(&timeinfo, 5000)) { // Wait for 5-sec for time to synchronise
    Serial.println("Failed to obtain time");
    return false;
  }
  CurrentHour = timeinfo.tm_hour;
  CurrentMin  = timeinfo.tm_min;
  CurrentSec  = timeinfo.tm_sec;
  //See http://www.cplusplus.com/reference/ctime/strftime/
  Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S");      // Displays: Saturday, June 24 2017 14:05:49
  if (Units == "M") {
    //sprintf(day_output,  "%s, %02u %s %04u", weekday_D[timeinfo.tm_wday], timeinfo.tm_mday, month_M[timeinfo.tm_mon], (timeinfo.tm_year) + 1900);
    sprintf(day_output,  "%04u-%02u-%02u", (timeinfo.tm_year) + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday );
    //strftime(update_time, sizeof(update_time), "%H:%M:%S", &timeinfo);  // Creates: '@ 14:05:49'   and change from 30 to 8 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    strftime(update_time, sizeof(update_time), "%H:%M", &timeinfo);  // Creates: '14:05'   and change from 30 to 5 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    sprintf(time_output, "%s", update_time);
  }
  else
  {
    strftime(day_output, sizeof(day_output), "%F", &timeinfo); // Creates  'Sat May-31-2019'
    strftime(update_time, sizeof(update_time), "%r", &timeinfo);        // Creates: '@ 02:05:49pm'
    sprintf(time_output, "%s", update_time);
  }
  Date_str = day_output;
  Time_str = time_output;
  return true;
}

void DrawBattery(int x, int y) {
  uint8_t percentage = 100;
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }
  Serial.println("The Vref :=>" + String(vref));
  float voltage = analogRead(BATT_PIN) / 4096.0 * 6.566 * (vref / 1000.0);
  Serial.println("The Voltage :" + String(voltage));
  if (voltage > 1 ) { // Only display if there is a valid reading
    percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
    if (voltage >= 4.20) percentage = 100;
    if (voltage <= 3.20) percentage = 0;  // orig 3.5
    drawRect(x + 25, y - 14, 40, 15, Black);
    fillRect(x + 65, y - 10, 4, 7, Black);
    fillRect(x + 27, y - 12, 36 * percentage / 100.0, 11, Black);
    drawString(x + 70, y - 14, String(percentage) + "% " + String(voltage, 1) + "v" + " ", LEFT);
  }
}

// Symbols are drawn on a relative 10x10grid and 1 scale unit = 1 drawing unit
void addcloud(int x, int y, int scale, int linesize) {
  fillCircle(x - scale * 3, y, scale, Black);                                                              // Left most circle
  fillCircle(x + scale * 3, y, scale, Black);                                                              // Right most circle
  fillCircle(x - scale, y - scale, scale * 1.4, Black);                                                    // left middle upper circle
  fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, Black);                                       // Right middle upper circle
  fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, Black);                                 // Upper and lower lines
  fillCircle(x - scale * 3, y, scale - linesize, White);                                                   // Clear left most circle
  fillCircle(x + scale * 3, y, scale - linesize, White);                                                   // Clear right most circle
  fillCircle(x - scale, y - scale, scale * 1.4 - linesize, White);                                         // left middle upper circle
  fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, White);                            // Right middle upper circle
  fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2, White); // Upper and lower lines
}

void addrain(int x, int y, int scale, bool IconSize) {
  if (IconSize == SmallIcon) {
    setFont(OpenSans8B);
    drawString(x - 25, y + 12, "///////", LEFT);
  }
  else
  {
    setFont(OpenSans18B);
    drawString(x - 60, y + 25, "///////", LEFT);
  }
}

void addsnow(int x, int y, int scale, bool IconSize) {
  if (IconSize == SmallIcon) {
    setFont(OpenSans8B);
    drawString(x - 25, y + 15, "* * * *", LEFT);
  }
  else
  {
    setFont(OpenSans18B);
    drawString(x - 60, y + 30, "* * * *", LEFT);
  }
}

void addtstorm(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 1; i < 5; i++) {
    drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, Black);
    drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, Black);
    drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, Black);
    drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, Black);
    drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, Black);
    drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, Black);
    drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, Black);
    drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, Black);
    drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, Black);
  }
}

void addsun(int x, int y, int scale, bool IconSize) {
  int linesize = 5;
  fillRect(x - scale * 2, y, scale * 4, linesize, Black);
  fillRect(x, y - scale * 2, linesize, scale * 4, Black);
  DrawAngledLine(x + scale * 1.4, y + scale * 1.4, (x - scale * 1.4), (y - scale * 1.4), linesize * 1.5, Black); // Actually sqrt(2) but 1.4 is good enough
  DrawAngledLine(x - scale * 1.4, y + scale * 1.4, (x + scale * 1.4), (y - scale * 1.4), linesize * 1.5, Black);
  fillCircle(x, y, scale * 1.3, White);
  fillCircle(x, y, scale, Black);
  fillCircle(x, y, scale - linesize, White);
}

void addfog(int x, int y, int scale, int linesize, bool IconSize) {
  if (IconSize == SmallIcon) linesize = 3;
  for (int i = 0; i < 6; i++) {
    fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize, Black);
    fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize, Black);
    fillRect(x - scale * 3, y + scale * 2.5, scale * 6, linesize, Black);
  }
}

void DrawAngledLine(int x, int y, int x1, int y1, int size, int color) {
  int dx = (size / 2.0) * (x - x1) / sqrt(sq(x - x1) + sq(y - y1));
  int dy = (size / 2.0) * (y - y1) / sqrt(sq(x - x1) + sq(y - y1));
  fillTriangle(x + dx, y - dy, x - dx,  y + dy,  x1 + dx, y1 - dy, color);
  fillTriangle(x - dx, y + dy, x1 - dx, y1 + dy, x1 + dx, y1 - dy, color);
}

void ClearSky(int x, int y, bool IconSize, String IconName) {
  int scale = Small;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  if (IconSize == LargeIcon) scale = Large;
  y += (IconSize ? 0 : 10);
  addsun(x, y, scale * (IconSize ? 1.7 : 1.2), IconSize);
}

void BrokenClouds(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  y += 15;
  if (IconSize == LargeIcon) scale = Large;
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
}

void FewClouds(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  y += 15;
  if (IconSize == LargeIcon) scale = Large;
  addcloud(x + (IconSize ? 10 : 0), y, scale * (IconSize ? 0.9 : 0.8), linesize);
  addsun((x + (IconSize ? 10 : 0)) - scale * 1.8, y - scale * 1.6, scale, IconSize);
}

void ScatteredClouds(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  y += 15;
  if (IconSize == LargeIcon) scale = Large;
  addcloud(x - (IconSize ? 35 : 0), y * (IconSize ? 0.75 : 0.93), scale / 2, linesize); // Cloud top left
  addcloud(x, y, scale * 0.9, linesize);                                         // Main cloud
}

void Rain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  y += 15;
  if (IconSize == LargeIcon) scale = Large;
  addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
  addrain(x, y, scale, IconSize);
}

void ChanceRain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  if (IconSize == LargeIcon) scale = Large;
  y += 15;
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale * (IconSize ? 1 : 0.65), linesize);
  addrain(x, y, scale, IconSize);
}

void Thunderstorms(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  if (IconSize == LargeIcon) scale = Large;
  y += 5;
  addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
  addtstorm(x, y, scale);
}

void Snow(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  if (IconSize == LargeIcon) scale = Large;
  addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
  addsnow(x, y, scale, IconSize);
}

void Mist(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5;
  if (IconName.endsWith("n")) addmoon(x, y, IconSize);
  if (IconSize == LargeIcon) scale = Large;
  addsun(x, y, scale * (IconSize ? 1 : 0.75), linesize);
  addfog(x, y, scale, linesize, IconSize);
}

void CloudCover(int x, int y, int CloudCover) {
  addcloud(x - 9, y,     Small * 0.3, 2); // Cloud top left
  addcloud(x + 3, y - 2, Small * 0.3, 2); // Cloud top right
  addcloud(x, y + 15,    Small * 0.6, 2); // Main cloud
  drawString(x + 30, y, String(CloudCover) + "%", LEFT);
}

void Visibility(int x, int y, String Visibility) {
  float start_angle = 0.52, end_angle = 2.61, Offset = 10;
  int r = 14;
  for (float i = start_angle; i < end_angle; i = i + 0.05) {
    drawPixel(x + r * cos(i), y - r / 2 + r * sin(i) + Offset, Black);
    drawPixel(x + r * cos(i), 1 + y - r / 2 + r * sin(i) + Offset, Black);
  }
  start_angle = 3.61; end_angle = 5.78;
  for (float i = start_angle; i < end_angle; i = i + 0.05) {
    drawPixel(x + r * cos(i), y + r / 2 + r * sin(i) + Offset, Black);
    drawPixel(x + r * cos(i), 1 + y + r / 2 + r * sin(i) + Offset, Black);
  }
  fillCircle(x, y + Offset, r / 4, Black);
  drawString(x + 20, y, Visibility, LEFT);
}

void addmoon(int x, int y, bool IconSize) {
  int xOffset = 65;
  int yOffset = 12;
  if (IconSize == LargeIcon) {
    xOffset = 130;
    yOffset = -40;
  }
  fillCircle(x - 28 + xOffset, y - 37 + yOffset, uint16_t(Small * 1.0), Black);
  fillCircle(x - 16 + xOffset, y - 37 + yOffset, uint16_t(Small * 1.6), White);
}

void Nodata(int x, int y, bool IconSize, String IconName) {
  if (IconSize == LargeIcon) setFont(chinese20); else setFont(OpenSans12B);
  drawString(x - 3, y - 10, "?", CENTER);
}

void DrawBaseImage() {
  Rect_t area = {
    .x = 0, .y = 0, .width  = baseimg_width, .height =  baseimg_height
  };
  epd_draw_grayscale_image(area, (uint8_t *) baseimg_data);
}

/* (C) D L BIRD
    This function will draw a graph on a ePaper/TFT/LCD display using data from an array containing data to be graphed.
    The variable 'max_readings' determines the maximum number of data elements for each array. Call it with the following parametric data:
    x_pos-the x axis top-left position of the graph
    y_pos-the y-axis top-left position of the graph, e.g. 100, 200 would draw the graph 100 pixels along and 200 pixels down from the top-left of the screen
    width-the width of the graph in pixels
    height-height of the graph in pixels
    Y1_Max-sets the scale of plotted data, for example 5000 would scale all data to a Y-axis of 5000 maximum
    data_array1 is parsed by value, externally they can be called anything else, e.g. within the routine it is called data_array1, but externally could be temperature_readings
    auto_scale-a logical value (TRUE or FALSE) that switches the Y-axis autoscale On or Off
    barchart_on-a logical value (TRUE or FALSE) that switches the drawing mode between barhcart and line graph
    barchart_colour-a sets the title and graph plotting colour
    If called with Y!_Max value of 500 and the data never goes above 500, then autoscale will retain a 0-500 Y scale, if on, the scale increases/decreases to match the data.
    auto_scale_margin, e.g. if set to 1000 then autoscale increments the scale by 1000 steps.
*/

int toggle(int value) {
  return value == 0 ? 1 : 0;
}

void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title, float DataArray[], int readings, boolean auto_scale, boolean barchart_mode) {
#define auto_scale_margin 0 // Sets the autoscale increment, so axis steps up fter a change of e.g. 3
#define y_minor_axis 5      // 5 y-axis division markers
  //setFont(OpenSans8B);
  setFont(agency20);
  int maxYscale = -10000;
  int minYscale =  10000;
  int last_x, last_y;
  int pmShift;
  float x2, y2, barWidth, minus2zero, moveup;
  String IsAMPM[2];
  String xaxisLabel;

  IsAMPM[0]="AM";
  IsAMPM[1]="PM";

 
  for (int i = 1; i < readings; i++ ) {
      if (DataArray[i] >= maxYscale) maxYscale = DataArray[i];
      if (DataArray[i] <= minYscale) minYscale = DataArray[i];
  }

  if (auto_scale == true) {
    maxYscale = round(maxYscale + auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Max
    Y1Max = round(maxYscale + 0.5);
    if (minYscale != 0) minYscale = round(minYscale - auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Min
    Y1Min = round(minYscale);
  }
 

  // Draw the graph
  last_x = x_pos + 1;
  last_y = y_pos + (Y1Max - constrain(DataArray[1], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight;
  drawRect(x_pos, y_pos, gwidth + 3, gheight + 2, Grey);
  //setFont(OpenSans10B);
  //drawString(x_pos - 20 + gwidth / 2, y_pos - 28, title, CENTER);
  //setFont(OpenSans8B);
  bool firstHalf = true;

  pmShift = ( CurrentHour >= 12) ? 12 : 0;

  for (int gx = 0; gx < readings; gx++) {
    moveup = ( DataArray[gx] >= 0 ) ? 10 : 0;
    x2 = x_pos + gx * gwidth / (readings - 1) - 1 ; // max_readings is the global variable that sets the maximum data that can be plotted
    y2 = y_pos + (Y1Max - constrain(DataArray[gx], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight + 1;
    //Serial.println("Drawing position = X: "+ String(x2) +" LastX: "+ String(x2) +" Y:" + String(y2));

    //ChatGPT optimized code
    if (barchart_mode) {
      barWidth = (gwidth / readings) - 1;
      if (firstHalf && CurrentMin < 30 && (CurrentHour - pmShift ) == (gx / 2)) {
          drawString(x_pos + 5, y_pos + 5, "NOW:" + String(DataArray[gx]) + "p MAX:" + String(maxYscale) + "p", LEFT);
          arrow(last_x + barWidth, last_y - barWidth - 20, barWidth, 180, 20, 20);
      } else if (!firstHalf && CurrentMin >= 30 && (CurrentHour - pmShift) == (gx / 2)) {
          drawString(x_pos + 5, y_pos + 5, "NOW:" + String(DataArray[gx]) + "p MAX:" + String(maxYscale) + "p", LEFT);
          arrow(last_x, last_y - barWidth - 20, barWidth, 180, 20, 20);
      }
      fillRect(last_x + (firstHalf ? 4 : 0), y2, barWidth, y_pos + gheight - y2 + 2 - moveup, Black);
      if (firstHalf) fillRect(last_x + 6, y2 + 2, barWidth - 3, y_pos + gheight - y2 - moveup, White);
      firstHalf = !firstHalf;
    } else {
      drawLine(last_x, last_y - 1, x2, y2 - 1, Black);
      drawLine(last_x, last_y, x2, y2, Black);
    }
    last_x = x2;
    last_y = y2;
  }
  //Draw the Y-axis scale
#define number_of_dashes 40
  setFont(OpenSans8B);
  for (int spacing = 0; spacing <= y_minor_axis; spacing++) {
    for (int j = 0; j < number_of_dashes; j++) { // Draw dashed graph grid lines
      if (spacing < y_minor_axis) drawFastHLine((x_pos + 3 + j * gwidth / number_of_dashes), y_pos + (gheight * spacing / y_minor_axis), gwidth / (2 * number_of_dashes), Grey);
    }
    if ((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing) < 5 || title == TXT_PRESSURE_IN) {
      drawString(x_pos - 10, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
    }
    else
    {
      if (Y1Min < 1 && Y1Max < 10) {
        drawString(x_pos - 3, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
      }
      else {
        drawString(x_pos - 7, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 0), RIGHT);
      }
    }
  }

  for (int i = 1; i < 7; i++) {
    if ( i > 3) {
      if ( i == 6 ) {
        xaxisLabel = "12" + IsAMPM[ pmShift / 12 ]; // draw PM
      } else {
        xaxisLabel = String( (i * 4) - 12 ) + IsAMPM[ toggle(pmShift / 12) ]; // draw PM
      }
    } else {
      if ( i == 3 ) {
        xaxisLabel = "12" + IsAMPM[ toggle(pmShift / 12) ]; // draw PM
      } else {
        xaxisLabel = String(i * 4) + IsAMPM[ pmShift / 12 ]; // draw AM
      }
    }
    drawString(x_pos + gwidth / 6 * i, y_pos + gheight + 10, xaxisLabel, CENTER);   
    //if (i < 2) drawFastVLine(x_pos + gwidth / 3 * i + gwidth / 3, y_pos, gheight, LightGrey);
  }
}

void drawString(int x, int y, String text, alignment align) {
  char * data  = const_cast<char*>(text.c_str());
  int  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  int w, h;
  int xx = x, yy = y;
  get_text_bounds(&currentFont, data, &xx, &yy, &x1, &y1, &w, &h, NULL);
  if (align == RIGHT)  x = x - w;
  if (align == CENTER) x = x - w / 2;
  int cursor_y = y + h;
  write_string(&currentFont, data, &x, &cursor_y, framebuffer);
}

void fillCircle(int x, int y, int r, uint8_t color) {
  epd_fill_circle(x, y, r, color, framebuffer);
}

void drawFastHLine(int16_t x0, int16_t y0, int length, uint16_t color) {
  epd_draw_hline(x0, y0, length, color, framebuffer);
}

void drawFastVLine(int16_t x0, int16_t y0, int length, uint16_t color) {
  epd_draw_vline(x0, y0, length, color, framebuffer);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  epd_write_line(x0, y0, x1, y1, color, framebuffer);
}

void drawCircle(int x0, int y0, int r, uint8_t color) {
  epd_draw_circle(x0, y0, r, color, framebuffer);
}

void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  epd_draw_rect(x, y, w, h, color, framebuffer);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  epd_fill_rect(x, y, w, h, color, framebuffer);
}

void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                  int16_t x2, int16_t y2, uint16_t color) {
  epd_fill_triangle(x0, y0, x1, y1, x2, y2, color, framebuffer);
}

void drawPixel(int x, int y, uint8_t color) {
  epd_draw_pixel(x, y, color, framebuffer);
}

void setFont(GFXfont const & font) {
  currentFont = font;
}

void edp_update() {
  epd_draw_grayscale_image(epd_full_screen(), framebuffer); // Update the screen
}
/*
   1071 lines of code 03-03-2021
*/
