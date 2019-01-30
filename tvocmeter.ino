#include <Arduino.h>
#include <EEPROM.h>

#include <Wire.h>
#include <SparkFunCCS811.h>
#include <SparkFunBME280.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "FastLED.h"

#define PIN_LED_VCC     D2
#define PIN_LED_DATA    D3

#define PIN_CCS811_WAK  D5
#define PIN_CCS811_SDA  D6
#define PIN_CCS811_SCL  D7
#define PIN_CCS811_GND  D8

#define CCS811_ADDR 0x5A

#define MQTT_HOST   "stofradar.nl"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "bertrik/%s%s"      // esp_id/subtopic

#define BASELINE_PERIOD_SEC 3600
#define ENV_PERIOD_SEC      10
#define LOG_PERIOD_SEC      10

#define NUM_LEDS 8

#define NVDATA_MAGIC 0x1337
struct {
    uint16_t baseline;
    uint32_t magic;
} nvdata;

typedef struct {
    int level;
    CRGB rgb;
} level_t;

typedef struct {
    uint32_t total;
    uint16_t count;
} averager_t;

static const level_t levels[] = {
    {62,    CRGB::Green},
    {125,   CRGB::Green},
    {250,   CRGB::Green},
    {500,   CRGB::Green},
    {1000,  CRGB::Yellow},
    {2000,  CRGB::Yellow},
    {4000,  CRGB::Red},
    {8000,  CRGB::Red},
    {-1, 0}
};

static CCS811 ccs811(CCS811_ADDR);
static BME280 bme280;
static char esp_id[16];

static WiFiManager wifiManager;
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static CRGB leds[NUM_LEDS];

// printf-like output to serial port
static void print(const char *fmt, ...)
{
    // format it
    char buf[256];
    va_list args;
    va_start (args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end (args);

    // send it to serial
    Serial.write(buf);
}

void setup(void)
{
    // setup pins
    pinMode(PIN_CCS811_GND, OUTPUT);
    digitalWrite(PIN_CCS811_GND, 0);
    pinMode(PIN_CCS811_WAK, OUTPUT);
    digitalWrite(PIN_CCS811_WAK, 1);

    // greeting
    Serial.begin(115200);
    print("\nTVOC meter\n");

    // get ESP id
    sprintf(esp_id, "%08X", ESP.getChipId());
    print("ESP ID: %s\n", esp_id);

    // setup I2C
    Wire.begin(PIN_CCS811_SDA, PIN_CCS811_SCL);

    // setup BME280
    bme280.setI2CAddress(0x76);
    if (!bme280.beginI2C()) {
        print("bme280.begin() returned with an error.\n");
        while (1);
    }

    // setup CCS811
    digitalWrite(PIN_CCS811_WAK, 0);
    CCS811Core::status returnCode = ccs811.begin();
    if (returnCode != CCS811Core::SENSOR_SUCCESS) {
        print("ccs811.begin() returned with an error.\n");
        while (1);
    }

    // restore CCS811 baseline
    EEPROM.begin(sizeof(nvdata));
    EEPROM.get(0, nvdata);
    if (nvdata.magic == NVDATA_MAGIC) {
        print("Restoring base line value %04X\n", nvdata.baseline);
        ccs811.setBaseline(nvdata.baseline);
    }

    // LED init
    pinMode(PIN_LED_VCC, OUTPUT);
    digitalWrite(PIN_LED_VCC, 1);
    FastLED.addLeds<WS2812, PIN_LED_DATA, RGB>(leds, NUM_LEDS);
    FastLED.setBrightness(5);
    FastLED.show();

    // connect to wifi
    print("Starting WIFI manager ...\n");
    wifiManager.autoConnect("ESP-TVOC");
}

static void mqtt_send(const char *topic, const char *value)
{
    if (!mqttClient.connected()) {
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        mqttClient.connect(esp_id);
    }
    if (mqttClient.connected()) {
        print("Publishing %s to %s ...", value, topic);
        int result = mqttClient.publish(topic, value, true);
        print(result ? "OK" : "FAIL");
        print("\n");
    }
}

static void show_on_led(uint16_t tvoc)
{
    int idx = 0;
    for (idx = 0; levels[idx].level > 0; idx++) {
        const level_t *l = &levels[idx];
        if (tvoc >= l->level) {
            leds[idx] = l->rgb;
            print("#");
        } else {
            leds[idx] = CRGB::Black; 
            print(".");
        }
    }
    print(" %d\n", tvoc);
    FastLED.show(); 
}

void loop(void)
{
    static unsigned long second_log = 0;
    static unsigned long second_baseline = 0;
    static unsigned long second_env = 0;

    static averager_t tvoc_avg = {0, 0};
    static averager_t eco2_avg = {0, 0};

    char topic[128];
    char message[16];

    // read CCS811 if available
    if (ccs811.dataAvailable()) {
        ccs811.readAlgorithmResults();
        uint16_t tvoc = ccs811.getTVOC();
        tvoc_avg.total += tvoc;
        tvoc_avg.count++;
        uint16_t eco2 = ccs811.getCO2();
        eco2_avg.total += eco2;
        eco2_avg.count++;

        // update LEDs
        show_on_led(tvoc);
    }

    // save CCS811 baseline every BASELINE_PERIOD_SEC
    unsigned long second = millis() / 1000;
    if ((second - second_baseline) > BASELINE_PERIOD_SEC) {
        second_baseline = second;
        nvdata.baseline = ccs811.getBaseline();
        nvdata.magic = NVDATA_MAGIC;

        print("Saving baseline value %04X\n", nvdata.baseline);
        EEPROM.put(0, nvdata);
        EEPROM.commit();
    }

    // update enivironment data every ENV_PERIOD_SEC
    if ((second - second_env) > ENV_PERIOD_SEC) {
        second_env = second;

        // disable CCS811 WAKE and read environment data
        digitalWrite(PIN_CCS811_WAK, 1);
        float tempC = bme280.readTempC();
        float humidity = bme280.readFloatHumidity();

        // enable CCS811 WAKE and write environment data
        digitalWrite(PIN_CCS811_WAK, 0);
        print("Applying T/RH compensation: T=%.2f, RH=%.2f\n", tempC, humidity);
        ccs811.setEnvironmentalData(humidity, tempC);
    
        // log to MQTT
        snprintf(topic, sizeof(topic), MQTT_TOPIC, esp_id, "/bme280/temperature");
        snprintf(message, sizeof(message), "%.2f", tempC);
        mqtt_send(topic, message);

        snprintf(topic, sizeof(topic), MQTT_TOPIC, esp_id, "/bme280/humidity");
        snprintf(message, sizeof(message), "%.2f", humidity);
        mqtt_send(topic, message);
    }

    // log TVOC and eCO2 over MQTT every LOG_PERIOD_SEC
    if ((second - second_log) > LOG_PERIOD_SEC) {
        second_log = second;

        // calculate average TVOC
        if (tvoc_avg.count > 0) {
            uint16_t tvoc = (tvoc_avg.total + (tvoc_avg.count / 2)) / tvoc_avg.count; 
            tvoc_avg.total = 0;
            tvoc_avg.count = 0;

            // send over MQTT
            snprintf(topic, sizeof(topic), MQTT_TOPIC, esp_id, "/ccs811/tvoc");
            snprintf(message, sizeof(message), "%u", tvoc);
            mqtt_send(topic, message);
        }

        // calculate average eCO2
        if (eco2_avg.count > 0) {
            uint16_t eco2 = (eco2_avg.total + (eco2_avg.count / 2)) / eco2_avg.count; 
            eco2_avg.total = 0;
            eco2_avg.count = 0;

            // send over MQTT
            snprintf(topic, sizeof(topic), MQTT_TOPIC, esp_id, "/ccs811/eco2");
            snprintf(message, sizeof(message), "%u", eco2);
            mqtt_send(topic, message);
        }
    }

    // keep MQTT alive
    mqttClient.loop();
}

