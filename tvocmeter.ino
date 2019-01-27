#include <Arduino.h>
#include <EEPROM.h>

#include <Wire.h>
#include <SparkFunCCS811.h>
#include <SparkFunBME280.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "FastLED.h"

#define PIN_CCS811_SCL  D1
#define PIN_CCS811_SDA  D2
#define PIN_CCS811_WAK  D3
#define PIN_CCS811_GND  D8
#define PIN_NEOPIXEL    D5

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
    uint32_t color;
} level_t;

static const level_t levels[] = {
    {25, 0x00FF00},
    {50, 0x00FF00},
    {100, 0x00FF00},
    {200, 0x00FF00},
    {400, 0xFFFF00},
    {800, 0xFFFF00},
    {1600, 0xFF0000},
    {3200, 0xFF0000},
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
    Serial.begin(115200);
    print("\nTVOC meter\n");

    // get ESP id
    sprintf(esp_id, "%08X", ESP.getChipId());
    print("ESP ID: %s\n", esp_id);

    // LED init
    FastLED.addLeds<NEOPIXEL, PIN_NEOPIXEL>(leds, NUM_LEDS);

    // setup I2C
    Wire.begin(PIN_CCS811_SDA, PIN_CCS811_SCL);

    // setup BME280
    bme280.setI2CAddress(0x76);
    if (!bme280.beginI2C()) {
        print("bme280.begin() returned with an error.\n");
        while (1);
    }

    // setup CCS811
    pinMode(PIN_CCS811_GND, OUTPUT);
    digitalWrite(PIN_CCS811_GND, 0);
    pinMode(PIN_CCS811_WAK, OUTPUT);
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
            leds[idx] = CRGB(l->color);
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

    static uint32_t meas_total = 0;
    static int meas_num = 0;

    char topic[128];
    char message[16];

    // read CCS811 if available
    if (ccs811.dataAvailable()) {
        ccs811.readAlgorithmResults();
        uint16_t tvoc = ccs811.getTVOC();
        meas_total += tvoc;
        meas_num++;

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

    // log TVOC over MQTT every LOG_PERIOD_SEC
    if ((second - second_log) > LOG_PERIOD_SEC) {
        second_log = second;

        // calculate average
        if (meas_num > 0) {
            uint16_t tvoc = (meas_total + (meas_num / 2)) / meas_num; 
            meas_num = 0;
            meas_total = 0;

            // send over MQTT
            snprintf(topic, sizeof(topic), MQTT_TOPIC, esp_id, "/ccs811/tvoc");
            snprintf(message, sizeof(message), "%u", tvoc);
            mqtt_send(topic, message);
        }
    }

    // keep MQTT alive
    mqttClient.loop();
}

