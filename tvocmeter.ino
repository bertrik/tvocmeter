#include <Arduino.h>

#include <Wire.h>
#include <SparkFunCCS811.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "FastLED.h"

#define PIN_CCS811_SCL  D1
#define PIN_CCS811_SDA  D2
#define PIN_CCS811_WAK  D3
#define PIN_CCS811_GND  D8
#define PIN_NEOPIXEL    D4

#define CCS811_ADDR 0x5A

#define MQTT_HOST   "test.mosquitto.org"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "revspace/sensors/tvoc"

#define LOG_PERIOD_SEC  10000

#define NUM_LEDS 8

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
static char esp_id[16];

static WiFiManager wifiManager;
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static CRGB leds[NUM_LEDS];

void setup(void)
{
    Serial.begin(115200);
    Serial.println("\nTVOC meter");

    FastLED.addLeds<NEOPIXEL, PIN_NEOPIXEL>(leds, NUM_LEDS); 

    // get ESP id
    sprintf(esp_id, "%08X", ESP.getChipId());
    Serial.print("ESP ID: ");
    Serial.println(esp_id);

    // setup CCS811
    pinMode(PIN_CCS811_GND, OUTPUT);
    digitalWrite(PIN_CCS811_GND, 0);
    pinMode(PIN_CCS811_WAK, OUTPUT);
    digitalWrite(PIN_CCS811_WAK, 0);
    Wire.begin(PIN_CCS811_SDA, PIN_CCS811_SCL);
    CCS811Core::status returnCode = ccs811.begin();
    if (returnCode != CCS811Core::SENSOR_SUCCESS) {
        Serial.println(".begin() returned with an error.");
        while (1);              //Hang if there was a problem.
    }

    // connect to wifi
    Serial.println("Starting WIFI manager ...");
    wifiManager.autoConnect("ESP-TVOC");
}

static void mqtt_send(const char *topic, const char *value)
{
    if (!mqttClient.connected()) {
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        mqttClient.connect(esp_id);
    }
    if (mqttClient.connected()) {
        Serial.print("Publishing ");
        Serial.print(value);
        Serial.print(" to ");
        Serial.print(topic);
        Serial.print("...");
        int result = mqttClient.publish(topic, value, true);
        Serial.println(result ? "OK" : "FAIL");
    }
}

static void show_on_led(uint16_t tvoc)
{
    int idx = 0;
    for (idx = 0; levels[idx].level > 0; idx++) {
        const level_t *l = &levels[idx];
        if (tvoc >= l->level) {
            leds[idx] = CRGB(l->color);
            Serial.print("#");
        } else {
            leds[idx] = CRGB::Black; 
            Serial.print(".");
        }
    }
    Serial.print(" ");
    Serial.println(tvoc);
	FastLED.show(); 
}

void loop(void)
{
    static uint16_t tvoc = 0;
    static unsigned long ms_prev = 0;

    // get time
    unsigned long ms = millis();

    // read if available
    if (ccs811.dataAvailable()) {
        ccs811.readAlgorithmResults();
        tvoc = ccs811.getTVOC();

        // update LEDs
        show_on_led(tvoc);
    }

    // report every LOG_PERIOD
    if ((ms - ms_prev) > LOG_PERIOD_SEC) {
        ms_prev = ms;

        // send over MQTT
        char message[16];
        snprintf(message, sizeof(message), "%d ppb", tvoc);
        mqtt_send(MQTT_TOPIC, message);
    }
    // keep MQTT alive
    mqttClient.loop();
}

