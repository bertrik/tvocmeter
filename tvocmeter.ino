#include <Arduino.h>

#include <Wire.h>
#include <SparkFunCCS811.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#define PIN_CCS811_WAK  D2
#define PIN_CCS811_SDA  D3
#define PIN_CCS811_SCL  D4

#define CCS811_ADDR 0x5A

#define MQTT_HOST   "mosquitto.space.revspace.nl"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "revspace/sensors/tvoc"

#define LOG_PERIOD_SEC  10

static CCS811 ccs811(CCS811_ADDR);
static char esp_id[16];

static WiFiManager wifiManager;
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);

void setup(void)
{
    Serial.begin(115200);
    Serial.println("\nTVOC meter");

    // get ESP id
    sprintf(esp_id, "%08X", ESP.getChipId());
    Serial.print("ESP ID: ");
    Serial.println(esp_id);

    // setup CCS811
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

void loop()
{
    static unsigned long ms_prev = 0;
    unsigned long ms = millis();

    // report every LOG_PERIOD
    if ((ms - ms_prev) > LOG_PERIOD_SEC) {
        ms_prev = ms;

        if (ccs811.dataAvailable()) {
            ccs811.readAlgorithmResults();
            uint16_t tvoc = ccs811.getTVOC();

            // send over MQTT
            char message[16];
            snprintf(message, sizeof(message), "%d ppb", tvoc);
            mqtt_send(MQTT_TOPIC, message);
        }
    }
    // keep MQTT alive
    mqttClient.loop();
}

