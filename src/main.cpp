#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AutoConnect.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

#include "SparkFunBME280.h"

/**
 * TODO:
 *  fix duty cycle increase/decrease to make it work without ±10
 *  mqtt support
 */

// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA {
    float temp;
    float humidity;
    float pressure;
} sensorData;

BME280 bme280;

WebServer server(80);
AutoConnect Portal(server);

/**
 * Current interrupt count
 */
volatile int interruptCounter = 0;
volatile int interruptCounter_fan2 = 0;
volatile int interruptCounter_fan3 = 0;
volatile int interruptCounter_fan4 = 0;

/**
 * Interrupts in the last `fan_tacho_read_interval_ms` interval.
 * 
 * This value is doubled as the fans send an interrupt every half-revolution
 */
int last_RPS = 0;
int last_RPS_fan2 = 0;
int last_RPS_fan3 = 0;
int last_RPS_fan4 = 0;

/* Current Fan duty cycle [0-255] */
int fan_pwm_duty_cycle = 0;

/* Fan duty cycle target [0-255] */
int fan_pwm_duty_cycle_target = 0;

/**
 * Timings
 */

/**
 * When was the value last checkd
 */
int last_fan_tacho_read_ms = 0;
int last_fan_duty_check_ms = 0;
int last_temp_check_ms = 0;

/* How often to check the values, in milliseconds */
int fan_duty_check_interval_ms = 1000;
int fan_tacho_read_interval_ms = 1000;
int temp_check_interval_ms = 5000;

/* Mutex to work with interrupts */
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux3 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux4 = portMUX_INITIALIZER_UNLOCKED;

/**
 * Fan Tacho read interrupt handler
 */
void IRAM_ATTR handleInterrupt() {
    portENTER_CRITICAL_ISR(&mux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR handleInterrupt_fan2() {
    portENTER_CRITICAL_ISR(&mux2);
    interruptCounter_fan2++;
    portEXIT_CRITICAL_ISR(&mux2);
}

void IRAM_ATTR handleInterrupt_fan3() {
    portENTER_CRITICAL_ISR(&mux3);
    interruptCounter_fan3++;
    portEXIT_CRITICAL_ISR(&mux3);
}

void IRAM_ATTR handleInterrupt_fan4() {
    portENTER_CRITICAL_ISR(&mux4);
    interruptCounter_fan4++;
    portEXIT_CRITICAL_ISR(&mux4);
}

/**
 * Resets the interrupt counter
 */
void resetInterrupt() {
    portENTER_CRITICAL_ISR(&mux);
    last_RPS = interruptCounter; // Store the last PRS value before resetting the counter
    interruptCounter = 0;
    portEXIT_CRITICAL_ISR(&mux);

    portENTER_CRITICAL_ISR(&mux2);
    last_RPS_fan2 = interruptCounter_fan2; // Store the last PRS value before resetting the counter
    interruptCounter_fan2 = 0;
    portEXIT_CRITICAL_ISR(&mux2);

    portENTER_CRITICAL_ISR(&mux3);
    last_RPS_fan3 = interruptCounter_fan3; // Store the last PRS value before resetting the counter
    interruptCounter_fan3 = 0;
    portEXIT_CRITICAL_ISR(&mux3);

    portENTER_CRITICAL_ISR(&mux4);
    last_RPS_fan4 = interruptCounter_fan4; // Store the last PRS value before resetting the counter
    interruptCounter_fan4 = 0;
    portEXIT_CRITICAL_ISR(&mux4);
}

String prepare_data_output() {
    char buffer[512];
    String out_string = (""
                         "<html>"
                         "<head>"
                         "<meta http-equiv=\"refresh\" content=\"5\">"
                         "</head>"
                         "<body>"
                         " Duty:     %d <br/>"
                         " Target:   %d <br/>"
                         " RPS:      %d <br/>"
                         " RPM:      %d <br/>"
                         " RPS 2:      %d <br/>"
                         " RPM 2:      %d <br/>"
                         " RPS 3:      %d <br/>"
                         " RPM 3:      %d <br/>"
                         " RPS 4:      %d <br/>"
                         " RPM 4:      %d <br/>"
                         " Temp:     %.2f <br/>"
                         " Humidity: %.2f <br/>"
                         " Pressure: %.2f <br/>"
                         "</body>"
                         "</html>");

    sprintf(
        buffer,
        out_string.c_str(),
        fan_pwm_duty_cycle,
        fan_pwm_duty_cycle_target,
        last_RPS / 2,
        (last_RPS / 2) * 60,
        last_RPS_fan2 / 2,
        (last_RPS_fan2 / 2) * 60,
        last_RPS_fan3 / 2,
        (last_RPS_fan3 / 2) * 60,
        last_RPS_fan4 / 2,
        (last_RPS_fan4 / 2) * 60,
        sensorData.temp,
        sensorData.humidity,
        sensorData.pressure);

    return (String)buffer;
}

/**
 * Multithreading options
 */

static uint8_t manager_core = 0;
static uint8_t fan_controller_core = 1;

void manage_core_task(void *pv_parameters) {
    if (MDNS.begin("esp32")) {
        Serial.println("MDNS responder started");
    }

    server.on("/", []() {
        server.send(200, "text/plain", "Hello from the ESP32 Fan Controller!");
    });

    server.on("/status", []() {
        server.send(200, "text/html", prepare_data_output());
    });

    server.on("/target/{}", []() {
        String target = server.pathArg(0);
        fan_pwm_duty_cycle_target = target.toInt();

        server.send(200, "text/html", prepare_data_output());
    });

    Portal.begin();

    Serial.println("HTTP server started");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
    });

    ArduinoOTA.setPort(8277);
    ArduinoOTA.begin();

    /**
	 * Multiple board versions were built and some of them have the I2C pins set differently than the others.
	 * 
	 * This code is supposed to fix that :) 
	 * 
	 * Variant1 pins: 21,22
	 * Variant2 pins: 15, 2
	 *  
	 * To detect - Begin, set address, read, if it's successful - OK, if it's not - try the other one
	 */
    bme280.setI2CAddress(0x76);

    Wire.begin(15, 2);
    if (bme280.beginI2C() == false) {
        Serial.println("Temp sensor not on 15,2 pins, attempting 21,22");
        Wire.begin(21, 22);
        if (bme280.beginI2C() == false) {
            Serial.println("Unable to find Temperature sensor :(");
        }
    }

    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            fan_pwm_duty_cycle = 180;
        }

        ArduinoOTA.handle();
        Portal.handleClient();

        double current_millis = millis();

        if (current_millis >= last_fan_tacho_read_ms + fan_tacho_read_interval_ms) {
            resetInterrupt();
            last_fan_tacho_read_ms = current_millis;
        }

        if (current_millis >= last_fan_duty_check_ms + fan_duty_check_interval_ms) {
            if (fan_pwm_duty_cycle < fan_pwm_duty_cycle_target) {
                fan_pwm_duty_cycle += 10;
            } else if (fan_pwm_duty_cycle > fan_pwm_duty_cycle_target) {
                fan_pwm_duty_cycle -= 10;
            }

            last_fan_duty_check_ms = current_millis;
        }

        // Temperature reading every 5 seconds

        if (current_millis >= last_temp_check_ms + temp_check_interval_ms) {
            if (bme280.beginI2C()) {
                bme280.setMode(MODE_FORCED);

                while (bme280.isMeasuring() == false)
                    yield(); //Wait for sensor to start measurment
                while (bme280.isMeasuring() == true)
                    yield(); //Hang out while sensor completes the reading

                sensorData.temp = bme280.readTempC();
                sensorData.humidity = bme280.readFloatHumidity();
                sensorData.pressure = bme280.readFloatPressure() / 100.0;

                Serial.println("TEMPS");
                Serial.println(sensorData.temp);
                Serial.println(sensorData.humidity);
                Serial.println(sensorData.pressure);

                fan_pwm_duty_cycle_target = min(max((int)map(sensorData.temp, 30, 45, 0, 250), 0), 250);
            } else {
                fan_pwm_duty_cycle_target = 100;
            }

            last_temp_check_ms = current_millis;
        }
    }
}

void fan_controller_task(void *pv_parameters) {
    Serial.println("Starting FAN controller task");

    /**
	 * PWM Setup
	 */
    int freq = 25000;
    int fan_pwm_channel = 0;
    int resolution = 8;

    /* Fan 1 */
    int fan_pwm_pin = 16;
    int fan_tacho_pin = 27;

    /* Fan 2 */
    int fan2_pwm_pin = 17;
    int fan2_tacho_pin = 26;

    /* Fan 3 */
    int fan3_pwm_pin = 18;
    int fan3_tacho_pin = 25;

    /* Fan 4 */
    int fan4_pwm_pin = 19;
    int fan4_tacho_pin = 33;

    pinMode(fan_pwm_pin, OUTPUT);
    pinMode(fan2_pwm_pin, OUTPUT);
    pinMode(fan3_pwm_pin, OUTPUT);
    pinMode(fan4_pwm_pin, OUTPUT);

    pinMode(fan_tacho_pin, INPUT_PULLUP);
    pinMode(fan2_tacho_pin, INPUT_PULLUP);
    pinMode(fan3_tacho_pin, INPUT_PULLUP);
    pinMode(fan4_tacho_pin, INPUT_PULLUP);

    // Setup PWM control
    ledcSetup(fan_pwm_channel, freq, resolution);

    ledcAttachPin(fan_pwm_pin, fan_pwm_channel);
    ledcAttachPin(fan2_pwm_pin, fan_pwm_channel);
    ledcAttachPin(fan3_pwm_pin, fan_pwm_channel);
    ledcAttachPin(fan4_pwm_pin, fan_pwm_channel);

    // Setup fan RPM readout
    attachInterrupt(digitalPinToInterrupt(fan_tacho_pin), handleInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(fan2_tacho_pin), handleInterrupt_fan2, FALLING);
    attachInterrupt(digitalPinToInterrupt(fan3_tacho_pin), handleInterrupt_fan3, FALLING);
    attachInterrupt(digitalPinToInterrupt(fan4_tacho_pin), handleInterrupt_fan4, FALLING);

    for (;;) {
        ledcWrite(fan_pwm_channel, fan_pwm_duty_cycle);
    }
}

TaskHandle_t Manager_Task;
TaskHandle_t Fan_Controller_Task;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Monitoring interrupts: ");

    xTaskCreatePinnedToCore(manage_core_task, "ManageTask", 10000, NULL, 0, &Manager_Task, manager_core);
    xTaskCreatePinnedToCore(fan_controller_task, "ControllerTask", 10000, NULL, 10, &Fan_Controller_Task, fan_controller_core);
}

void loop() {
}