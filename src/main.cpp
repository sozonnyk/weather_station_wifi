#include <secrets.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ModbusMaster.h>
#include <LTR390.h>
#include <ArduinoHA.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define WIND_UART  2
#define RAIN_INPUT 2
#define DWELL_TIME 100 //ms
#define RAIN_FACTOR 0.28 //mm per pulse
#define INTERRUPT_DELAY_MS 100 //Ignore interrupts for the time
#define WIFI_WAIT_TIME_MS 10000
#define TZ_DEF "AEST-10AEDT,M10.1.0,M4.1.0/3"
#define NTP_SERVER "time.google.com"

volatile float rainDayValue = 0;
volatile float rainHourValue = 0;
volatile unsigned long lastInterrupt = 0;

int previousMinute = -1;
int previousHour = -1;
int previousDay = -1;

AsyncWebServer server(80);

HardwareSerial windSerial(WIND_UART);
ModbusMaster mainMeterNode;
LTR390 ltr390(0x53);

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device, 7);

HASensorNumber sunLightLux("sun_light_lux", HASensorNumber::PrecisionP0);
HASensorNumber sunLightUv("sun_light_uv", HASensorNumber::PrecisionP0);
HASensorNumber wind("wind", HASensorNumber::PrecisionP1);
HASensorNumber rain_day("rain_day", HASensorNumber::PrecisionP2);
HASensorNumber rain_hour("rain_hour", HASensorNumber::PrecisionP2);
HASensorNumber rssi("rssi", HASensorNumber::PrecisionP0);

float getWind() {
	uint16_t startAddress = 0x0000;
	uint8_t length = 2;
	uint16_t result;
	result = mainMeterNode.readInputRegisters(startAddress, length);
	if (result == mainMeterNode.ku8MBSuccess) {
		return mainMeterNode.getResponseBuffer(0) * 0.36;
	} else {
		return 0;
	}
}

float getUvi() {
	ltr390.setGain(LTR390_GAIN_18);   // Sensitivity reduction is nessesary
	ltr390.setResolution(LTR390_RESOLUTION_20BIT);
	ltr390.setMode(LTR390_MODE_UVS);

	do {
		delay(DWELL_TIME * 5);
	} while (!ltr390.newDataAvailable());

	float uvi = ltr390.readUVS(); //ltr390.getUVI();
	return uvi;
}

float getLux() {
	ltr390.setGain(LTR390_GAIN_1); // Overflow in full sun if >1
	ltr390.setResolution(LTR390_RESOLUTION_20BIT);
	ltr390.setMode(LTR390_MODE_ALS);

	do {
		delay(DWELL_TIME * 5);
	} while (!ltr390.newDataAvailable());

	float lux = ltr390.getLux();
	return lux;

}

void IRAM_ATTR onRainPulse() {
	unsigned long currentTime = micros() / 1000;
	if (currentTime < lastInterrupt)
		lastInterrupt = -1;
	if (currentTime < lastInterrupt + INTERRUPT_DELAY_MS)
		return;
	lastInterrupt = currentTime;
	rainDayValue += RAIN_FACTOR;
	rainHourValue += RAIN_FACTOR;
}

void initWifi() {
	WiFi.setHostname("weatherstation");
	WiFi.mode(WIFI_STA);
	WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
	WiFi.begin(WIFI_SSID, WIFI_PASSWD);
	while (WiFi.waitForConnectResult(WIFI_WAIT_TIME_MS) != WL_CONNECTED) {
		Serial.println("WiFi failed. Rebooting...");
		ESP.restart();
	}
	Serial.println("WiFi connected");
	Serial.print("IP: ");
	Serial.println(WiFi.localIP());
}

void initTime() {
	configTime(0, 0, NTP_SERVER);
	setenv("TZ", TZ_DEF, 1);
	tzset();
}

void initOta() {
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(200, "text/plain", "Weather station");
	});

	AsyncElegantOTA.begin(&server);
	server.begin();
}

void setup() {
	Serial.begin(115200);
	Serial.println("Start");

	windSerial.begin(4800);
	mainMeterNode.begin(1, windSerial);
	Wire.begin();
	ltr390.init();

	pinMode(RAIN_INPUT, INPUT);
	attachInterrupt(digitalPinToInterrupt(RAIN_INPUT), onRainPulse, RISING);

	initWifi();
	initTime();
	initOta();

	byte mac[6];
	WiFi.macAddress(mac);
	device.setUniqueId(mac, sizeof(mac));
	device.setName("Weather Station");
	device.setModel("Weather Station");
	device.setSoftwareVersion("1.0.0");
	device.setManufacturer("Andrew");

	sunLightLux.setIcon("mdi:weather-sunny");
	sunLightLux.setName("Brightness");
	sunLightLux.setUnitOfMeasurement("Lux");

	sunLightUv.setIcon("mdi:sun-wireless-outline");
	sunLightUv.setName("UV Index");
	sunLightUv.setUnitOfMeasurement("UVI");

	rssi.setIcon("mdi:wifi");
	rssi.setName("RSSI");
	rssi.setUnitOfMeasurement("RSSI");

	wind.setIcon("mdi:weather-windy");
	wind.setName("Wind");
	wind.setUnitOfMeasurement("kmh");

	rain_day.setIcon("mdi:weather-pouring");
	rain_day.setName("Rain Daily");
	rain_day.setUnitOfMeasurement("mm");

	rain_hour.setIcon("mdi:weather-pouring");
	rain_hour.setName("Rain Hourly");
	rain_hour.setUnitOfMeasurement("mm");

	mqtt.begin(MQTT_HOST, MQTT_USER, MQTT_PASSWD);
}

void loop() {
	struct tm time;
	getLocalTime(&time);

	if (time.tm_min != previousMinute) {
		previousMinute = time.tm_min;
		Serial.println(&time, "%A, %B %d %Y %H:%M:%S");

		if (WiFi.status() != WL_CONNECTED) {
			Serial.println("WiFi lost");
			WiFi.disconnect();
			WiFi.reconnect();
			if (WiFi.waitForConnectResult(WIFI_WAIT_TIME_MS) != WL_CONNECTED) {
				Serial.println("Unable to connect WiFi");
				return;
			}
		}

		if (time.tm_hour != previousHour) {
			previousHour = time.tm_hour;
			rainHourValue = 0;
		}

		if (time.tm_mday != previousDay) {
			previousDay = time.tm_mday;
			rainDayValue = 0;
		}

		float windValue = getWind();
		float luxValue = getLux();
		float uvValue = getUvi();
		int8_t rssiValue = WiFi.RSSI();

		Serial.print("Wind raw ");
		Serial.println(windValue);

		Serial.print("Lux raw ");
		Serial.println(luxValue);

		Serial.print("UV raw ");
		Serial.println(uvValue);

		Serial.print("Rain Day: ");
		Serial.println(rainDayValue);

		Serial.print("Rain Hour: ");
		Serial.println(rainHourValue);

		Serial.print("RSSI: ");
		Serial.println(rssiValue);

		Serial.println();

		sunLightLux.setValue(luxValue);
		sunLightUv.setValue(uvValue);
		wind.setValue(windValue);
		rssi.setValue(rssiValue);
		rain_day.setValue(rainDayValue);
		rain_hour.setValue(rainHourValue);
	}

	mqtt.loop();
}
