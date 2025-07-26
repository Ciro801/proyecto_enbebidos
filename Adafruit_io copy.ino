#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <HTTPClient.h>

// ===== CONFIGURACIÓN ADAFRUIT IO =====
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "ciro8014"        // ⚠️ CAMBIAR POR TU USERNAME
#define AIO_KEY         "mi_llave"        // ⚠️ CAMBIAR POR TU AIO KEY

// ===== CONFIGURACIÓN WEBHOOKS IFTTT =====
#define IFTTT_WEBHOOK_KEY "mi_otra llave"  // ⚠️ CAMBIAR POR TU WEBHOOK KEY

// ===== CONFIGURACIÓN DE RED =====
const char* ssid = "POCO X7 Pro";          // ⚠️ CAMBIAR POR TU WIFI
const char* password = "ciro8014";          // ⚠️ CAMBIAR POR TU PASSWORD

// ===== CONFIGURACIÓN DE PINES =====
#define DHT_PIN 4
#define DHT_TYPE DHT22
#define LED_PIN 12
#define SERVO_PAN_PIN 17
#define SERVO_TILT_PIN 18
#define CURRENT_SENSOR_PIN 32

// ===== CONFIGURACIÓN SENSOR DE CORRIENTE =====
const int NUM_SAMPLES = 1000;
const float ADC_REFERENCE = 3.3;
const int ADC_RESOLUTION = 4096;
const float VOLTAGE_DIVIDER = ADC_REFERENCE / ADC_RESOLUTION;
const float CT_TURNS_RATIO = 2000.0;
const float BURDEN_RESISTOR = 22.0;
const float VOLTAGE_CALIBRATION = 220.0;
const float CURRENT_CALIBRATION = 0.001;
const float NOISE_THRESHOLD = 0.001;

// ===== OBJETOS =====
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoPan;
Servo servoTilt;
WiFiClient espClient;
PubSubClient client(espClient);
BluetoothSerial SerialBT;

// ===== FEEDS DE ADAFRUIT IO =====
// Feeds de comandos (reciben de IFTTT)
const char* led_feed = AIO_USERNAME "/feeds/robot-led";
const char* servo_pan_feed = AIO_USERNAME "/feeds/robot-servo-pan";
const char* servo_tilt_feed = AIO_USERNAME "/feeds/robot-servo-tilt";
const char* command_feed = AIO_USERNAME "/feeds/robot-command";

// Feeds de sensores (envían a Adafruit IO)
const char* temperature_feed = AIO_USERNAME "/feeds/robot-temperature";
const char* humidity_feed = AIO_USERNAME "/feeds/robot-humidity";
const char* current_feed = AIO_USERNAME "/feeds/robot-current";
const char* power_feed = AIO_USERNAME "/feeds/robot-power";
const char* status_feed = AIO_USERNAME "/feeds/robot-status";

// ===== VARIABLES DE ESTADO =====
struct SensorData {
  float temperature;
  float humidity;
  float current;
  float power;
  int panPosition;
  int tiltPosition;
  int ledState; // 0=OFF, 1=ON, 2=BLINKING
};

SensorData sensorData = {0, 0, 0, 0, 90, 90, 0};

// ===== VARIABLES DE CONTROL =====
unsigned long lastSensorRead = 0;
unsigned long lastDataSend = 0;
unsigned long lastLedBlink = 0;
bool ledBlinkState = false;
float zeroCurrentOffset = 0;
bool isCalibrated = false;

void setup() {
  Serial.begin(115200);
  Serial.println("=== INICIANDO SISTEMA IOT CON ADAFRUIT IO ===");
  
  // ===== INICIALIZAR HARDWARE =====
  initializeHardware();
  
  // ===== CALIBRAR SENSOR DE CORRIENTE =====
  calibrateCurrentSensor();
  
  // ===== CONECTAR WIFI =====
  connectWiFi();
  
  // ===== CONFIGURAR ADAFRUIT IO =====
  client.setServer(AIO_SERVER, AIO_SERVERPORT);
  client.setCallback(adafruitCallback);
  
  // ===== CONECTAR A ADAFRUIT IO =====
  connectToAdafruitIO();
  
  // ===== INICIALIZAR BLUETOOTH =====
  SerialBT.begin("ESP32_Robot_Voice");
  
  // ===== CONFIGURAR ADC =====
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  Serial.println("=== SISTEMA LISTO PARA COMANDOS DE VOZ ===");
  Serial.println("Bluetooth: ESP32_Robot_Voice");
  Serial.println("Adafruit IO: Conectado");
  Serial.println("Comandos disponibles via Google Assistant + IFTTT");
  Serial.println("============================================");
}

void loop() {
  // ===== MANTENER CONEXIÓN ADAFRUIT IO =====
  if (!client.connected()) {
    connectToAdafruitIO();
  }
  client.loop();
  
  // ===== LEER SENSORES (cada 5 segundos) =====
  if (millis() - lastSensorRead > 5000) {
    readSensors();
    lastSensorRead = millis();
  }
  
  // ===== ENVIAR DATOS A ADAFRUIT IO (cada 10 segundos) =====
  if (millis() - lastDataSend > 10000) {
    sendDataToAdafruitIO();
    lastDataSend = millis();
  }
  
  // ===== CONTROLAR LED PARPADEANTE =====
  handleLedBlinking();
  
  // ===== PROCESAR COMANDOS BLUETOOTH =====
  handleBluetoothCommands();
  
  delay(50);
}

void initializeHardware() {
  // DHT22
  dht.begin();
  
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Servos
  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TILT_PIN);
  servoPan.write(90);
  servoTilt.write(90);
  
  Serial.println("✅ Hardware inicializado correctamente");
}

void calibrateCurrentSensor() {
  Serial.println("⚡ Calibrando sensor de corriente...");
  Serial.println("Asegúrate de que NO haya carga conectada");
  delay(3000);
  
  float sumVoltage = 0;
  int calibrationSamples = 5000;
  
  for (int i = 0; i < calibrationSamples; i++) {
    int rawADC = analogRead(CURRENT_SENSOR_PIN);
    float voltage = rawADC * VOLTAGE_DIVIDER;
    sumVoltage += voltage;
    delayMicroseconds(100);
  }
  
  zeroCurrentOffset = sumVoltage / calibrationSamples;
  isCalibrated = true;
  
  Serial.println("✅ Sensor calibrado - Offset: " + String(zeroCurrentOffset) + "V");
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("📶 Conectando a WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("✅ WiFi conectado!");
  Serial.println("📍 IP: " + WiFi.localIP().toString());
}

void connectToAdafruitIO() {
  int attempts = 0;
  while (!client.connected() && attempts < 5) {
    attempts++;
    Serial.print("🔌 Conectando a Adafruit IO (");
    Serial.print(attempts);
    Serial.print("/5)...");
    
    if (client.connect("ESP32_Robot", AIO_USERNAME, AIO_KEY)) {
      Serial.println(" ✅ Conectado!");
      
      // Suscribirse a feeds de comandos
      client.subscribe(led_feed);
      client.subscribe(servo_pan_feed);
      client.subscribe(servo_tilt_feed);
      client.subscribe(command_feed);
      
      Serial.println("📡 Suscrito a feeds de comandos");
      
      // Enviar estado inicial
      sendStatusUpdate();
      
    } else {
      Serial.print(" ❌ Error: ");
      Serial.println(client.state());
      delay(2000 * attempts);
    }
  }
  
  if (!client.connected()) {
    Serial.println("⚠️ No se pudo conectar a Adafruit IO");
    delay(30000);
  }
}

void adafruitCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("🎙️ Comando de voz recibido [" + String(topic) + "]: " + message);
  
  String topicStr = String(topic);
  
  // ===== CONTROL DE LED =====
  if (topicStr.indexOf("robot-led") != -1) {
    int ledValue = message.toInt();
    sensorData.ledState = ledValue;
    
    switch (ledValue) {
      case 0: // OFF
        digitalWrite(LED_PIN, LOW);
        Serial.println("💡 LED: OFF");
        break;
      case 1: // ON
        digitalWrite(LED_PIN, HIGH);
        Serial.println("💡 LED: ON");
        break;
      case 2: // BLINKING
        Serial.println("💡 LED: BLINKING");
        break;
    }
  }
  
  // ===== CONTROL SERVO PAN =====
  else if (topicStr.indexOf("robot-servo-pan") != -1) {
    int panValue = constrain(message.toInt(), 0, 180);
    servoPan.write(panValue);
    sensorData.panPosition = panValue;
    Serial.println("🔄 Servo Pan: " + String(panValue) + "°");
  }
  
  // ===== CONTROL SERVO TILT =====
  else if (topicStr.indexOf("robot-servo-tilt") != -1) {
    int tiltValue = constrain(message.toInt(), 0, 180);
    servoTilt.write(tiltValue);
    sensorData.tiltPosition = tiltValue;
    Serial.println("🔄 Servo Tilt: " + String(tiltValue) + "°");
  }
  
  // ===== COMANDOS COMPLEJOS =====
  else if (topicStr.indexOf("robot-command") != -1) {
    handleComplexCommand(message);
  }
}

void handleComplexCommand(String command) {
  command.toLowerCase();
  
  if (command == "center") {
    // Centrar cámara
    servoPan.write(90);
    servoTilt.write(90);
    sensorData.panPosition = 90;
    sensorData.tiltPosition = 90;
    Serial.println("📹 Cámara centrada");
  }
  else if (command == "up") {
    // Mover hacia arriba
    int newTilt = constrain(sensorData.tiltPosition + 30, 0, 180);
    servoTilt.write(newTilt);
    sensorData.tiltPosition = newTilt;
    Serial.println("⬆️ Cámara arriba: " + String(newTilt) + "°");
  }
  else if (command == "down") {
    // Mover hacia abajo
    int newTilt = constrain(sensorData.tiltPosition - 30, 0, 180);
    servoTilt.write(newTilt);
    sensorData.tiltPosition = newTilt;
    Serial.println("⬇️ Cámara abajo: " + String(newTilt) + "°");
  }
  else if (command == "left") {
    // Mover hacia izquierda
    int newPan = constrain(sensorData.panPosition - 30, 0, 180);
    servoPan.write(newPan);
    sensorData.panPosition = newPan;
    Serial.println("⬅️ Cámara izquierda: " + String(newPan) + "°");
  }
  else if (command == "right") {
    // Mover hacia derecha
    int newPan = constrain(sensorData.panPosition + 30, 0, 180);
    servoPan.write(newPan);
    sensorData.panPosition = newPan;
    Serial.println("➡️ Cámara derecha: " + String(newPan) + "°");
  }
  else if (command == "temperatura") {
    // Enviar temperatura via webhook
    sendWebhookNotification("temperatura", String(sensorData.temperature));
    Serial.println("🌡️ Temperatura solicitada via voz: " + String(sensorData.temperature) + "°C");
  }
  else if (command == "humedad") {
    // Enviar humedad via webhook
    sendWebhookNotification("humedad", String(sensorData.humidity));
    Serial.println("💧 Humedad solicitada via voz: " + String(sensorData.humidity) + "%");
  }
  else if (command == "consumo") {
    // Enviar datos de consumo via webhook
    String consumoData = String(sensorData.current) + "A, " + String(sensorData.power) + "W";
    sendWebhookNotification("consumo", consumoData);
    Serial.println("⚡ Consumo solicitado via voz: " + consumoData);
  }
  else if (command == "estado") {
    // Enviar estado completo via webhook
    String estadoCompleto = "T:" + String(sensorData.temperature) + "°C, H:" + 
                           String(sensorData.humidity) + "%, I:" + 
                           String(sensorData.current) + "A, P:" + 
                           String(sensorData.power) + "W";
    sendWebhookNotification("estado", estadoCompleto);
    Serial.println("📊 Estado completo solicitado via voz");
  }
}

void readSensors() {
  // ===== LEER DHT22 =====
  sensorData.temperature = dht.readTemperature();
  sensorData.humidity = dht.readHumidity();
  
  // ===== LEER SENSOR DE CORRIENTE =====
  float sumSquares = 0;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int rawADC = analogRead(CURRENT_SENSOR_PIN);
    float voltage = rawADC * VOLTAGE_DIVIDER;
    float centeredVoltage = voltage - zeroCurrentOffset;
    sumSquares += centeredVoltage * centeredVoltage;
    delayMicroseconds(100);
  }
  
  float rmsVoltage = sqrt(sumSquares / NUM_SAMPLES);
  float rmsCurrent = (rmsVoltage / BURDEN_RESISTOR) * CT_TURNS_RATIO;
  rmsCurrent *= CURRENT_CALIBRATION;
  
  if (rmsVoltage < 0.02 || rmsCurrent < NOISE_THRESHOLD) {
    rmsCurrent = 0.0;
  }
  
  sensorData.current = rmsCurrent;
  sensorData.power = rmsCurrent * VOLTAGE_CALIBRATION;
  
  // Debug
  Serial.println("📊 T:" + String(sensorData.temperature, 1) + 
                "°C H:" + String(sensorData.humidity, 1) + 
                "% I:" + String(sensorData.current, 3) + 
                "A P:" + String(sensorData.power, 2) + "W");
}

void sendDataToAdafruitIO() {
  // Enviar datos de sensores a Adafruit IO
  if (client.connected()) {
    client.publish(temperature_feed, String(sensorData.temperature).c_str());
    client.publish(humidity_feed, String(sensorData.humidity).c_str());
    client.publish(current_feed, String(sensorData.current).c_str());
    client.publish(power_feed, String(sensorData.power).c_str());
    
    Serial.println("📤 Datos enviados a Adafruit IO");
  }
}

void sendStatusUpdate() {
  String status = "online|" + String(millis()) + "|" + WiFi.localIP().toString();
  client.publish(status_feed, status.c_str());
  Serial.println("📋 Estado enviado a Adafruit IO");
}

void handleLedBlinking() {
  if (sensorData.ledState == 2) { // BLINKING
    if (millis() - lastLedBlink > 100) { // 100ms = 0.2Hz
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
      lastLedBlink = millis();
    }
  }
}

void handleBluetoothCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readString();
    command.trim();
    
    Serial.println("📱 Comando Bluetooth: " + command);
    
    if (command == "STATUS") {
      sendBluetoothStatus();
    } else if (command == "CALIBRATE") {
      calibrateCurrentSensor();
    } else {
      // Procesar comandos de control
      if (command.startsWith("LED:")) {
        int ledValue = command.substring(4).toInt();
        sensorData.ledState = ledValue;
        handleLedCommand(ledValue);
      }
    }
  }
}

void handleLedCommand(int ledValue) {
  switch (ledValue) {
    case 0:
      digitalWrite(LED_PIN, LOW);
      break;
    case 1:
      digitalWrite(LED_PIN, HIGH);
      break;
    case 2:
      // Blinking se maneja en handleLedBlinking()
      break;
  }
}

void sendBluetoothStatus() {
  String status = "T:" + String(sensorData.temperature) + 
                 ",H:" + String(sensorData.humidity) + 
                 ",I:" + String(sensorData.current) + 
                 ",P:" + String(sensorData.power) + 
                 ",PAN:" + String(sensorData.panPosition) + 
                 ",TILT:" + String(sensorData.tiltPosition) + 
                 ",LED:" + String(sensorData.ledState);
  
  SerialBT.println(status);
}

void sendWebhookNotification(String evento, String valor) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("http://maker.ifttt.com/trigger/" + evento + "_robot/with/key/" + String(IFTTT_WEBHOOK_KEY));
    http.addHeader("Content-Type", "application/json");
    
    // Crear JSON con los datos
    String jsonData = "{\"value1\":\"" + valor + "\",\"value2\":\"" + WiFi.localIP().toString() + "\"}";
    
    int httpResponseCode = http.POST(jsonData);
    
    if (httpResponseCode > 0) {
      Serial.println("📤 Webhook enviado: " + evento + " = " + valor);
    } else {
      Serial.println("❌ Error webhook: " + String(httpResponseCode));
    }
    
    http.end();
  }
}