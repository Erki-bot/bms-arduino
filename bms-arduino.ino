#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>

// LCD I2C Configuration (adresse 0x27 ou 0x3F selon le modèle)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Définition des broches
#define SDA_PIN 4
#define SCL_PIN 5
#define batteryPin 7 // GPIO2 comme entrée analogique
#define optoPin 38
#define RXD2 17 // RX du HC-05
#define TXD2 18 // TX du HC-05

// Wifi
//  Wi-Fi Credentials
//  const char* ssid = "DESKTOP-L1AM7N1 1273";
//  const char* password = "308199AA";

const char *ssid = "Erki A54";
const char *password = "00949612";

// WiFi Led indicator
#define WIFI_NOT_CONNECTED_LED_PIN 19
#define WIFI_CONNECTED_LED_PIN 20
// MQTT Broker
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char *mqtt_data_topic = "ange/data/battery";
const char *mqtt_onduleur_topic = "ange/command/onduleur";

// Wi-Fi & MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Définition des bornes de tension pour la batterie 12V
const float maxVoltage12V = 14.4;
const float minVoltage12V = 10.5;

// Diviseur de tension
const float R1 = 10000.0;
const float R2 = 4700.0;

// Variables globales
bool smsEnvoye15 = false;
bool smsEnvoye100 = false;
bool modeManuel = false;      // False = Mode Auto, True = Mode Manuel
char commandeBluetooth = '3'; // Par défaut en mode automatique

// wifi task handler
TaskHandle_t wifiReconnectTaskHandle = NULL;

// MQTT task handler
TaskHandle_t MQTTReconnectTaskHandle = NULL;
// Led indicator handlers
TaskHandle_t ledIndicatorHandler = NULL;
TaskHandle_t blinkBlueLedHandler = NULL;
TaskHandle_t blinkBothLedsHandler = NULL;

void setup()
{

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Initialisation du Bluetooth HC-05
  connectToWifi();

  xTaskCreate(
      ledIndicatorTask,    // Task function
      "ledIndicatorTask",  // Task name
      2048,                // Stack size
      NULL,                // Task parameters
      4,                   // Priority
      &ledIndicatorHandler // Task handle
  );

  delay(200);
  if (ledIndicatorHandler)
    xTaskNotifyGive(ledIndicatorHandler);

  xTaskCreatePinnedToCore(
      WiFiReconnectTask,        // Task function
      "WiFiReconnectTask",      // Task name
      4096,                     // Stack size
      NULL,                     // Task parameters
      1,                        // Priority
      &wifiReconnectTaskHandle, // Task handle
      0);
  // Initialisation de l'I2C avec les pins spécifiés
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.begin(16, 2);
  lcd.backlight();

  pinMode(batteryPin, INPUT);
  pinMode(optoPin, OUTPUT);
  pinMode(WIFI_NOT_CONNECTED_LED_PIN, OUTPUT);
  pinMode(WIFI_CONNECTED_LED_PIN, OUTPUT);

  digitalWrite(WIFI_NOT_CONNECTED_LED_PIN, HIGH);
  digitalWrite(WIFI_CONNECTED_LED_PIN, HIGH);

  // xTaskCreatePinnedToCore(wifiHandler,"wifi-handler",2048,NULL,3,NULL,0);
  digitalWrite(optoPin, LOW); // Par défaut, la charge est coupée

  Serial.println("Système en mode automatique.");

  // digitalWrite(WIFI_CONNECTED_LED_PIN, LOW);
  // Serial.println("\nConnected to WiFi");
  client.setServer(mqtt_server, mqtt_port);
  xTaskCreate(
      MQTTConnectionTask,      // Task function
      "MQTTConnectionTask",    // Task name
      4096,                    // Stack size
      NULL,                    // Task parameters
      1,                       // Priority
      &MQTTReconnectTaskHandle // Task handle
  );
}

void MQTTConnectionTask(void *args)
{
  while (true)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (!client.connected())
      {
        if (ledIndicatorHandler)
          xTaskNotifyGive(ledIndicatorHandler);

        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32Client9782173139613"))
        {
          Serial.println("MQTT Connected");
          client.subscribe(mqtt_onduleur_topic);
          client.setCallback(callback);
          if (ledIndicatorHandler)
            xTaskNotifyGive(ledIndicatorHandler);
        }
        else
        {
          Serial.print("Failed, rc=");
          Serial.print(client.state());
          Serial.println(" retrying in 5 seconds...");
        }
      }
      client.loop(); // Maintain MQTT connection
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Avoid watchdog timeout, check every 500ms
  }
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client9782173139613"))
    {
      Serial.println("Connected");
      client.subscribe(mqtt_onduleur_topic);
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void loop()
{
  // if (!client.connected() && MQTTReconnectTaskHandle == NULL) {
  //     xTaskCreate(
  //     MQTTConnectionTask,      // Task function
  //     "MQTTConnectionTask",    // Task name
  //     4096,                   // Stack size
  //     NULL,                   // Task parameters
  //     1,                      // Priority
  //     &MQTTReconnectTaskHandle // Task handle
  // );
  // }else{
  //   client.loop();
  // }

  // Lire la tension de la batterie
  float sensorValue = analogRead(batteryPin);
  float voltage = sensorValue * (3.3 / 4095.0) * ((R1 + R2) / R2);
  float batteryPercentage = 0.0;

  if (voltage >= minVoltage12V && voltage <= maxVoltage12V)
  {
    batteryPercentage = (voltage - minVoltage12V) / (maxVoltage12V - minVoltage12V) * 100;
    if (voltage >= 14.4)
      batteryPercentage = 100;
  }
  else
  {
    batteryPercentage = -1;
  }

  // Affichage LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage: ");
  lcd.print(voltage, 2);
  lcd.setCursor(0, 1);
  if (batteryPercentage >= 0)
  {
    lcd.print("Battery: ");
    lcd.print(batteryPercentage, 1);
    lcd.print("%");
  }
  else
  {
    lcd.print("Battery not detected");
  }

  // Envoi des données MQTT
  char message[20];
  snprintf(message, sizeof(message), "%.2f,%.2f", voltage, batteryPercentage);
  if (client.connected())
    client.publish(mqtt_data_topic, message);

  // Mode Manuel via Bluetooth
  if (Serial2.available())
  {
    commandeBluetooth = Serial2.read(); // Lecture de la commande Bluetooth
    Serial.print("Commande reçue: ");
    Serial.println(commandeBluetooth);

    if (commandeBluetooth == '1')
    { // Activer la charge
      modeManuel = true;
      digitalWrite(optoPin, HIGH);
      Serial.println("🔌 Charge activée (Mode Manuel) !");
    }
    else if (commandeBluetooth == '0')
    { // Désactiver la charge
      modeManuel = true;
      digitalWrite(optoPin, LOW);
      Serial.println("❌ Charge désactivée (Mode Manuel) !");
    }
    else if (commandeBluetooth == '3')
    { // Revenir en mode automatique
      modeManuel = false;
      Serial.println("🔄 Mode automatique activé !");
    }
  }

  // Mode automatique si Bluetooth est désactivé (commande '3')
  if (!modeManuel)
  {
    if (voltage >= 14.4)
    {
      digitalWrite(optoPin, LOW); // Désactivation de la charge
      if (!smsEnvoye100)
      {
        Serial.println("⚡ Batterie pleine (100%). Charge coupée !");
        smsEnvoye100 = true;
        smsEnvoye15 = false;
      }
    }
    else if (batteryPercentage <= 15)
    {
      digitalWrite(optoPin, HIGH); // Activation de la charge
      if (!smsEnvoye15)
      {
        Serial.println("🔋 Batterie faible (≤15%). Charge activée !");
        smsEnvoye15 = true;
        smsEnvoye100 = false;
      }
    }
  }

  delay(2000); // Délai de 2 secondes avant la prochaine mise à jour
}

void WiFiReconnectTask(void *args)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for disconnection event

    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("Attempting to reconnect...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);

      int retryCount = 0;
      while (WiFi.status() != WL_CONNECTED && retryCount < 10)
      {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second between retries
        retryCount++;
      }

      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.println("Reconnected!");
        break;
      }
      else
      {
        Serial.println("Reconnect failed. Retrying later...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait before retrying
      }
    }
  }
}

void ledIndicatorTask(void *args)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification

    if (WiFi.status() == WL_CONNECTED)
    {
      if (client.connected())
      {
        // Blink blue LED
        if (blinkBothLedsHandler != NULL)
        {
          vTaskDelete(blinkBothLedsHandler);
          blinkBothLedsHandler = NULL; // Reset handle
        }

        if (blinkBlueLedHandler == NULL)
        { // Create task only if it doesn't exist
          if (xTaskCreate(
                  blinkBlueLedTask,    // Task function
                  "blinkBlueLedTask",  // Task name
                  2048,                // Increased stack size
                  NULL,                // Task parameters
                  5,                   // Priority
                  &blinkBlueLedHandler // Task handle
                  ) != pdPASS)
          {
            Serial.println("Failed to create blinkBlueLedTask");
            while (1)
              ; // Halt execution to prevent further issues
          }
        }
      }
      else
      {
        // Blink both LEDs
        if (blinkBlueLedHandler != NULL)
        {
          vTaskDelete(blinkBlueLedHandler);
          blinkBlueLedHandler = NULL; // Reset handle
        }

        if (blinkBothLedsHandler == NULL)
        { // Create task only if it doesn't exist
          if (xTaskCreate(
                  blinkBothLedsTask,    // Task function
                  "blinkBothLedsTask",  // Task name
                  2048,                 // Increased stack size
                  NULL,                 // Task parameters
                  5,                    // Priority
                  &blinkBothLedsHandler // Task handle
                  ) != pdPASS)
          {
            Serial.println("Failed to create blinkBothLedsTask");
            while (1)
              ; // Halt execution to prevent further issues
          }
        }
      }
    }
    else
    {
      // Turn off blue LED and turn on red LED
      WiFiDisconnectedLedState();

      // Ensure no blinking tasks are running
      if (blinkBlueLedHandler != NULL)
      {
        vTaskDelete(blinkBlueLedHandler);
        blinkBlueLedHandler = NULL;
      }
      if (blinkBothLedsHandler != NULL)
      {
        vTaskDelete(blinkBothLedsHandler);
        blinkBothLedsHandler = NULL;
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void blinkBothLedsTask(void *args)
{
  resetLedsState();
  while (1)
  {
    Serial.println("both");
    WiFiConnectedLedState();
    pause(500);
    WiFiDisconnectedLedState();
    pause(500);
  }
}

void blinkBlueLedTask(void *args)
{
  resetLedsState();
  while (1)
  {
    Serial.println("blue");
    digitalWrite(WIFI_CONNECTED_LED_PIN, LOW);
    pause(70);
    digitalWrite(WIFI_CONNECTED_LED_PIN, HIGH);
    pause(2000);
  }
}

void pause(int duration)
{
  vTaskDelay(duration / portTICK_PERIOD_MS);
}

void connectToWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent);
  delay(1000);
  WiFi.begin(ssid, password);
}

// Wi-Fi Event Handler
void WiFiEvent(WiFiEvent_t event)
{
  if (ledIndicatorHandler)
    xTaskNotifyGive(ledIndicatorHandler);
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    Serial.println("Wi-Fi connected.");
    // WiFiConnectedLedState();
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    // WiFiConnectedLedState();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    // WiFiDisconnectedLedState();
    Serial.println("Wi-Fi lost connection. Reconnecting...");
    xTaskNotifyGive(wifiReconnectTaskHandle); // Notify the reconnect task
    // xTaskCreatePinnedToCore(wifiHandler, "reconnect_to_wifi", configMINIMAL_STACK_SIZE * 3, NULL, 2, &wifi_reconnect_handler, 0);

    break;
  }
}

void WiFiConnectedLedState()
{
  digitalWrite(WIFI_NOT_CONNECTED_LED_PIN, HIGH);
  digitalWrite(WIFI_CONNECTED_LED_PIN, LOW);
}

void WiFiDisconnectedLedState()
{
  digitalWrite(WIFI_NOT_CONNECTED_LED_PIN, LOW);
  digitalWrite(WIFI_CONNECTED_LED_PIN, HIGH);
}

void resetLedsState()
{
  digitalWrite(WIFI_NOT_CONNECTED_LED_PIN, HIGH);
  digitalWrite(WIFI_CONNECTED_LED_PIN, HIGH);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message reçu sur le topic: ");
  Serial.println(topic);

  String message;
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.println("Message: " + message + " Longueur: " + length);
  if (message == "ON")
  {
    Serial.println("Mise en marche...");

    modeManuel = true;
    digitalWrite(optoPin, HIGH);
  }
  else if (message == "OFF")
  {
    Serial.println("Extinction...");
    digitalWrite(optoPin, LOW);
    modeManuel = true;
  }
}
