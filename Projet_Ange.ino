#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>

// LCD I2C Configuration (adresse 0x27 ou 0x3F selon le modèle)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Définition des broches
#define SDA_PIN 4
#define SCL_PIN 5
#define batteryPin 7  // GPIO2 comme entrée analogique
#define optoPin 38
#define RXD2 17  // RX du HC-05
#define TXD2 18  // TX du HC-05

// Wi-Fi Credentials
const char* ssid = "DESKTOP-L1AM7N1 1273";
const char* password = "308199AA";

// MQTT Broker
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_topic = "ange/data/battery";

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
bool modeManuel = false;  // False = Mode Auto, True = Mode Manuel
char commandeBluetooth = '3'; // Par défaut en mode automatique

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Initialisation du Bluetooth HC-05
    WiFi.begin(ssid, password);

    // Initialisation de l'I2C avec les pins spécifiés
    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.begin(16, 2);
    lcd.backlight();

    pinMode(batteryPin, INPUT);
    pinMode(optoPin, OUTPUT);
    digitalWrite(optoPin, LOW); // Par défaut, la charge est coupée

    Serial.println("Système en mode automatique.");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    client.setServer(mqtt_server, mqtt_port);
    reconnect();
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32Client9782173139613")) {
            Serial.println("Connected");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Lire la tension de la batterie
    float sensorValue = analogRead(batteryPin);  
    float voltage = sensorValue * (3.3 / 4095.0) * ((R1 + R2) / R2);
    float batteryPercentage = 0.0;

    if (voltage >= minVoltage12V && voltage <= maxVoltage12V) {
        batteryPercentage = (voltage - minVoltage12V) / (maxVoltage12V - minVoltage12V) * 100;
        if (voltage >= 14.4) batteryPercentage = 100;
    } else {
        batteryPercentage = -1;
    }

    // Affichage LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Voltage: ");
    lcd.print(voltage, 2);
    lcd.setCursor(0, 1);
    if (batteryPercentage >= 0) {
        lcd.print("Battery: ");
        lcd.print(batteryPercentage, 1);
        lcd.print("%");
    } else {
        lcd.print("Battery not detected");
    }

    // Envoi des données MQTT
    char message[20];
    snprintf(message, sizeof(message), "%.2f,%.2f", voltage, batteryPercentage);
    client.publish(mqtt_topic, message);

    // Mode Manuel via Bluetooth
    if (Serial2.available()) {
        commandeBluetooth = Serial2.read(); // Lecture de la commande Bluetooth
        Serial.print("Commande reçue: ");
        Serial.println(commandeBluetooth);

        if (commandeBluetooth == '1') {  // Activer la charge
            modeManuel = true;
            digitalWrite(optoPin, HIGH);
            Serial.println("🔌 Charge activée (Mode Manuel) !");
        } 
        else if (commandeBluetooth == '0') {  // Désactiver la charge
            modeManuel = true;
            digitalWrite(optoPin, LOW);
            Serial.println("❌ Charge désactivée (Mode Manuel) !");
        } 
        else if (commandeBluetooth == '3') {  // Revenir en mode automatique
            modeManuel = false;
            Serial.println("🔄 Mode automatique activé !");
        }
    }

    // Mode automatique si Bluetooth est désactivé (commande '3')
    if (!modeManuel) {
        if (voltage >= 14.4) {
            digitalWrite(optoPin, LOW); // Désactivation de la charge
            if (!smsEnvoye100) {
                Serial.println("⚡ Batterie pleine (100%). Charge coupée !");
                smsEnvoye100 = true;
                smsEnvoye15 = false;
            }
        } else if (batteryPercentage <= 15) {
            digitalWrite(optoPin, HIGH); // Activation de la charge
            if (!smsEnvoye15) {
                Serial.println("🔋 Batterie faible (≤15%). Charge activée !");
                smsEnvoye15 = true;
                smsEnvoye100 = false;
            }
        }
    }

    delay(2000); // Délai de 2 secondes avant la prochaine mise à jour
}