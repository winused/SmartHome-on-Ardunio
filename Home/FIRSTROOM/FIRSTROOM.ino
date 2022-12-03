
#define LIGHT_SENSOR_PIN 18 

const int motionSensor = 23; // PIR Motion Sensor
bool motionDetected = false;
const int ledPin = 22;
int nTimer = 0;

#include <WiFi.h>
#include <PubSubClient.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
 
const char* ssid = "FitbitLab";
const char* password = "13578077";
const char* mqttServer = "broker.mqtt-dashboard.com";
const int mqttPort = 1883;

char temp[5];
String tempStr;
int tempInt;

int LEDLow = 25;
int LEDNice = 33;
int LEDHigh = 32;

WiFiClient espClient;
PubSubClient client(espClient);
 
void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
 
        temp[i] = (char)payload[i];
      
  }
  
  tempStr = String(temp);
  tempInt = tempStr.toInt();
  Serial.println("Temp from tempInt: ");
  Serial.println(tempInt);
  delay(500);

  if (tempInt <= 15) {
    
    Serial.println(tempInt);
    digitalWrite(LEDHigh, LOW);
    digitalWrite(LEDNice, LOW);
    digitalWrite(LEDLow, HIGH);
  }
  else if (tempInt > 15 && tempInt <= 28) {
    digitalWrite(LEDHigh, LOW);
    digitalWrite(LEDNice, HIGH);
    digitalWrite(LEDLow, LOW);
  }
  else {
    digitalWrite(LEDHigh, HIGH);
    digitalWrite(LEDNice, LOW);
    digitalWrite(LEDLow, LOW);
  }
 
  Serial.println();
  Serial.println("-----------------------");
 
}

// Indicates when motion is detected
void IRAM_ATTR detectsMovement() {
  //Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
}

// Reconnect
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      client.subscribe("esp32dhttemperature");
      //client.subscribe("esp32dhthumidity");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Setup
void setup() {
  
  Serial.begin(115200);

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  pinMode(ledPin, OUTPUT);

 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client")) {
 
      Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }

  pinMode(LEDLow, OUTPUT);
  pinMode(LEDNice, OUTPUT);
  pinMode(LEDHigh, OUTPUT);  
   
  client.subscribe("esp32dhttemperature");
  //client.subscribe("esp32dhthumidity");

  delay(500);

  digitalWrite(LEDHigh, LOW);
  digitalWrite(LEDNice, LOW);
  digitalWrite(LEDLow, LOW);
 
}


// Loop
void loop() {

  int analogValue = analogRead(LIGHT_SENSOR_PIN);

  Serial.print("Analog Value = ");
  Serial.print(analogValue);   // the raw analog reading

  // We'll have a few threshholds, qualitatively determined
  if (analogValue < 800) 
  {
    Serial.println(" => Dark/Dim");
    
    if(motionDetected)
    {
        Serial.println("Motion Detected");
        motionDetected = false;

        digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level 
    }

    if (nTimer > 1000) {
      digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
      nTimer = 0;
      
    }

  delay(10);
  nTimer++;
 } 

 else {
    Serial.println(" => Light/Bright");
  }
    delay(5000);

  
  
  

    if (!client.connected()) {
      reconnect();
      Serial.println("Reconnect");
    }
    client.loop();

}
