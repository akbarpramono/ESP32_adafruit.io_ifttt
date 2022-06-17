#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <SimpleDHT.h>

#define WLAN_SSID       "DigitakHQ"
#define WLAN_PASS       "P4stiBis4"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define IO_USERNAME  "akbarpramono"
#define IO_KEY       "aio_GSCV968Cy9ZsMITcSPi1dVA8mTv2"


WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/photocell");

Adafruit_MQTT_Subscribe Kontroll_Relay = Adafruit_MQTT_Subscribe(&mqtt, IO_USERNAME "/feeds/kontrol-relay");
Adafruit_MQTT_Subscribe Kontroll_Relay2 = Adafruit_MQTT_Subscribe(&mqtt, IO_USERNAME "/feeds/kontrol-relay-2");
Adafruit_MQTT_Subscribe Kontroll_Relay3 = Adafruit_MQTT_Subscribe(&mqtt, IO_USERNAME "/feeds/kontrol-relay-3");
Adafruit_MQTT_Subscribe Kontroll_Relay4 = Adafruit_MQTT_Subscribe(&mqtt, IO_USERNAME "/feeds/kontrol-relay-4");
Adafruit_MQTT_Publish Kontroll_Temp = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/suhu");
Adafruit_MQTT_Publish Kontroll_Temp2 = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/welcome-feed");

void MQTT_connect();

#define pin_relay1 2
#define pin_relay2 13
#define pin_relay3 25
#define pin_relay4 26 
int pinDHT11 = 14;    // Connect your DHT22 data pin here!
SimpleDHT11 dht11(pinDHT11);
byte hum = 0;  //Stores humidity value
byte temp = 0; //Stores temperature value


String Data;

void setup() {
  Serial.begin(115200);
  delay(10);
  
  //dht.begin();
//  Serial.println(F("Kontrol Relay Google Assistant"));

  // Connect to WiFi access point.
//  Serial.println(); Serial.println();
//  Serial.print("Connecting to ");
//  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }
//  Serial.println();

//  Serial.println("WiFi connected");
//  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&Kontroll_Relay);
  pinMode (pin_relay1, OUTPUT);
  digitalWrite (pin_relay1, 1); 
  
  mqtt.subscribe(&Kontroll_Relay2);
  pinMode (pin_relay2, OUTPUT);
  digitalWrite (pin_relay2, 1);
  
  mqtt.subscribe(&Kontroll_Relay3);
  pinMode (pin_relay3, OUTPUT);
  digitalWrite (pin_relay3, 1);
  
  mqtt.subscribe(&Kontroll_Relay4);
  pinMode (pin_relay4, OUTPUT);
  digitalWrite (pin_relay4, 1); 
  
  connect();
  
//  mqtt.publish(&Kontroll_Temp);
//  pinMode (pin_Temp, OUTPUT);
//  digitalWrite (pin_Temp, 1);
//
//  mqtt.publish(&Kontroll_Temp2);
//  pinMode (pin_Temp, OUTPUT);
//  digitalWrite (pin_Temp, 1);
}

uint32_t x=0;
void connect()
{
//  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0)
  {
    switch (ret)
    {
//      case 1: Serial.println(F("Wrong protocol")); break;
//      case 2: Serial.println(F("ID rejected")); break;
//      case 3: Serial.println(F("Server unavail")); break;
//      case 4: Serial.println(F("Bad user/pass")); break;
//      case 5: Serial.println(F("Not authed")); break;
//      case 6: Serial.println(F("Failed to subscribe")); break;
//      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
//    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
//  Serial.println(F("Adafruit IO Connected!"));
}
void loop() {
   MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(2000))) 
  {
    if (subscription == &Kontroll_Relay) {
      Data = (char *)Kontroll_Relay.lastread;
//      Serial.println(Data);
      if (Data == "OFF1"){
        digitalWrite (pin_relay1, 1);
      }
      else if (Data == "ON1"){
        digitalWrite(pin_relay1, 0);
      }
    }
    if (subscription == &Kontroll_Relay2) {
      Data = (char *)Kontroll_Relay2.lastread;
//      Serial.println(Data);
      if (Data == "OFF2"){      
        digitalWrite (pin_relay2, 1);
      }
      else if (Data == "ON2"){        
        digitalWrite(pin_relay2, 0);
      }
    }
    if (subscription == &Kontroll_Relay3) {
      Data = (char *)Kontroll_Relay3.lastread;   
//      Serial.println(Data);
      if (Data == "OFF3"){
        digitalWrite (pin_relay3, 1);
      }
      else if (Data == "ON3"){
        digitalWrite(pin_relay3, 0);  
      }
    }
    if (subscription == &Kontroll_Relay4) {
      Data = (char *)Kontroll_Relay4.lastread;
//      Serial.println(Data);
      if (Data == "OFF4"){
        digitalWrite (pin_relay4, 1);        
      }
      else if (Data == "ON4"){
        digitalWrite(pin_relay4, 0);
      }
    }
  }

    dht11.read(&temp, &hum, NULL);
//    Serial.print((int)temp); Serial.print(" *C, ");
//    Serial.print((int)hum); Serial.println(" H");
    delay(5000);
    
    if (! Kontroll_Temp.publish(temp)) {               //Publish Temperature data to Adafruit
//      Serial.println(F("Failed"));
    }
       if (! Kontroll_Temp2.publish(hum)) {               //Publish Humidity data to Adafruit
//      Serial.println(F("Failed"));
    }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

//  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
//       Serial.println(mqtt.connectErrorString(ret));
//       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
//  Serial.println("MQTT Connected!");
}
