/*
 * Send temperature from ESP8266 with multiple DS18B20 to MQTT server.
 * A simple Sketch to read the Temperature from multiple DS18B20 and publish them to a MQTT-Server using a ESP8266. 
 * Compiles in the Arduino IDE for the ESP8266
 * 
 * For deep sleep support uncomment 'deep sleep' part
 * For DHT22 support uncomment 'dht22' part
 * OTA currently does not work.
 * 
 * 
 */
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <Streaming.h>
#include <SPI.h>
#include <Ethernet.h>

// the media access control (ethernet hardware) address for the shield:
byte mac[] = { 0x90, 0xA2D, 0xDA, 0x0D, 0x4C, 0x56 };  
//the IP address for the shield:
byte ip[] = { 10, 0, 0, 177 };    

/* dht22
#include <DHT.h>
*/

// deep sleep
#define SLEEP_DELAY_IN_SECONDS  10
//

// data cable connected to D4 pin
#define ONE_WIRE_BUS 2

// dht22
/*
// data cable connected to D3
#define DHTPIN D3
#define DHTTYPE DHT22
char tString[6];
char hString[6];
long previousMillis = 0;
long interval = 60000;
*/

//mqtt
const char* mqtt_server = "pidrei.fritz.box";
//const char* mqtt_username = "mqtt";
//const char* mqtt_password = "mqtt";
String host = "uno";

// ora
/*
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
*/
//dht22
/*
DHT dht(DHTPIN, DHTTYPE, 20);
*/

EthernetClient ethClient;
PubSubClient client(ethClient);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

String mqttTopic = "iot/temp";

void setup_network() {
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }

  // print your local IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());

  // dht22
  /*
  dht.begin();
  */
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  // setup serial port
  Serial.begin(115200);

  // setup WiFi
  setup_network();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

 // ota
 /*
  MDNS.begin(host);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", host);
 */
  // setup OneWire bus
  DS18B20.begin();
}

void reconnect() {
  // Loop until we're reconnected
  //while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    //if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
    }
  //}
}

void loop() {
    switch (Ethernet.maintain()) {
    case 1:
      //renewed fail
      Serial.println("Error: renewed fail");
      break;

    case 2:
      //renewed success
      Serial.println("Renewed success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    case 3:
      //rebind fail
      Serial.println("Error: rebind fail");
      break;

    case 4:
      //rebind success
      Serial.println("Rebind success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    default:
      //nothing happened
      break;
  }
  
  if (!client.connected()) {
    reconnect();
  } else {
    client.loop();
  }

  /*
  // UNCOMMENT TO ENABLE DHT22 READINGS
  unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > interval) {
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float h = dht.readHumidity();
      dtostrf(h, 2, 2, hString);
      // Read temperature as Celsius
      float t = dht.readTemperature();
      dtostrf(t, 2, 2, tString);
      // Read temperature as Fahrenheit
      float f = dht.readTemperature(true);
        // Check if any reads failed and exit early (to try again).
        if (isnan(h) || isnan(f)) {
          Serial.println("Failed to read from DHT sensor!");
          return;
        }
      previousMillis = currentMillis;
      Serial.println("DTH sensor read and transmitted");
      Serial << "Sending temperature: " << tString << endl;
      Serial << "Sending humidity: " << hString << endl;
      
      client.publish("/sensors/esp1/dht/temperature",tString);
      client.publish("/sensors/esp1/dht/humidity",hString);
  */  
      
// ds18b20

  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  byte devCount = 0;
  String sensorId = "";
  String sensorData = "";

  //Loop through all DS1820
  
  while(oneWire.search(addr)) { 
  
  //Topic is built from a static String plus the ID of the DS18B20
    sensorId = "";
    for( i = 0; i < 8; i++) {
      sensorId += String(addr[i], HEX);
    }
  
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        sensorData += "-999";
    } else {
      //Serial.println();
     
    // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:
          //Serial.println("  Chip = DS18S20");  // or old DS1820
          type_s = 1;
          break;
        case 0x28:
          //Serial.println("  Chip = DS18B20");
          type_s = 0;
          break;
        case 0x22:
          //Serial.println("  Chip = DS1822");
          type_s = 0;
          break;
        default:
          Serial.println("Device is not a DS18x20 family device.");
          return;
      } 
    
      oneWire.reset();
      oneWire.select(addr);
      oneWire.write(0x44, 1);        // start conversion, with parasite power on at the end
      
      delay(1000);     // maybe 750ms is enough, maybe not
      // we might do a oneWire.depower() here, but the reset will take care of it.
      
      present = oneWire.reset();
      oneWire.select(addr);    
      oneWire.write(0xBE);         // Read Scratchpad
    
      /*
      Serial.print("  Data = ");
      Serial.print(present, HEX);
      Serial.print(" ");
      */
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = oneWire.read();
        /*
        Serial.print(data[i], HEX);
        Serial.print(" ");
        */
      }
      /*
      Serial.print(" CRC=");
      Serial.print(OneWire::crc8(data, 8), HEX);
      Serial.println();
      */
  
      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }
    
      //convert RAW Temperature to String
      String raw_temp = String(raw, DEC);
      //convert RAW Temperature to celsius
      double temp = raw * 0.0625;
      //convert to string
      char tempString[6];
      dtostrf(temp, 2, 2, tempString);
      sensorData = tempString;
    }

    String mqttMessage = String( "{ \"temperature\":"+sensorData+" }");
    if ( client.connected() && client.publish( String( mqttTopic+"/"+sensorId).c_str(), mqttMessage.c_str())) {
      Serial.print("pub temp ok ["+sensorId+"] = [");
      Serial.print(  mqttMessage   );
      Serial.println( "]" );
    }
    else {
      Serial.println("pub temp err ["+sensorId+"] ["+mqttMessage+"]");
    }

    devCount++;
  }

  //End of the OneWire-Devices, reset Loop
  Serial.print("1wire #[");
  Serial.print( devCount);
  Serial.println("] .");
  oneWire.reset_search();
  
  delay(SLEEP_DELAY_IN_SECONDS * 1000);  
/*
  // deep sleep
  //Serial << "Closing MQTT connection..." << endl;
  client.disconnect();
  //Serial << "Closing WiFi connection..." << endl;
  WiFi.disconnect();
  
 
  delay(100);
  // deep sleep
  //Serial << "Entering deep sleep mode for " << SLEEP_DELAY_IN_SECONDS << " seconds..." << endl;
  ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
  ESP.deepSleep(10 * 1000, WAKE_NO_RFCAL);
  delay(500); 
  */
}
