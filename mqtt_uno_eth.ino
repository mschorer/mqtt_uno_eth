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

#define SW_VERSION  "ardu_0.96"

// the media access control (ethernet hardware) address for the shield:
byte mac[] = { 0x90, 0xA2D, 0xDA, 0x0D, 0x4C, 0x56 };  
//the IP address for the shield:
byte ip[] = { 10, 0, 0, 177 };    

byte sensors = 0;
byte sensorIDs[ 32][8];

/* dht22
#include <DHT.h>
*/

// deep sleep
#define SLEEP_DELAY_IN_SECONDS  30
//

// data cable connected to D4 pin
#define ONE_WIRE_BUS 2

#define FALSE 0
#define TRUE 1

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
const char* mqtt_username = "mqtt";
const char* mqtt_password = "mqtt";
const char* node = "uno";

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

int busScans = 0;

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
      delay(5);
      Serial.println( "no eth connected.");
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

void networkLoop() {
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
}

bool reconnect() {
  byte i = 3;
  int state = client.state();
  /*
  Serial.print( "state [");
  Serial.print( state);
  Serial.print( "] [");
  Serial.print( client.connected());
  Serial.println( "]");
  */
  // Loop until we're reconnected
  while (( state != MQTT_CONNECTED) && (i > 0)) {
    Serial.print("mqtt[");
    Serial.print( state);
    Serial.print( "] ");

    if (client.connect( node)) {
      Serial.print("ok ");
      client.loop();

      client.subscribe("iot/cmd/#");
      break;
    } else {
            delay( 1000);
      state = client.state();
    }
    i--;
  }

  return (i > 0);
}

byte crcCheck( byte* buf, byte maxIdx) {
    byte crc = OneWire::crc8( buf, maxIdx);
    if ( crc != buf[maxIdx]) {
      Serial.print("CRC err ");
      Serial.print( maxIdx, DEC);
      Serial.print(" ");
      Serial.print( crc, HEX);
      Serial.print( " != ");
      Serial.println( buf[maxIdx], HEX);
      return TRUE;
    }

    return FALSE;
}

union busAddress {
  byte raw[8];
  long data[2];
};

void loop() {
  busAddress addr;
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte devCount = 0;
  char sensorId[32];
  String sensorData = "";

  //Loop through all DS1820
  
  while( oneWire.search( addr.raw)) { 
    networkLoop();
    client.loop();
 
    //Topic is built from a static String plus the ID of the DS18B20
    if ( crcCheck( addr.raw, 7)) {
        continue;
    }

    // the first ROM byte indicates which chip
    switch (addr.raw[0]) {
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
    oneWire.select( addr.raw);
    oneWire.write( 0x44, 1);        // start conversion, with parasite power on at the end
    
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a oneWire.depower() here, but the reset will take care of it.
    
    present = oneWire.reset();
    oneWire.select( addr.raw);    
    oneWire.write( 0xBE);         // Read Scratchpad
  
    /*
    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    */
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = oneWire.read();
    }
    if ( crcCheck( data, 8)) {
      continue;
    }

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

    sensorId[0] = 0;
    char hex[20];
    for( byte x=0; x < 8; x++) {
      sprintf( hex, "%02x", addr.raw[x]);
      strcat( sensorId, hex);
    }

    String fullTopic = String( mqttTopic+"/"+sensorId);
    String mqttMessage = String( "{ ");
    mqttMessage += "\"node\":\""+String( node)+"\", ";
    mqttMessage += "\"ts\":"+String(busScans)+", ";
    mqttMessage += "\"temperature\":"+sensorData;
    mqttMessage += " }";

    Serial.print("sensor [");
    Serial.print( fullTopic);
    Serial.print("] = [");
    Serial.print(  mqttMessage   );
    Serial.print("] ");    

    if ( busScans || temp != 85.0) {
      if ( reconnect()) {
        if (client.publish( fullTopic.c_str(), mqttMessage.c_str(), TRUE)) {
          Serial.println( "pub ok" );
        }
        else {
          Serial.println("pub err");
        }
      } else {
        Serial.println("recon ");
      }
      client.loop();
    } else {
      Serial.println("skip");
    }

    devCount++;
  }
  oneWire.reset_search();

  if (devCount > 0) busScans++;
  byte gap = SLEEP_DELAY_IN_SECONDS - devCount;

  String fullTopic = String( "iot/nodes/")+String(node);
  String mqttMessage = String( "{ ");
  mqttMessage += "\"name\":\""+String(node)+"\", ";
  mqttMessage += "\"version\":\""+String( SW_VERSION)+"\", ";
  mqttMessage += "\"ticks\":"+String(busScans)+", ";
  mqttMessage += "\"devices\":"+String(devCount)+" ";
  mqttMessage += "}";

  Serial.print("node [");
  Serial.print( fullTopic);
  Serial.print("] = [");
  Serial.print(  mqttMessage   );
  Serial.print("] ");    

  if ( reconnect()) {
    if ( client.publish( fullTopic.c_str(), mqttMessage.c_str(), TRUE)) {
      Serial.println( "pub ok" );
    } else {
      Serial.println("pub err");
    }
  } else {
    Serial.print( " err[");
    Serial.print( client.state());
    Serial.println( "]");
  }
  Serial.print( "dev #"+String(devCount));
  Serial.print( " sleep ");
  Serial.print( gap);
  Serial.println( "s.");

  while( gap > 0) {
    delay( 1000);  
    client.loop();
    networkLoop();
    gap--;
  }  
}
