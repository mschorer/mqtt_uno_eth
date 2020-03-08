/*
  Send temperature from Arduino Uno with multiple DS18B20 to MQTT server.
  Try to reuse as much as possible from ESP8266 code

*/
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <Streaming.h>
#include <SPI.h>
#include <Ethernet.h>
//#include <Dns.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>

#include <EEPROM.h>

#define SW_VERSION  "ardu_0.111"

#define SLEEP_DELAY_IN_SECONDS  60
//---------------------------

// the media access control (ethernet hardware) address for the shield:
const byte mac[] = { 0x90, 0xA2D, 0xDA, 0x0D, 0x4C, 0x56 };

union busAddress {
  byte raw[8];
  long data[2];
};

#define RTC_BLOCK_OFFSET 496
struct rtcMem {
  long timestamp;
  long crc32;
};

rtcMem rtcCache;

// data cable connected to D4 pin
#define ONE_WIRE_BUS 2

#define FALSE 0
#define TRUE 1

//mqtt
const char* mqtt_server = "pidrei.fritz.box";
const char* mqtt_username = "mqtt";
const char* mqtt_password = "mqtt";

char nodeName[32] = "";
char nodeId[24] = "";

//---------------------------

EthernetClient ethClient;
PubSubClient client(ethClient);
//DNSClient dns;
IPAddress resolvedIP;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

//#define BME_I2C_ADDR 0x76
//Adafruit_BME280 bme; // I2C
//bool bmeAvailable = FALSE;

const char* mqttTopicTemp = "iot/temp";
const char* mqttTopicHumid = "iot/humidity";
const char* mqttTopicPAtmo = "iot/atmosphere";

//---------------------------

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void setup_network() {

  while(Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }

    delay(5000);
  }

  sprintf( nodeId, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sprintf( nodeName, "uno_%s", nodeId);
  Serial.print( "name [");
  Serial.print( nodeName);
  Serial.print( "]");

  // print your local IP address:
  Serial.print(" @");
  Serial.println(Ethernet.localIP());
  /*
    Serial.print("DNS #1 ");
    Ethernet.dnsServerIP().printTo(Serial);
    Serial.println();

    dns.begin(Ethernet.dnsServerIP());
    if (!dns.getHostByName( mqtt_server, resolvedIP)) {
      Serial.print("DNS lookup [");
      Serial.print( mqtt_server);
      Serial.print("] [");
      Serial.print( resolvedIP);
      Serial.println("] failed.  Rebooting...");
      Serial.flush();
      //ESP.reset();
    }
  */
  resolvedIP = IPAddress( 192, 168, 1, 71);
  Serial.print("mqtt: ");
  Serial.print( mqtt_server);
  Serial.print(" @");
  Serial.println(resolvedIP);

  pinMode(A0, INPUT);

  // dht22
  /*
    dht.begin();
  */
}

void updateSettings() {
    rtcCache.crc32 = calculateCRC32( (uint8_t *)&rtcCache.timestamp, 4);
    EEPROM.put( RTC_BLOCK_OFFSET, rtcCache);
    //ESP.rtcUserMemoryWrite( RTC_BLOCK_OFFSET, (uint32_t*)&rtcCache, sizeof(rtcCache));
}

byte retrieveSettings() {
  //ESP.rtcUserMemoryRead( RTC_BLOCK_OFFSET, (uint32_t*)&rtcCache, sizeof(rtcCache));
  EEPROM.get( RTC_BLOCK_OFFSET, rtcCache);
  uint32_t crcNow = calculateCRC32( (uint8_t *)&rtcCache.timestamp, 4);

  if ( rtcCache.crc32 != crcNow) {
    Serial.print( "crc error [");
    Serial.print( rtcCache.crc32);
    Serial.print( "] != [");
    Serial.print( crcNow);
    Serial.println( "] resetting time");

    return FALSE;
  } else return TRUE;
}

//---------------------------

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//---------------------------

void setup() {
  // setup serial port
  Serial.begin(115200);

  if ( ! retrieveSettings()) {
    rtcCache.timestamp = 0;
    updateSettings();
  }

  Serial.print( "time [");
  Serial.print( rtcCache.timestamp);
  Serial.println( "]");

  // setup Ethernet
  setup_network();
  client.setServer( resolvedIP, 1883);
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

  /*
    if ( ! bme.begin( BME_I2C_ADDR, &Wire)) {
      Serial.println( "Could not find a valid BME280 sensor, skipping ...");
      bmeAvailable = FALSE;
    } else {
    bmeAvailable = TRUE;
    // weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    }
  */
}

//---------------------------

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

void publishTopic( char* mqttTopic, char* sensorID, char* sensorName, double value) {
  char topic[32];
  char msg[64];
  char sensorData[6];

  dtostrf(value, 4, 2, sensorData);
  sprintf( topic, "%s/%s", mqttTopic, sensorID);
  sprintf( msg, "{ \"node\":\"%s\", \"ts\":%ld, \"%s\":%s }", nodeName, rtcCache.timestamp, sensorName, sensorData);

  Serial.print("node [");
  Serial.print( topic);
  Serial.print("] = [");
  Serial.print( msg);
  Serial.print("] ");

  if ( reconnect()) {
    if ( client.publish( topic, msg, TRUE)) {
      Serial.println( "pub ok" );
    } else {
      Serial.println("pub err");
    }
  } else {
    Serial.print( " err[");
    Serial.print( client.state());
    Serial.println( "]");
  }
}

void publishNodeSummary( int devCount) {
  char topic[32];
  char msg[100];
  
  sprintf( topic, "iot/nodes/%s", nodeName);
  sprintf( msg, "{ \"name\":\"%s\", \"version\":\"%s\", \"ticks\":%ld, \"devices\":%i }", nodeName, SW_VERSION, rtcCache.timestamp, devCount);

  Serial.print("node [");
  Serial.print( topic);
  Serial.print("] = [");
  Serial.print( msg);
  Serial.print("] ");

  if ( reconnect()) {
    if ( client.publish( topic, msg, TRUE)) {
      Serial.println( "pub ok" );
    } else {
      Serial.println("pub err");
    }
  } else {
    Serial.print( " err[");
    Serial.print( client.state());
    Serial.println( "]");
  }
}

bool reconnect() {
  byte i = 3;
  int state = client.state();

  // Loop until we're reconnected
  while (( state != MQTT_CONNECTED) && (i > 0)) {
    Serial.print("mqtt[");
    Serial.print( state);
    Serial.print( "] ");

    if (client.connect( nodeName)) {
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

//---------------------------

void loop() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte devCount = 0;
  byte sensor = 0;
  char sensorId[16];
  char sensorType[8];
  char hex[3];
  
  byte sensors = 0;
  busAddress sensorIDs[ 16];

  /*
    if ( bmeAvailable)
    bme.takeForcedMeasurement();
  */

  //Loop through all DS1820
  sensors = 0;
  while ( oneWire.search( sensorIDs[sensors].raw)) {
    networkLoop();

    //Topic is built from a static String plus the ID of the DS18B20
    if ( crcCheck( sensorIDs[sensors].raw, 7)) {
      continue;
    }

    // the first ROM byte indicates which chip
    switch (sensorIDs[sensors].raw[0]) {
      case 0x10:
        strcpy( sensorType, "DS18S20");  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        strcpy( sensorType, "DS18B20");
        type_s = 0;
        break;
      case 0x22:
        strcpy( sensorType, "DS1822");
        type_s = 0;
        break;
      default:
        strcpy( sensorType, "not a DS18x20 family device");
        return;
    }

    oneWire.reset();
    oneWire.select( sensorIDs[sensors].raw);
    oneWire.write( 0x44, 1);        // start conversion, with parasite power on at the end

    sensorId[0] = 0;
    for ( byte x = 0; x < 8; x++) {
      sprintf( hex, "%02x", sensorIDs[sensors].raw[x]);
      strcat( sensorId, hex);
    }

    Serial.print( "found: [");
    Serial.print( sensorType);
    Serial.print( "] @ [");
    Serial.print( sensorId);
    Serial.println( "]");

    // we might do a oneWire.depower() here, but the reset will take care of it.

    present = oneWire.reset();

    sensors++;
  }
  oneWire.reset_search();

  Serial.print( "found #[");
  Serial.print( sensors);
  Serial.println( "] oneWire sensors");

  delay(1000);     // maybe 750ms is enough, maybe not

  sensor = 0;
  while ( sensor < sensors) {
    oneWire.select( sensorIDs[sensor].raw);
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
    //String raw_temp = String(raw, DEC);
    //convert RAW Temperature to celsius
    double temp = raw * 0.0625;

    sensorId[0] = 0;
    for ( byte x = 0; x < 8; x++) {
      sprintf( hex, "%02x", sensorIDs[sensor].raw[x]);
      strcat( sensorId, hex);
    }

    if (rtcCache.timestamp || temp != 85.0) {
      publishTopic( mqttTopicTemp, sensorId, "temperature", temp);
      client.loop();
    } else {
      Serial.println("skip");
    }

    devCount++;
    sensor++;
  }

  /*
    if ( bmeAvailable) {
    publishTopic( mqttTopicTemp, "bme_"+String( nodeId), "temperature", bme.readTemperature());
    devCount++;
    publishTopic( mqttTopicPAtmo, "bme_"+String( nodeId), "pressure", bme.readPressure() / 100.0F);
    devCount++;
    publishTopic( mqttTopicHumid, "bme_"+String( nodeId), "humidity", bme.readHumidity());
    devCount++;
    }
  */

  Serial.print( "dev #");
  Serial.println( devCount);

  if (devCount > 0) rtcCache.timestamp++;
  byte gap = SLEEP_DELAY_IN_SECONDS;

  publishNodeSummary( devCount);

  if ( 0 == (rtcCache.timestamp % 15)) {
    updateSettings();
  }

  Serial.print( "sleep ");
  Serial.print( gap);
  Serial.println( "s.");

  while ( gap > 0) {
    client.loop();
    networkLoop();
    delay( 1000);
    gap--;
  }
}
