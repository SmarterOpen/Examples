#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include <PMS.h>
#include <Adafruit_GPS.h>

// pins for OLED
#define SDA    4
#define SCL   15
#define RST   16 //RST must be set by software
#define Vext  21
SSD1306  display(0x3c, SDA, SCL, RST);
String millisec;
String pmstr;
String lines[] = {"", "", "", "", "", ""};

// setup for GPS
HardwareSerial GPSSerial(2);
Adafruit_GPS GPS(&GPSSerial);

// setup for PMS sensor (Plantower)
HardwareSerial PMSSerial(1);
PMS pms(PMSSerial);
PMS::DATA data;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ INSERT YOUR APP EUI };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ INSERT YOUR DEV EUI };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { INSERT YOUR APP KEY};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t payload[9];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

int txcomplete = 0;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

struct pm
{
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10_0;
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            txcomplete = 1;
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        // insert sensor readings here in the loop, read, then queue up the readings to be sent.
        struct pm pmdata = getPM();
        buildPayload(pmdata.pm2_5, GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.satellites);
        displayLine("Sending job");
        Serial.println("sending job");
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    PMSSerial.begin(9600, SERIAL_8N1, 36, 37);
    GPSSerial.begin(9600, SERIAL_8N1, 21, 13);
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("Starting"));

    pinMode(RST,OUTPUT);
    digitalWrite(Vext, LOW);    // OLED USE Vext as power supply, must turn ON Vext before OLED init
    delay(50);
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    // Turn on RMC and GGA including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    SPI.begin(5, 19, 27);

    displayLine("Initializing");
    
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // expand the receive window slightly
    // see:  https://www.thethingsnetwork.org/forum/t/over-the-air-activation-otaa-with-lmic/1921/58
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // disable link check mode for now
    //LMIC_setLinkCheckMode(0);

    // disable ADR mode as well
    //LMIC_setAdrMode(0);

    // set up txpower and a higher SF for longer range
    //LMIC_setDrTxpow(DR_SF10, 27);
    //LMIC.txpow=27;
    
    do_send(&sendjob);
    
}

void loop() {

    os_runloop_once();
    GPS.read();
    
    /*if(!(LMIC.opmode & OP_TXRXPEND)) {
      millisec = String(millis());
      //displayLine(millisec);
      displayLine("Tx at " + millisec);
      Serial.println("tx complete=1, delaying");

      int start = millis();
      //while(millis() - start < 30000) {
        GPS.read();
      //}

     
      
      //payload[0]=pmdata.pm2_5 & 0xff;
      //payload[1]=(pmdata.pm2_5 >> 8);
      
      txcomplete = 0;
      // Start job (sending automatically starts OTAA too)
      displayLine("Sending job");
      Serial.println("sending job");
      //do_send(&sendjob);
    }*/
}

void buildPayload(int pm2_5, float lat, float lon, int sats) {

   // do we have a GPS fix?  If so, add gps data to the package
      if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA());   
        Serial.println("Got message, parsed data");
      }
      
      if(GPS.fix) {
        Serial.println(F("GPS fix available"));
        displayLine("Got GPS fix");
        Serial.print("Sats: ");
        Serial.println(GPS.satellites);
        Serial.print("HDOP: ");
        Serial.println(GPS.HDOP);
        Serial.print("GPS.latitudeDegrees: ");
        Serial.println(GPS.latitudeDegrees, 8);
        Serial.print("GPS.longitudeDegrees: ");
        Serial.println(GPS.longitudeDegrees, 8);
        /*JsonObject& gpsdataobj = root.createNestedObject("gps");
        gpsdataobj["fixquality"] = (int)GPS.fixquality;
        gpsdataobj["latdegrees"] = GPS.latitudeDegrees;
        gpsdataobj["londegrees"] = GPS.longitudeDegrees;
        gpsdataobj["speed"] = GPS.speed;
        gpsdataobj["altitude"] = GPS.altitude;
        gpsdataobj["sats"] = GPS.satellites;*/
      }else {
        Serial.println(F("No GPS fix available"));
        displayLine("No GPS fix");
      }

  payload[0] = pm2_5 & 0xff;
  payload[1] = (pm2_5 >> 8);

  /*
   * first, get the int part of the GPS coordinate part:
   * 44.12345 becomes 44
   * then, subtract that fromt he original:
   * 44.12345 becomes .12345
   * then, multiply by 10000 to shift the decimal point
   * 44.12345 becomes 1234.5
   * finally, take the int value of the bit we need
   * 1234.5 becomes 1234
   * 
   * This is possible because we have a well-defined bounding box around
   * the area we want to map.  lat will start with 44, lon with -86
   */
  int latintp = (int)lat;
  int lonintp = (int)lon;
  lat = (lat - latintp) * 10000;
  lon = (lon - lonintp) * 10000;
  int latval = abs((int)lat);
  int lonval = abs((int)lon);

  Serial.print("lat: ");
  Serial.println( latval);
  Serial.print("lon: ");
  Serial.println(lonval);
  
  payload[2] = latval & 0xff;
  payload[3] = (latval >> 8);

  payload[4] = lonval & 0xff;
  payload[5] = (lonval >> 8);

  // fudge the altitude value for now
  payload[6] = 100;

  /*uint32_t val = HDOP * 10 + 0.5;
  payload[7] = val & 0xff;
  payload[8] = val >> 8;*/
  payload[7] = sats;
  
}

/**
 * Adds a line to the bottom of the display, shifts other lines up (scrolling display)
 */
void displayLine(String line) {

   display.clear();
   for(int i = 0; i < 5; i++) {
    display.drawString(0, i * 10, lines[i+1]);
    lines[i] = lines[i+1];
   }
   display.drawString(0, 50, line);
   lines[5] = line;
   display.display();
}

/**
 * Reads from Plantower *003 sensor, returns struct with all of the read values.
 */
struct pm getPM() {

  struct pm pm_instance;
  int read = 0;

  // TODO - add a timeout here so that the read can't run forever.
  while(!read) {
    if (pms.read(data)){
        
       pm_instance.pm1_0 = data.PM_AE_UG_1_0;
       pm_instance.pm2_5 = data.PM_AE_UG_2_5;
       pm_instance.pm10_0 = data.PM_AE_UG_10_0;
       read = 1;
    }
  }

  pmstr = String("10=");
  pmstr += pm_instance.pm10_0;
  pmstr += " 2.5=";
  pmstr += pm_instance.pm2_5;
  pmstr += " 1.0=";
  pmstr += pm_instance.pm1_0;
  
  displayLine(pmstr);
  Serial.println(pmstr);
  return pm_instance;
}
