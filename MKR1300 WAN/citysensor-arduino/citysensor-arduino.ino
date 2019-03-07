#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "ArduinoLowPower.h"
#include <MKRWAN.h>

#include "secrets.h"

// address for SI1132
#define Addr 0x60
int response = 0;

Adafruit_BME680 bme;
LoRaModem modem;

struct bme
{
  float temp;
  float rh;
  float bp;
  float gas;
};

struct si
{
  float ir;
  float visible;
  float uv;
};

boolean confirmedup = false;
int loops = 1;

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Starting conserv prototype");

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.println("BME680 Initialized");

  initsi1132();
  Serial.println("SI1132 Initialized");

  if(!modem.begin(US915)) {
    Serial.println("Failed to init modem");
  }
  Serial.print("DEV EUI:");
  Serial.println(modem.deviceEUI());
  int connected = 0;
  while(!connected) {
    Serial.println("Joining TTN");
    connected = modem.joinOTAA(APP_EUI, APP_KEY);
    delay(2000);
    // set status LED here, and start retrying
  }
  Serial.println("Joined TTN");

  Serial.println("done");
}

void loop() {
  struct bme bmedata = bmeRead();
  struct si sidata = si1132Read();
  modem.beginPacket();
  modem.print("test");

  int err = modem.endPacket(confirmedup);
  if(err > 0) {
    Serial.println("Sent packet to TTN");
  }else {
    Serial.println("Error sending?");
  }
  
  //delay(180000);
  Serial.println("Sleeping");
  //modem.sleep(true);
  //LowPower.sleep(180000);
  delay(120000);
  //modem.sleep(false);
  loops++;
}

struct bme bmeRead() {

  struct bme bme_instance;
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  bme_instance.temp = bme.temperature;
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  bme_instance.bp = bme.pressure;
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  bme_instance.rh = bme.humidity;
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  bme_instance.gas = bme.gas_resistance / 1000;
  Serial.println(" KOhms");

  return bme_instance;
}

struct si si1132Read() {
  
  unsigned int data[4];
  struct si si_instance;
  
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Start ALS conversion
  Wire.write(0x0E);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x22);
  // Stop I2C Transmission
  Wire.endTransmission();
  
  // Request 4 byte of data
  Wire.requestFrom(Addr, 4);

  // Read 4 bytes of data
  // visible lsb, visible msb, ir lsb, ir msb
  if (Wire.available() == 4)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
  }

  float visible = (data[1] * 256.0 + data[0]);
  float ir = (data[3] * 256 + data[2]);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x2C);
  // Stop I2C Transmission
  Wire.endTransmission();
  
  // Request 2 bytes of data
  Wire.requestFrom(Addr, 2);

  // Read 2 bytes of data
  // uv lsb, uv msb
  if (Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  // Convert the data
  float uv = (data[1] * 256 + data[0]);

  // Output data to screen
  Serial.print("Visible Light of Source : ");
  Serial.print(visible);
  si_instance.visible = visible;
  Serial.println(" lux");
  Serial.print("IR Of Source : ");
  Serial.print(ir);
  si_instance.ir = ir;
  Serial.println(" lux");
  Serial.print("UV Of the Source : ");
  Serial.print(uv);
  si_instance.uv = uv;
  Serial.println(" lux");

  return si_instance;
}

void initsi1132() {
// Enable UVindex measurement coefficients
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-1 register
  Wire.write(0x13);
  // Default value
  Wire.write(0x29);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-2 register
  Wire.write(0x14);
  // Default value
  Wire.write(0x89);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-3 register
  Wire.write(0x15);
  // Default value
  Wire.write(0x02);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-4 register
  Wire.write(0x16);
  // Default value
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();


  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Enable uv, Visible, IR
  Wire.write(0xF0);
  // Stop I2C Transmission
  Wire.endTransmission();


  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select command register
  Wire.write(0x18);
  // Select CHLIST register in RAM
  Wire.write(0x01 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select HW_KEY register
  Wire.write(0x07);
  // Default value
  Wire.write(0x17);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Small IR photodiode
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_IR_ADCMUX register in RAM
  Wire.write(0x0E | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set ADC Clock divided / 1
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_IR_ADC_GAIN register in RAM
  Wire.write(0x1E | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set 511 ADC Clock
  Wire.write(0x70);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_IR_ADC_COUNTER register in RAM
  Wire.write(0x1D | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set ADC Clock divided / 1
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
 // Select ALS_VIS_ADC_GAIN register in RAM
  Wire.write(0x11 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // High Signal Range
  Wire.write(0x20);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_IR_ADC_MISC register in RAM
  Wire.write(0x1F | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set 511 ADC Clock
  Wire.write(0x70);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_VIS_ADC_COUNTER register in RAM
  Wire.write(0x10 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // High Signal Range
  Wire.write(0x20);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_VIS_ADC_MISC register in RAM
  Wire.write(0x12 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();
  delay(300);

}
