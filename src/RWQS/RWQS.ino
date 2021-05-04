/*
 *  Remote Water Quality Sensor
 *  Node B
 *  Author: Derek Schrag
 *  Last Updated: 5/4/2021
 */

#include <SPI.h>      // Arduino lib for serial communications
#include <OneWire.h>  // Temperature Sensor lib
#include <TinyGPS++.h> // GPS lib
#include <TinyLoRa.h> // LoRa lib

// Sensor Pins
#define PHSensorPin A1
#define TdsSensorPin A2
#define TempSensorPin A0

// Device Voltage, Constants for formulas
#define VREF 3.3
#define SCOUNT 30
#define OFFSET -0.24
#define PHSLOPE 0.0766666667
#define LED 13

//*****************************************Gateway Connection Keys*****************************************
// Application Key (MSB)
//uint8_t Appkey[16] = { 0x72 0x46 0xdf 0xc8 0x75 0x12 0x7d 0x47 0x75 0x86 0xe5 0x67 0x34 0xdd 0x52 0x45 };

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0xe8, 0x45, 0x82, 0xfc, 0x62, 0xb1, 0x4c, 0xa2, 0x64, 0xcd, 0x67, 0xe7, 0x86, 0xd4, 0x20, 0x01 };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x81, 0x83, 0x1d, 0x29, 0xc4, 0xb2, 0xd7, 0xbb, 0xb5, 0x5c, 0x6a, 0x6f, 0x1f, 0xc8, 0xc1, 0xac };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x01, 0x81, 0x1e, 0xc2 };
//*********************************************************************************************************

//*****************************************Variable Setup**************************************************
// Instance of lora radio with pins for Feather 32u4
TinyLoRa lora = TinyLoRa(7, 8, 4);

static const uint32_t GPSBaud = 9600;

int tempPin = TempSensorPin;

// Instance of the temperature sensor
OneWire ds(tempPin);

// Instance of our GPS object
TinyGPSPlus gps;

// Buffers, indices, and holding variables for sensor data
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, tdstemperature = 25, temperature;
char cBuff[10];
int pHBuff[10];
int cnt = 0;
unsigned long int avgValuePH;
//bool firstTime = 1;
//**********************************************************************************************************

//*****************************************Functions********************************************************
// Returns the temperature from one DS18S20 in DEG Celsius
// Source: https://wiki.dfrobot.com/Gravity__DS18B20_Temperature_Sensor__Arduino_Compatible__V2_SKU__DFR0024
float getTemp()
{

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}

// Returns the median from a given array and specified max length
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];

  // Fill working array
  for(byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];

  int i, j, bTemp;

  // Sort working array
  for(j = 0; j < iFilterLen - 1; j++)
  {
    for(i = 0; i < iFilterLen - j - 1; i++)
    {
      if(bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  // Grab median number
  if((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;

  return bTemp;
}

// Rounds a given float to 2 decimal places and returns as a float
float phRound(float var)
{
  float v = (int) (var * 100 + .5);
  return (float) v / 100;
}
//**********************************************************************************************************

void setup() 
{
  delay(3000);

  // Set sensor pins to read
  pinMode(TdsSensorPin, INPUT);
  pinMode(TempSensorPin, INPUT);
  pinMode(PHSensorPin, INPUT);

  // Baud rate 115200 for all sensors
  Serial.begin(115200);
  delay(100);

  // Begin serial for GPS
  Serial1.begin(GPSBaud);

  // Set LoRa parameters for transmission in US
  lora.setChannel(CH2);
  lora.setDatarate(SF7BW125);
  if(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check the radio");
    while(true);
  }
 
  Serial.println("Starting up...");
}
 
void loop()
{
  static unsigned long analogSampleTimepoint = millis();

  // Every 60000 milliseconds (60 seconds) take a TDS and PH reading
  // We are taking a rolling average of these readings (30 for TDS, 10 for PH)
  if(millis() - analogSampleTimepoint > 60000U)  
  {
    // Read TDS value, increment buffer index
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;

    // For rolling buffer
    if(analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;

    // Read pH value, rolling buffer
    if(cnt < 10)
      pHBuff[cnt] = analogRead(PHSensorPin);
    else
    {
      cnt = 0;
      pHBuff[cnt] = analogRead(PHSensorPin);
    }
  }

  // Time for broadcasting sensor data
  static unsigned long printSensorTimepoint = millis();

  // Every 3 600 000 milliseconds (60 minutes), gather all sensor information, format it, and transmitting it
  if(millis() - printSensorTimepoint > 3600000U)
  {
    printSensorTimepoint = millis();

    // Copy over TDS values for use
    for(copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

    // Sort PH values in buffer
    for(int i = 0; i < 9; i++)
    {
      for(int j = i + 1; j < 10; j++)
      {
        if(pHBuff[i] > pHBuff[j])
        {
          int phtemp = pHBuff[i];
          pHBuff[i] = pHBuff[j];
          pHBuff[j] = phtemp;
        }
      }
    }

    // Take average PH reading (currently analog value)
    avgValuePH = 0;
    for(int i = 2; i < 8; i++)
      avgValuePH += pHBuff[i];

    // Calculate average voltage for TDS value (ignore TDS temperature, sensor didn't come with one)
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    float compensationCoefficient = 1.0 + 0.02 * (tdstemperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    // TDS formula from manufacturer of the sensor; tds_val = (133.42 * compVolt^2 - 255.86 * compVolt^2 + 857.39 * compVolt) * 0.5
    // Source: https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0
    tdsValue = (133.42 * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

    // Grab current temperature in Degrees Celcius, make sure there is no error
    temperature = getTemp();
    while((int)temperature == -1000)
      temperature = getTemp();
      
    // Calculate voltage reading from PH sensor, then calculate actual PH value.
    // Formula for pH: pH = PH7 + ((PH7_Voltage - Voltage) - pH_Slope) + Offset
    float phVolt = (float) avgValuePH * 3.3 / 1024 / 6;
    float phValue = 7 + ((1.13-phVolt) / -PHSLOPE) + OFFSET;
  
    Serial.println("Transmitting..."); // Send a message to rf95_server
    digitalWrite(LED, HIGH);

    // Data packet to be transmitted
    unsigned char radiopacket[20] = "";
    
    // Convert TDS value to string for packet, add to packet, delimit with :
    itoa(tdsValue, radiopacket, 10); 
    strcat(radiopacket, ":");

    // Convert temperature to string for packet, add to packet, delimit with :
    itoa(temperature, cBuff, 10);
    strcat(radiopacket, cBuff);
    strcat(radiopacket, ":");

    // Round PH value to 2 decimal places
    phValue = phRound(phValue);
    phValue = phValue * 100;

    // Convert PH value to string for packet, add to packet, delimit with :
    itoa(int(phValue), cBuff, 10);
    strcat(radiopacket, cBuff);

    // Broadcast the packet over LoRa, increment frame counter
    lora.sendData(radiopacket, sizeof(radiopacket), lora.frameCounter);
    Serial.print("Frame Counter: ");
    Serial.println(lora.frameCounter);
    lora.frameCounter++;
    Serial.println("Sending...");
    delay(10);
  }

  // This will end up offsetting the GPS transmission and sensor data transmission by 30 minutes
  static unsigned long gpsTimepoint = millis() - 1800000U;

  // Every 3 600 000 milliseconds (60 minutes) get location data, format it, and transmit it.
  if(millis() - gpsTimepoint > 3600000U)
  {
    gpsTimepoint = millis();
    unsigned char radiopacket[20] = "";
    int gpsLatitude, gpsLongitude;
    if(gps.encode(Serial1.read()))
    {
      if(gps.location.isValid())
      {
        // Multiply lat and long coords by 10,000 to essential save 4 decimal places
        gpsLatitude = int(gps.location.lat() * 10000);
        gpsLongitude = int(gps.location.lng() * 10000);

        // Add latitude and longitude to the packet string, delimit with semi-colon ;
        itoa(gpsLatitude, radiopacket, 10);
        strcat(radiopacket, ';');

        itoa(gpsLongitude, cBuff, 10);
        strcat(radiopacket, cBuff);


        // Broadcast the packet over LoRa, increment frame counter
        lora.sendData(radiopacket, sizeof(radiopacket), lora.frameCounter);
        Serial.print("Frame Counter: ");
        Serial.println(lora.frameCounter);
        lora.frameCounter++;
        Serial.println("Sending...");
        delay(10);
      }
    }
  }
  
}
