/*
 *  Remote Water Quality Sensor
 *  Node B
 *  Author: Derek Schrag
 *  Last Updated: 2/28/2021
 *  
 *  TODO: Implement Gateway communication (authentication, uplink/downlink frames)
 *        Implement GPS (communication, formatting, transmission; Uses TinyGPS+ https://github.com/mikalhart/TinyGPSPlus) 
 *        Recalibrate PHSLOPE
 *        Rethink usage of delimitters in packet string
 */

#include <SPI.h>      // Arduino lib for serial communications
#include <OneWire.h>  // Temperature Sensor lib
#include <RH_RF95.h>  // LoRa lib

#define PHSensorPin A1	// Data pins for sensors
#define TdsSensorPin A2
#define TempSensorPin A0

#define VREF 3.3	// Arduino Output Voltage
#define SCOUNT 30	

#define Offset 0	// Offset for PH calculation
#define PHSLOPE 5.3846	// Calculated slope for PH value calulation

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define LED 13

// 900MHz
#define RF95_FREQ 900.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int tempPin = TempSensorPin;

// Instance of the temperature sensor
OneWire ds(tempPin);

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, tdstemperature = 25, temperature;
char cBuff[6];
int pHBuff[10];
int cnt = 0;
unsigned long int avgValuePH;

// Returns the temperature from one DS18S20 in DEG Celsius
// Source: https://wiki.dfrobot.com/Gravity__DS18B20_Temperature_Sensor__Arduino_Compatible__V2_SKU__DFR0024
float getTemp(){

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
float round(float var)
{
  float v = (int) (var * 100 + .5);
  return (float) v / 100;
}

void setup() 
{
  // Init & starts LoRa radio 
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Set sensor pins to read
  pinMode(TdsSensorPin, INPUT);
  pinMode(TempSensorPin, INPUT);
  pinMode(PHSensorPin, INPUT);

  // Baud rate 115200 for all sensors
  Serial.begin(115200);
  delay(100);
 
  Serial.println("Starting up...");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Check for failure to initialize LoRa raido
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}
 
int16_t packetnum = 0;  // packet counter, we increment per xmission
 
void loop()
{
  static unsigned long analogSampleTimepoint = millis();

  // Every 3000 milliseconds take a TDS and PH reading
  // We are taking a rolling average of these readings (30 for TDS, 10 for PH)
  if(millis() - analogSampleTimepoint > 3000U)  
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;

    if(analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;

    if(cnt < 10)
      pHBuff[cnt] = analogRead(PHSensorPin);
    else
    {
      cnt = 0;
      pHBuff[cnt] = analogRead(PHSensorPin);
    }
  }

  static unsigned long printTimepoint = millis();

  // Every 6 000 000 milliseconds (10 minutes), gather all sensor information, format it, and transmitting it
  if(millis() - printTimepoint > 6000000U)
  {
    printTimepoint = millis();

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

    // Calculate average voltage for TDS value
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
    float phVolt = (float) avgValuePH * 3.3 / 1024 / 6;
    float phValue = PHSLOPE * phVolt + Offset;
  
    Serial.println("Transmitting..."); // Send a message to rf95_server
    digitalWrite(LED, HIGH);

    // Data packet to be transmitted
    char radiopacket[20] = "";
    
    // Convert TDS value to string for packet, add to packet, delimit with :
    itoa(tdsValue, radiopacket, 10); 
    strcat(radiopacket, ":");

    // Convert temperature to string for packet, add to packet, delimit with :
    itoa(temperature, cBuff, 10);
    strcat(radiopacket, cBuff);
    strcat(radiopacket, ":");

    // Round PH value to 2 decimal places
    phValue = round(phValue);
    phValue = phValue * 100;

    //Convert PH value to string for packet, add to packet, delimit with :
    itoa(int(phValue), cBuff, 10);
    strcat(radiopacket, cBuff);
    strcat(radiopacket, ":");
    
    Serial.print("Sending "); Serial.println(radiopacket);

    // Make packet null-terminated for LoRa
    radiopacket[19] = 0;
    
    Serial.println("Sending...");
    delay(10);

    // Transmit packet unto the void
    rf95.send((uint8_t *)radiopacket, 20);
   
    Serial.println("Waiting for packet to complete..."); 
    delay(10);

    // Wait for confirmation of packet being received
    // TODO: Change to work with Gateway communication (uplink/downlink frame tracking)
    rf95.waitPacketSent();

  }
}
