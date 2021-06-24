#include <Arduino.h>
#include <SPIMemory.h>
#include <HDC2080.h>

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include "RTClib.h"

RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define ADDR 0x40
HDC2080 sensor(ADDR);

#define LED_Blink_rate 1000
#define TEMP_READ_rate 1000
#define BOOTTIMEOUT 3
#define ADDS_OFFSET 4500

uint32_t strAddr;
SPIFlash flash;
bool flashSetup()
{
  long lastmillis1 = millis();
  long lastmillis = millis();
  bool timeout = true;
  int timetotimeout = BOOTTIMEOUT;
  flash.begin();
  strAddr = flash.readULong(0);
  Serial.print("Flash Start Address: ");
  Serial.println(strAddr);
  Serial.println(flash.readShort(sizeof(strAddr) + 1));
  if (strAddr != 0 && flash.readByte(sizeof(strAddr) + 1) == 1)
  {
    Serial.println("Data Saved to flash, Dump flash contents? \n Enter 1 to dump");
    while (timeout)
    {
      if (Serial.available() != 0)
      {
        char value = Serial.read();

        if (value == '1')
        {
          Serial.println("Dumping Flash in CSV form");
        }
        Serial.println("Year,Month,date,Hr,Min,Sec,Temp,Humidity");

        for (auto i = 0; i < strAddr; i+= 15)
        {
          uint16_t year = flash.readShort(i  + ADDS_OFFSET);        //year HIGH
          uint8_t month = flash.readByte(i+ 1 * 2 + ADDS_OFFSET);    //month
          uint8_t day = flash.readByte(i+ 1 * 3 + ADDS_OFFSET);      //day
          uint8_t hour = flash.readByte(i+ 1 * 4 + ADDS_OFFSET);     //hour
          uint8_t min = flash.readByte(i+ 1 * 5 + ADDS_OFFSET);      //min
          uint8_t sec = flash.readByte(i + 1 * 6 + ADDS_OFFSET);      //sec
          float temp = flash.readFloat(i+ 1 * 7 + ADDS_OFFSET);     //sec
          float hum = flash.readFloat(i+ 1 * 11+ ADDS_OFFSET);      //sec

          Serial.print(year);
          Serial.print(",");
          Serial.print(month);
          Serial.print(",");
          Serial.print(day);
          Serial.print(",");
          Serial.print(hour);
          Serial.print(",");
          Serial.print(min);
          Serial.print(",");
          Serial.print(sec);
          Serial.print(",");
          Serial.print(temp);
          Serial.print(",");
          Serial.println(hum);
        }
        timeout = false;
        Serial.println("Erase Chip, on next boot? 1 for yes, anything else for no");
        Serial.flush();

        while (Serial.available() == 0)
        {
          /* code */
        }

        /* code */
        if (Serial.read() == '1')
        {
          Serial.println("Flash will be erased on next boot, halting execution");
          flash.eraseSector(0);
          while (1)
          {
            /* code */
          }
        }
        else
        {
          Serial.println("Continuing logging");
        }
      }

      if (millis() / 1000 - lastmillis1 / 1000 >= BOOTTIMEOUT)
      {
        timeout = false;
      }
      if (millis() - lastmillis >= 1000)
      {
        Serial.println(timetotimeout--);
        lastmillis = millis();
      }
    }
  }
  else
  {
    Serial.println("Flash empty, Erasing Chip, Takes 60+ Seconds");
    flash.eraseChip();
    Serial.println(flash.writeShort(sizeof(strAddr) + 1, 1, true));
    strAddr = 0;
  }
}

void rtcSetup()
{
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}
void setupTempSensor()
{
  // // Initialize I2C communication
  sensor.begin();

  // // Begin with a device reset
  sensor.reset();

  // Set up the comfort zone
  sensor.setHighTemp(28);     // High temperature of 28C
  sensor.setLowTemp(22);      // Low temperature of 22C
  sensor.setHighHumidity(55); // High humidity of 55%
  sensor.setLowHumidity(40);  // Low humidity of 40%

  // Configure Measurements
  sensor.setMeasurementMode(TEMP_AND_HUMID); // Set measurements to temperature and humidity
  sensor.setRate(ONE_HZ);                    // Set measurement frequency to 1 Hz
  sensor.setTempRes(FOURTEEN_BIT);
  sensor.setHumidRes(FOURTEEN_BIT);

  //begin measuring
  sensor.triggerMeasurement();
}

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(9, HIGH);
  Serial.begin(115200);
  setupTempSensor();
  rtcSetup();
  flashSetup();
}

// the loop function runs over and over again forever
auto num = 0;
static long lastmilisLED;
static long lastmilisTEMP;
void loop()
{

  flash.writeULong(0, 512, true);

  delay(1000);
  auto currentmillis = millis();
  // Non-Blocking toggling of LED pins using PINx Register
  if (currentmillis - lastmilisLED >= LED_Blink_rate)
  {
    PINB = 0b11;
    lastmilisLED = millis();
  }
  if (currentmillis - lastmilisTEMP >= TEMP_READ_rate)
  {
    Serial.print("Temperature (C): ");
    Serial.print(sensor.readTemp());
    Serial.print("\t\tHumidity (%): ");
    Serial.println(sensor.readHumidity());

    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    Serial.print("Start Address: ");
    Serial.println(strAddr);
    flash.eraseSector(0);
    if (!flash.writeULong(0, strAddr))
    {
      Serial.println("StrAddr write fail");
    }
    
    if (!flash.writeShort(sizeof(strAddr) + 1, 1, true))
    {
      Serial.println("data in flash write fail");
    }

    
    if (!flash.writeShort(strAddr + ADDS_OFFSET, now.year()))
    {
      Serial.println("year write fail");
    }
    
    if (!flash.writeByte(strAddr + 2 + ADDS_OFFSET, now.month()))
    {
      Serial.println("month write fail");
    }
    
    if (!flash.writeByte(strAddr + 3 + ADDS_OFFSET, now.day()))
    {
      Serial.println("day write fail");
    }
    
    if (!flash.writeByte(strAddr + 4 + ADDS_OFFSET, now.hour()))
    {
      Serial.println("hour write fail");
    }
    
    if (!flash.writeByte(strAddr + 5 + ADDS_OFFSET, now.minute()))
    {
      Serial.println("min write fail");
    }
    
    if (!flash.writeByte(strAddr + 6 + ADDS_OFFSET, now.second()))
    {
      Serial.println("sec write fail");
    }
    if (!flash.writeFloat(strAddr + 7 + ADDS_OFFSET, sensor.readTemp()))
    {
      Serial.println("sec write fail");
    }
    if (!flash.writeFloat(strAddr + 11 + ADDS_OFFSET, sensor.readHumidity()))
    {
      Serial.println("sec write fail");
    }
    strAddr += 15;
    lastmilisTEMP = millis();
  }
  if (strAddr == 5*7)
  {
 while (1)
 {
   /* code */
 }
 
    /* code */
  }
  
  // wait for a second

  // Blink LEDs alternativly every second
  // Read Tempetature and humidity every second
  // Save to flash
}