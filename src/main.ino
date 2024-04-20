#include <Arduino.h>
#include <SPI.h>
#include <config.h>
#include <ST7735.h>
#include <RDA5807.h> 
#include <AHTxx.h> 

#define SCR_WD   128
#define SCR_HT   64
float ahtValue;                               //to store T/RH result

AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR); //sensor address, sensor type
ST7735 lcd = ST7735(PIN_DC, PIN_RST, -1);
RDA5807 rx; 

void setup(void) 
{
//   Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL, 100000);
    lcd.init();
    lcd.fillScreen(BLUE);
    delay(1000);
    lcd.fillScreen(RED);
    delay(1000);
    lcd.fillRect(0, 0, 128, 64, GREEN);
    delay(1000);
    
    pinMode(PIN_KEY1, INPUT_PULLUP); // Arduino pin 4 - Seek station down
    pinMode(PIN_KEY2, INPUT_PULLUP); // Arduino pin 5 - Seek station up
    rx.setup(); // Starts the receiver with default parameters
    rx.setFrequency(10390); // Tunes in 103.9 MHz  - Switch to your local favorite station

    while (aht21.begin() != true)
    {
        delay(1000);
    } 
    ahtValue = aht21.readTemperature(); //read 6-bytes via I2C, takes 80 milliseconds

    Serial.print(F("Temperature...: "));
    
    if (ahtValue != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
    {
        Serial.print(ahtValue);
        Serial.println(F(" +-0.3C"));
    }
    else
    {

        if   (aht21.softReset() == true) Serial.println(F("reset success")); //as the last chance to make it alive
        else                             Serial.println(F("reset failed"));
    }
    delay(2000); //measurement with high frequency leads to heating of the sensor, see NOTE

    ahtValue = aht21.readHumidity(); //read another 6-bytes via I2C, takes 80 milliseconds

    Serial.print(F("Humidity......: "));
    
    if (ahtValue != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
    {
        Serial.print(ahtValue);
        Serial.println(F(" +-2%"));
    }
    else
    {
    }

}

void loop()
{
    if (digitalRead(PIN_KEY1) == LOW) rx.seek(RDA_SEEK_WRAP,RDA_SEEK_DOWN);
    if (digitalRead(PIN_KEY2) == LOW) rx.seek(RDA_SEEK_WRAP,RDA_SEEK_UP);
    delay(200);
}