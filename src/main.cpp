
// #include <Adafruit_GFX.h>    // Core graphics library
// #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
// #include <SPI.h>
// #include <config.h>
// #define TFT_CS          -1
// #define TFT_RST         PIN_RST
// #define TFT_DC          PIN_DC
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, PIN_SDATA, PIN_SCLK, TFT_RST);

//   tft.initR(INITR_GREENTAB);
// }
// void drawImage(uint16_t*bmp)
// {
//     tft.startWrite();
//     tft.setAddrWindow(0, 0, 128, 64);
//     tft.writePixels(bmp, 128*64, true, false);
//     tft.endWrite();
// }
// void loop() 
// {
//     // drawImage(bmp1);
//     // delay(1000);
//     // drawImage(bmp2);
//     // delay(1000);
//     // tft.fillScreen(ST7735_RED);
//     // delay(1000);
// }
#include <Arduino.h>
#include <config.h>
#include <st7735.h>
ST7735 lcd = ST7735(PIN_DC, PIN_RST, -1);
uint16_t *bmp1;
uint16_t *bmp2;
void setup(void) 
{
    bmp1 = (uint16_t*)malloc(128*64*2);
    bmp2 = (uint16_t*)malloc(128*64*2);
    for(int i=0;i<128*64;i++)
    {
        bmp1[i] = 0xa416;
        bmp2[i] = 0x2A84;
    }
    USBSerial.begin(115200);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_ADC, INPUT);
    pinMode(PIN_PWM, OUTPUT);
    digitalWrite(PIN_PWM, HIGH);
    lcd.begin();
}
void loop()
{
    // USBSerial.printf("adc:%d\n",analogRead(PIN_ADC));
    // digitalWrite(PIN_LED, HIGH);
    
    lcd.drawImage(0,0,128,64,bmp1);
    delay(1000);
    lcd.drawImage(0,0,128,64,bmp2);
    delay(1000);
    lcd.fillRect(0, 0, 128, 64, GREEN);
    delay(1000);
    // digitalWrite(PIN_LED, LOW);
    // delayMicroseconds(100);
}
// #include <RDA5807.h> 
// #include <AHTxx.h> 

// #define SCR_WD   128
// #define SCR_HT   64
// float ahtValue;                               //to store T/RH result

// AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR); //sensor address, sensor type
// 
// RDA5807 rx; 

// void setup(void) 
// {
// //   Serial.begin(115200);
//     pinMode(PIN_ADC, INPUT);
//     analogRead(PIN_ADC);
//     Wire.begin(PIN_SDA, PIN_SCL, 100000);
    
//     pinMode(PIN_KEY1, INPUT_PULLUP); // Arduino pin 4 - Seek station down
//     pinMode(PIN_KEY2, INPUT_PULLUP); // Arduino pin 5 - Seek station up
//     rx.setup(); // Starts the receiver with default parameters
//     rx.setFrequency(10390); // Tunes in 103.9 MHz  - Switch to your local favorite station

//     while (aht21.begin() != true)
//     {
//         delay(1000);
//     } 
//     ahtValue = aht21.readTemperature(); //read 6-bytes via I2C, takes 80 milliseconds

//     Serial.print(F("Temperature...: "));
    
//     if (ahtValue != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
//     {
//         Serial.print(ahtValue);
//         Serial.println(F(" +-0.3C"));
//     }
//     else
//     {

//         if   (aht21.softReset() == true) Serial.println(F("reset success")); //as the last chance to make it alive
//         else                             Serial.println(F("reset failed"));
//     }
//     delay(2000); //measurement with high frequency leads to heating of the sensor, see NOTE

//     ahtValue = aht21.readHumidity(); //read another 6-bytes via I2C, takes 80 milliseconds

//     Serial.print(F("Humidity......: "));
    
//     if (ahtValue != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
//     {
//         Serial.print(ahtValue);
//         Serial.println(F(" +-2%"));
//     }
//     else
//     {
//     }

// }

// void loop()
// {
//     if (digitalRead(PIN_KEY1) == LOW) rx.seek(RDA_SEEK_WRAP,RDA_SEEK_DOWN);
//     if (digitalRead(PIN_KEY2) == LOW) rx.seek(RDA_SEEK_WRAP,RDA_SEEK_UP);
//     delay(200);
// }