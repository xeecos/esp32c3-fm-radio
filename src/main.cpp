#include <Arduino.h>
#include <config.h>
#include <st7735.h>
#include "driver/adc.h"
#include "arduinoFFT.h"

#define TIMES              128

const uint16_t samples = TIMES; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 22000;
const uint8_t amplitude = 100;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

ST7735 lcd = ST7735(PIN_DC, PIN_RST, -1);
uint16_t *bmp1;
uint16_t *bmp2;

uint32_t idx = 0;
uint16_t result[TIMES] = {0};
void readADC(void*params)
{
    while(1)
    {
        idx++;
        if(idx>=TIMES)
        {
            // FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();
            delay(15);
            idx = 0;
        }
        vReal[idx] = analogRead(PIN_ADC);
        vImag[idx] = 0;
        delayMicroseconds(5);
    }
}

template <typename T> static inline void swap(T& a, T& b) { T t = a; a = b; b = t; }
void drawLine(uint16_t* bmp, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    if(y0 == y1)
    {
        for (; x0 <= x1; x0++) {
            bmp[y0*128+x0] = color;
        }
    }
    else if(x0 == x1)
    {
        if(y0>y1)
        {
            swap(y0, y1);
            swap(x0, x1);
        }
        for (; y0 <= y1; y0++) {
            bmp[y0*128+x0] = color;
        }
    }
    else
    {
        int16_t steep = abs(y1 - y0) > abs(x1 - x0);
        if (steep) {
            swap(x0, y0);
            swap(x1, y1);
        }
        if (x0 > x1) {
            swap(x0, x1);
            swap(y0, y1);
        }
        int16_t dx, dy;
        dx = x1 - x0;
        dy = abs(y1 - y0);

        int16_t err = dx / 2;
        int16_t ystep;

        if (y0 < y1) {
            ystep = 1;
        } else {
            ystep = -1;
        }

        for (; x0 <= x1; x0++) {
            if (steep) {
                bmp[x0*128+y0] = color;
            } else {
                bmp[y0*128+x0] = color;
            }
            err -= dy;
            if (err < 0) {
                y0 += ystep;
                err += dx;
            }
        }
    }
}
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
    
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate( readADC, "readADC", 1024, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
}

void loop()
{

    // adc_digi_read_bytes((uint8_t*)result, TIMES, &ret_num, ADC_MAX_DELAY);
    // USBSerial.printf("adc:%d\n",analogRead(PIN_ADC));
    // digitalWrite(PIN_LED, HIGH);
    memset(bmp1, 0 ,128*64*2);
    if(idx>=TIMES)
    {
        idx = 0;
        for(int i = 0; i < TIMES; i++)
        {
            // USBSerial.printf("adc:%f\n",vReal[i]);

            int ii = idx+i;
            if(ii>=TIMES)
            {
                ii -= TIMES;
            }
            if(vReal[ii] > 0)
            {
                int x0 = ii;
                int y0 = (0.0+vReal[ii]/2048*64.0-0.0)*0.5;
                if(y0<0)y0=0;
                if(y0>63)y0=63;
                if(ii>0)
                {
                    int iii = idx+i-1;
                    if(iii>=TIMES)
                    {
                        iii -= TIMES;
                    }
                    if(iii<0)
                    {
                        iii += TIMES;
                    }
                    int x1 = iii;
                    int y1 = (0.0+vReal[iii]/2048*64.0-0.0)*0.5;
                    if(y1<0)y1=0;
                    if(y1>63)y1=63;
                    // x1 -= 64;
                    // x0 -= 64;
                    // if(x1<0)x1+=128;
                    // if(x0<0)x0+=128;
                    drawLine(bmp1, x1, y1, x0, y0, 0xffff);
                }
                else
                {
                    int iidx = (y0*128+x0);
                    bmp1[iidx] = 0xffff;
                }
            }
        }    
    }
    // drawLine(bmp1, 64, 0, 32, 63, 0xffff);
    // drawLine(bmp1, 64, 0, 96, 63, 0xffff);
    // drawLine(bmp1, 0, 32, 128, 32, 0xffff);
    // drawLine(bmp1, 64, 0, 64, 63, 0xffff);
    // drawLine(bmp1,  32, 0,64, 63, 0xffff);
    // drawLine(bmp1,  96, 0,64, 63, 0xffff);
    lcd.drawImage(0,0,128,64,bmp1);
    delay(30);

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