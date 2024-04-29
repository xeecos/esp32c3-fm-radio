#include <Arduino.h>
#include <config.h>
#include <st7735.h>
#include "driver/adc.h"
#include "arduinoFFT.h"
#include <Wire.h>
#include <RDA5807.h> 
// #define RDA_INIT_VOL    3             // volume on system start (0..15)
// #define RDA_INIT_FREQ   97.10        // channel frequency on system start

#define TIMES              128

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 22100;


/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);
ST7735 lcd = ST7735(PIN_DC, PIN_RST, -1);
RDA5807 rx;
uint16_t *bmp1;
uint16_t *bmp2;

uint32_t idx = 0;
uint16_t result[TIMES] = {0};

void readADC(void*params)
{
    while(1)
    {
        if(idx>=samples)
        {
            // FFT.windowing(FFTWindow::Blackman, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();
            idx = 0;
            delay(20);
        }
        vReal[idx] = analogRead(PIN_ADC);
        vImag[idx] = 0;
        idx++;
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
int fidx = 0;
int list[6] = {8980,9420,9710,9910,10570,10620};
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
    pinMode(PIN_KEY1, INPUT_PULLUP);
    pinMode(PIN_KEY2, INPUT);
    pinMode(PIN_PWM, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    digitalWrite(PIN_PWM, HIGH);
    lcd.begin();
    ledcSetup(0, 32768, 8);
    ledcAttachPin(PIN_LED, 0);
    ledcWrite(0, 128);
    Wire.begin(PIN_SDA, PIN_SCL, 100000);

    rx.setup(CLOCK_32K, OSCILLATOR_TYPE_PASSIVE); // Starts the receiver with default parameters
    
    // while(1)
    {
        USBSerial.printf("id:%x\n",rx.getDeviceId());
        delay(1000);
    }
    // rx.setRDS(true);
    // rx.setRdsFifo(true);
    rx.setMono(false);
    rx.setVolume(2);
    rx.setBand(RDA_FM_BAND_WORLD);
    // rx.setAFC(true);
    // rx.setChannel(187);
    rx.setFrequency(8980); // Tunes in 97.1 MHz  - Switch to your local favorite station

    // rx.setSeekThreshold(10);
    // rx.seek(1,0);
    // rx.setVolumeUp();
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate( readADC, "readADC", 1024, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
}
long now = 0;
int key1 = 1;
int key2 = 1;
void loop()
{

    // adc_digi_read_bytes((uint8_t*)result, TIMES, &ret_num, ADC_MAX_DELAY);
    // USBSerial.printf("adc:%d\n",analogRead(PIN_ADC));
    // digitalWrite(PIN_LED, HIGH);
    if(millis()-now>1000){
        now = millis();
        USBSerial.printf("freq:%d\n",rx.getRealFrequency());//,digitalRead(PIN_KEY1),digitalRead(PIN_KEY2));
    }
    if(key1!=digitalRead(PIN_KEY1))
    {
        key1 = digitalRead(PIN_KEY1);
        if(key1==0)
        {
            int freq = rx.getFrequency();
            fidx-=1;
            if(fidx<0)fidx=5;
            rx.setFrequency(list[fidx]);
        }
    }
    if(key2!=digitalRead(PIN_KEY2))
    {
        key2 = digitalRead(PIN_KEY2);
        if(key2==0)
        {
            int freq = rx.getFrequency();
            fidx+=1;
            if(fidx>5)fidx=0;
            rx.setFrequency(list[fidx]);
        }
    }
    if(idx==0)
    {
        memset(bmp1, 0 ,128*64*2);
        for(int i = 0; i < TIMES; i++)
        {
            // USBSerial.printf("adc:%f\n",vReal[i]);

            int ii = i;
            if(vReal[ii] > 0)
            {
                int x0 = ii;
                int y0 = (0.0+vReal[ii]/2048*64.0-0.0)*0.5;
                if(y0<0)y0=0;
                if(y0>63)y0=63;
                if(ii>0)
                {
                    int iii = i-1;
                    int x1 = iii;
                    int y1 = (0.0+vReal[iii]/2048*64.0-0.0)*0.5;
                    if(y1<0)y1=0;
                    if(y1>63)y1=63;
                    // x1 -= 64;
                    // x0 -= 64;
                    // if(x1<0)x1+=128;
                    // if(x0<0)x0+=128;
                    drawLine(bmp1, x1, y1, x1, 0, lcd.color565(128-iii,64+iii,iii));
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
// #include <AHTxx.h> 

// #define SCR_WD   128
// #define SCR_HT   64
// float ahtValue;                               //to store T/RH result

// AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR); //sensor address, sensor type
// 
// 

// void setup(void) 
// {
// //   Serial.begin(115200);
//     pinMode(PIN_ADC, INPUT);
//     analogRead(PIN_ADC);
    
//     pinMode(PIN_KEY1, INPUT_PULLUP); // Arduino pin 4 - Seek station down
//     pinMode(PIN_KEY2, INPUT_PULLUP); // Arduino pin 5 - Seek station up

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