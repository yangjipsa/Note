#include "ESP32_PINMAP.h"
#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <DHT11.h>
#include <math.h>

#define Fs 20.0 // float 40
#define D_Lth 256 // Dependent to Fs 512
#define MedFilterWinSize 21 // Dependent to Fs 41
#define NumCutSample 10 // Dependent to Fs 20
#define Z_Lth 2048 //(int)(Fs*60) //Fs*60 보다 큰 2의 거듭제곱수 4096


// #define Fs 30.0 // float 40
// #define D_Lth 384  // Dependent to Fs 512
// #define MedFilterWinSize 31 // Dependent to Fs 41
// #define NumCutSample 15 // Dependent to Fs 20
// #define Z_Lth 2048 //(int)(Fs*60) //Fs*60 보다 큰 2의 거듭제곱수 4096

// float b0 = 0.9391;
// float b1 = -1.8782;
// float b2 = 0.9391;
// float a1 = -1.8734;
// float a2 = 0.8784;

// 2차 Butterworth 하이패스 필터 계수 (Dependent to Fs and Fc(0.8Hz))
float b0 = 0.7548;
float b1 = -1.5096;
float b2 = 0.7548;
float a1 = -1.4461;
float a2 = 0.5741;

int DD_Lth = D_Lth-(NumCutSample*2);
int DD_Rng[2] = {NumCutSample, NumCutSample+DD_Lth-1};
float vReal[Z_Lth], vImag[Z_Lth];

#define SERIAL_DEBUG

#define SW1 pinL10 
#define SW2 pinL09
#define SW3 pinL12
#define SW4 pinL11

#define LED pinL13
#define BUZZER pinR10
#define TEMP_HUMI pinL15

#define PRESSURE_DOU  pinL07
#define PRESSURE_SCK  pinL08
#define VALVE_A       pinR08
#define VALVE_B       pinR09
#define PUMP_A        pinR15
#define PUMP_B        pinR16


ArduinoFFT<float> FFT;

LiquidCrystal_I2C lcd(0x3f, 16, 2); 
Adafruit_NeoPixel RGB_LED(1, LED, NEO_GRB + NEO_KHZ800);
DHT11 dht11(TEMP_HUMI);

float G = 0.1; // Light Gain

//for Switch
bool cState[4] = {true,true,true,true};
bool lState[4] = {true,true,true,true};
unsigned char SW[4] = {SW1, SW2, SW3, SW4};
bool setAirDone = false;
bool isPerson = false;

unsigned char PWM_VAl = 255;

int refSamples = 20;
long valPressure = 0;
long refPressure = 0;
float refGain = 1.1; // 20%
long minPress = 10000000;
long maxPress = 12000000;


long readPressure(int pinSCK, int pinVout);
void MedianFilter(float *Data, int DataSize, int WindowSize);
void AscendSort(float *Data, int ArrSize);
int DetectPPM();

void setup()
{
  Serial.begin(115200);

  pinMode(PRESSURE_SCK, OUTPUT);
  pinMode(PRESSURE_DOU, INPUT);

  pinMode(VALVE_A, OUTPUT);
  pinMode(VALVE_B, OUTPUT);
  pinMode(PUMP_A, OUTPUT);
  pinMode(PUMP_B, OUTPUT);
  //pinMode(BUZZER, OUTPUT);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);

  Serial.println("Start");

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("SIG Cooperation ");
  lcd.setCursor(0,1);
  lcd.print("IoT Sensor Pad  ");

  RGB_LED.begin();
  RGB_LED.clear();
  RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*0,G*0));
  RGB_LED.show();
  delay(1000);

  buzzerBeep(1000);

  RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*0,G*0)); 
  RGB_LED.show();
}
void loop()
{
  for (int i = 0 ; i < 4 ; i++)
  {
    lState[i] = cState[i]; // for update
    cState[i] = digitalRead(SW[i]); // read digitalvalue
  }

  if(setAirDone)
  {
    long tempPres = avgPressure();
    Serial.println(tempPres);
    if( tempPres > refGain*10000000 /*maxPress*/)
    {
      //Serial.println("Person!!");
      isPerson = true;
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*0,G*150)); //in operating
      RGB_LED.show();
    }
    else
    {
      isPerson = false;
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*0,G*0)); //in operating
      RGB_LED.show();
    }
  }
  //   RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*150,G*0)); //in operating
  //   RGB_LED.show();
  //   long PressVal = readPressure(PRESSURE_SCK, PRESSURE_DOU); 
  //   if(PressVal >= refGain * refPressure)
  //   {
  //     Serial.print("in IF : "); Serial.println(PressVal);
  //     long temp1 = avgPressure();
  //     //long temp2 = avgPressure();
  //     //long temp3 = avgPressure();
  //     if( temp1 > refGain*refPressure)
  //     {
  //       Serial.println("Person!!");
  //       isPerson = true;
  //     }
  //     else
  //     {
  //       Serial.println("miss...");
  //     }
  //   }
  //   else if ((PressVal <= 1.05*refPressure) && (PressVal >= 0.95*refPressure))
  //   {
      
  //     Serial.print("in IF2 : "); Serial.println(PressVal);
  //     long temp1 = avgPressure();
  //     //long temp2 = avgPressure();
  //     //long temp3 = avgPressure();
  //     long tempAvg = temp1; //(temp1+temp2+temp3) / 3;
  //     if((tempAvg <= 1.05*refPressure) && (tempAvg >= 0.95*refPressure))
  //     {
  //       Serial.println("Person is not Here");
  //       isPerson = false;
  //     }
  //     else
  //     {
  //       Serial.println("miss...");
  //     }
  //   }
  //   RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*150,G*0)); //in operating
  //   RGB_LED.show();
  // }

  if( cState[0] < lState[0] ) // measure heart rate
  { // cal Heart Rate
    buzzerBeep(250);

    int Idx, brpm, bpm;
    float D[D_Lth], FiltD[D_Lth], SumX = 0, SumY = 0, SumXY = 0, SumXX = 0, Slope, Intercept;
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0; 

    //check person on pad
    if(setAirDone && isPerson)
    {
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*150,G*0)); //in operating
      RGB_LED.show();
      lcd.setCursor(0,0); lcd.print("*IoT Sensor Pad*");
      lcd.setCursor(0,1); lcd.print("Measuring Data..");

      unsigned long previousMillis = 0; 
      const long interval = 1000/Fs; //50; // 50 ms interval for 20Hz 
      int dataCount = 0; 
      
      while (dataCount < D_Lth) 
      { // D_Lth * interval = 512 * 50 = 25600ms = 25.6sec
        unsigned long currentMillis = millis(); 
        if (currentMillis - previousMillis >= interval) 
        { 
          previousMillis = currentMillis; 
          D[dataCount] = readPressure(PRESSURE_SCK, PRESSURE_DOU); 
#ifdef SERIAL_DEBUG
          Serial.println( D[dataCount]);
#endif          
          dataCount++; 
        } 
      }

    while(1)
    {
      float AbsDD[DD_Lth], SortDD[DD_Lth], MAD;
      bool BreakFlag = 1;

      for (int i = 0; i<D_Lth; i++)
      {
        FiltD[i] = D[i];        
      }

      MedianFilter(FiltD, D_Lth, MedFilterWinSize);

      Idx = 0;

      for (int i = DD_Rng[0]; i<=DD_Rng[1]; i++)
      {
        AbsDD[Idx] = abs(D[i]-FiltD[i]);
        SortDD[Idx] = AbsDD[Idx];
        Idx++;
      }

      AscendSort(SortDD, DD_Lth);
      MAD = SortDD[(int)round(DD_Lth*0.90)]*3;

      Idx = 0;

      for (int i = DD_Rng[0]; i<=DD_Rng[1]; i++)
      {
        if (MAD<AbsDD[Idx])
        {
          D[i] = FiltD[i];
          BreakFlag = 0;
        }

        Idx++;
      }

      if (BreakFlag==1)
      {
        break;
      } 
    }
//-------------------Breath Rate---------------------------
      
    lcd.setCursor(0,1); lcd.print("Analyzing Data...");
    for (int i = DD_Rng[0]; i<=DD_Rng[1]; i++)
    {
      SumX += i;
      SumY += FiltD[i];
      SumXY += i*FiltD[i];
      SumXX += i*i;
    }
    lcd.setCursor(0,1); lcd.print("STEP1           ");
    Slope = (DD_Lth*SumXY-SumX*SumY)/(DD_Lth*SumXX-SumX*SumX);
    Intercept = (SumY-Slope*SumX)/DD_Lth;

    Idx = 0;

    for (int i = DD_Rng[0]; i<=DD_Rng[1]; i++)
    {
      vReal[Idx] = FiltD[i]-(Slope*Idx+Intercept);
      vImag[Idx] = 0;
      Idx++;
    }
    lcd.setCursor(0,1); lcd.print("STEP2           ");
    brpm = DetectPPM(); 
    lcd.setCursor(0,1); lcd.print("STEP3           ");

//-------------------Heart Rate---------------------------

for (int i = 0; i<D_Lth; i++)
  {
    D[i] = D[i]-FiltD[i];
  }

  MedianFilter(D, D_Lth, 5);  

  for (int i = 1; i<D_Lth; i++)
  {
    FiltD[i] = b0*D[i]+b1*x1+b2*x2-a1*y1-a2*y2;
    x2 = x1;
    x1 = D[i];
    y2 = y1;
    y1 = FiltD[i];
  }

  Idx = 0;

  for (int i = DD_Rng[0]; i<=DD_Rng[1]; i++)
  {
    vReal[Idx] = FiltD[i];
    vImag[Idx] = 0;
    Idx++;
  }

  bpm = DetectPPM(); 


      lcd.setCursor(0,0); lcd.print("B_rpm :             ");
      lcd.setCursor(0,1); lcd.print("H_rpm :             ");

      char BtempBuff[10];
      itoa(brpm, BtempBuff, 10);
      lcd.setCursor(8,0); lcd.print(BtempBuff);

      char HtempBuff[10];
      itoa(bpm, HtempBuff, 10);
      lcd.setCursor(8,1); lcd.print(HtempBuff);

      RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*150,G*0)); //done
      RGB_LED.show();

      buzzerBeep(1000);
    }
    else
    {
      lcd.setCursor(0,0); lcd.print("Check AirTube or");
      lcd.setCursor(0,1); lcd.print("No Person ......");
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*0,G*0)); //error
      RGB_LED.show();
    }
  }

  if( cState[1] < lState[1] ) // test SW
  { // set Air tube 

    buzzerBeep(250);
    if(isPerson == false)
    {
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*150,G*0)); //in operating
      RGB_LED.show();
      lcd.setCursor(0,0);
      lcd.print("*** Air Tube ***");
      lcd.setCursor(0,1);
      lcd.print("    in Operation");
      
      delay(1000);
      long pressVal = 0;
      // long minPress = 10000000;
      // long maxPress = 12000000;

      for (int i=0 ; i<100 ; i++) 
      {
        pressVal = readPressure(PRESSURE_SCK, PRESSURE_DOU);
  #ifdef SERIAL_DEBUG
        Serial.println(pressVal);
  #endif
        delay(10);
      }

      while( pressVal < minPress)
      {
        //digitalWrite(dcPump, HIGH);
        digitalWrite(PUMP_A, HIGH);
        digitalWrite(PUMP_B, LOW);
        pressVal = readPressure(PRESSURE_SCK, PRESSURE_DOU); 
        delay(50);
  #ifdef SERIAL_DEBUG      
        Serial.println(pressVal);
  #endif
      }
      digitalWrite(PUMP_A, LOW);
      digitalWrite(PUMP_B, LOW);

      while( pressVal > maxPress)
      {
        digitalWrite(VALVE_A, HIGH);
        digitalWrite(VALVE_B, LOW);
        delay(100);
        digitalWrite(VALVE_A, LOW);
        digitalWrite(VALVE_B, LOW);
        delay(500);
        pressVal = readPressure(PRESSURE_SCK, PRESSURE_DOU); delay(10);
        Serial.println(pressVal);
      }

      setAirDone = true;
      lcd.setCursor(0,1);
      lcd.print("        complete");
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*150,G*0)); // green
      RGB_LED.show();
      buzzerBeep(1000);
    }
    else
    {
      lcd.setCursor(0,0);
      lcd.print("*** Air Tube ***");
      lcd.setCursor(0,1);
      lcd.print("Person is on pad");
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*0,G*0)); // green
      RGB_LED.show();
    }
  }

  if( cState[2] < lState[2] ) // test SW
  { // pull out air
    buzzerBeep(250);
    RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*150,G*0)); //in operating
    RGB_LED.show();
    lcd.setCursor(0,0);
    lcd.print("*** Air Valve **");
    lcd.setCursor(0,1);
    lcd.print("      OPEN      ");

    digitalWrite(VALVE_A, HIGH);
    digitalWrite(VALVE_B, LOW);
    delay(500);
    digitalWrite(VALVE_A, LOW);
    digitalWrite(VALVE_B, LOW);
    delay(500);

    lcd.setCursor(0,1);
    lcd.print("     CLOSED     ");

    setAirDone= false;

    RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*150,G*0)); //in operating
    RGB_LED.show();
    // if(setAirDone)
    // {
    //   RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*150,G*0)); // green
    //   RGB_LED.show();
    // }
   // buzzerBeep(1000);
  }
  if( cState[3] < lState[3] ) // test SW
  {  // stop
    buzzerBeep(250);
   // RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*0,G*0)); // yellow
   // RGB_LED.show();
   // delay(500);

    int temperature = 0;
    int humidity = 0;

    // Attempt to read the temperature and humidity values from the DHT11 sensor.
    int result = dht11.readTemperatureHumidity(temperature, humidity);
    delay(100);

    if (result == 0) {
        char tempBuf[10];

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("TEMP:");
        itoa(temperature, tempBuf, 10);
        lcd.print(tempBuf);
        lcd.print(" C");
        lcd.setCursor(0,1);
        lcd.print("HUMI:");
        itoa(humidity, tempBuf, 10);
        lcd.print(tempBuf);
        lcd.print(" %");
    } else {
        // Print error message based on the error code.
        Serial.println(DHT11::getErrorString(result));
    }
    // if(setAirDone)
    // {
    //   RGB_LED.setPixelColor(0, RGB_LED.Color(G*0,G*150,G*0)); // green
    //   RGB_LED.show();
    // }
    //buzzerBeep(1000);
  }
}

long readPressure(int pinSCK, int pinVout)
{
  long result = 0;
  for (int i = 0; i < 24; i++) 
  {
    digitalWrite(pinSCK, HIGH);
    digitalWrite(pinSCK, LOW);
    result = result << 1;
    if (digitalRead(pinVout)) result++;
  }

  // get the 2s compliment
  result = result ^ 0x800000;

  // pulse the clock line 3 times to start the next pressure reading
  for (char i = 0; i < 3; i++) 
  {
    digitalWrite(pinSCK, HIGH);
    digitalWrite(pinSCK, LOW);
  }

  return result;
}

long avgPressure(void)
{
  long tempPress[refSamples];
  long avgPress = 0;

  for (int i = 0 ; i < refSamples ; i++)
  {
    tempPress[i] = readPressure(PRESSURE_SCK, PRESSURE_DOU); 
    avgPress = avgPress + tempPress[i]/refSamples;
    delay(10); // total 10 sec
  }
  return avgPress;
}


void buzzerBeep(int T)
{ 
  analogWrite(BUZZER, 150);
  delay(T);
  analogWrite(BUZZER, 0);
}

//-----------20250626

void MedianFilter(float *Data, int DataSize, int WindowSize)
{
  float OriData[DataSize], WindowData[WindowSize];
  int HalfWindowSize = (WindowSize-1)/2, WindowIdx;

  for (int i = 0; i<DataSize; i++)
  {
    OriData[i] = Data[i];
  }

  for (int i = 0; i<DataSize; i++)
  {
    WindowIdx = 0;

    for (int j = i-HalfWindowSize; j<=i+HalfWindowSize; j++)
    {
      if ((j<0)||(DataSize<=j))
      {
        WindowData[WindowIdx] = 0;
      }
      else
      {
        WindowData[WindowIdx] = OriData[j];
      }

      WindowIdx++;
    }

    AscendSort(WindowData, WindowSize);
    Data[i] = WindowData[HalfWindowSize];
  }
}

void AscendSort(float *Data, int ArrSize)
{
  for (int i = 0; i<(ArrSize-1); i++)
  {
    for (int j = 0; j<(ArrSize-1-i); j++)
    {
      if (Data[j]>Data[j+1])
      {
        float Temp = Data[j];
        Data[j] = Data[j+1];
        Data[j+1] = Temp;
      }
    }
  }
}

int DetectPPM()
{
  int PeakIdx = 0;
  float F, Magnitude, PeakVal = 0;

  for (int i = DD_Lth; i<Z_Lth; i++)
  {
    vReal[i] = 0;    
    vImag[i] = 0;
  }
  lcd.setCursor(0,1); lcd.print("STEP2.1         ");
  FFT.windowing(vReal, Z_Lth, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  lcd.setCursor(0,1); lcd.print("STEP2.2         ");
  FFT.compute(vReal, vImag, Z_Lth, FFT_FORWARD); 
  lcd.setCursor(0,1); lcd.print("STEP2.3         ");
  for (int i = 0; i<Z_Lth; i++)
  {
    F = (float)i*Fs/(float)Z_Lth;
    Magnitude = sqrt(vReal[i]*vReal[i]+vImag[i]*vImag[i]);

    if (F>5.0)
    {
      lcd.setCursor(0,1); lcd.print("STEP2.4         ");
      delay(1000);     
      return ((float)PeakIdx * Fs * 60.0)/ (float)Z_Lth;
     //return 1000;
   
    }

    if (PeakVal<Magnitude)
    {
      PeakVal = Magnitude;
      PeakIdx = i;
    }
  }

  //return 1000;
}
