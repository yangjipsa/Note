#include "ESP32_PINMAP.h"
#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <DHT11.h>

//#define SERIAL_DEBUG

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

#define Fs 20
//#define D_Lth 512
#define D_Lth 256
#define Z_Lth 2048
#define F_EndIdx 513
#define LimCoeff 1.5
#define HalfMedWinSize 10

const int bpmHzRngIdx[2] = {81, 341}; // (47bpm to 200bpm) 
int PeakIdx;
float PeakVal;
int brpm;
int bpm;
float MedValD;
float UpperVal;
float LowerVal;
float UpperLim;
float LowerLim;

//float F[F_EndIdx];
float F[Z_Lth];
float D[D_Lth];
float MedFiltD[D_Lth];
float MedFiltD2[D_Lth];
float tempMedFiltD[D_Lth];
float SigTF[Z_Lth], vImag[Z_Lth];  


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

    //check person on pad
    if(setAirDone && isPerson)
    {
      RGB_LED.setPixelColor(0, RGB_LED.Color(G*150,G*150,G*0)); //in operating
      RGB_LED.show();
      lcd.setCursor(0,0); lcd.print("*IoT Sensor Pad*");
      lcd.setCursor(0,1); lcd.print("Measuring Data..");
      for (int i = 0; i < F_EndIdx; i++) 
      { 
        F[i] = (float)i * (float)Fs / (float)Z_Lth; 
#ifdef SERIAL_DEBUG
        //Serial.println(F[i]);
#endif
      }

      unsigned long previousMillis = 0; 
      const long interval = 50; // 50 ms interval for 20Hz 
      int dataCount = 0; 
      
      while (dataCount < D_Lth) 
      { // D_Lth * interval = 512 * 50 = 25600ms = 25.6sec
        unsigned long currentMillis = millis(); 
        if (currentMillis - previousMillis >= interval) 
        { 
          previousMillis = currentMillis; 
          D[dataCount] = readPressure(PRESSURE_SCK, PRESSURE_DOU); 
#ifdef SERIAL_DEBUG
        // Serial.println( D[dataCount]);
#endif          
          dataCount++; 
        } 
      }
//-------------------Breath Rate---------------------------
      for (int i = 0; i < D_Lth; i++)
      {
        int SttArrIdx = i - HalfMedWinSize;
        int EndArrIdx = i + HalfMedWinSize;
        int ArrIdxSize;

        if (SttArrIdx < 0)
        {
          SttArrIdx = 0;
          ArrIdxSize = EndArrIdx + 1;
        }
        else if (EndArrIdx >= D_Lth)
        {
          EndArrIdx = D_Lth - 1;
          ArrIdxSize = EndArrIdx - SttArrIdx+1;
        }
        else
        {
          ArrIdxSize = Fs + 1;
        }
        
        //ArrIdxSize = 1;

        float TempArr[ArrIdxSize];
        int k = 0;
#ifdef SERIAL_DEBUG 
        // Serial.println("ArrIdxSize : " + String(ArrIdxSize));
        // Serial.println("Index Range : "+ String(SttArrIdx) + " to " + String(EndArrIdx));
        // Serial.print(" - TempArr: ");
#endif
        for (int j = SttArrIdx; j <= EndArrIdx; j++)
        //for (int j = i; j <= i; j++)
        {
          TempArr[k] = D[j];
          k++;
#ifdef SERIAL_DEBUG         
          // Serial.print(String(TempArr[k]) + ", ");
#endif
        }
#ifdef SERIAL_DEBUG       
        // Serial.println();
#endif
        MedFiltD[i] = findMedian(TempArr, ArrIdxSize);
        tempMedFiltD[i] = MedFiltD[i];
#ifdef SERIAL_DEBUG       
        // Serial.println(" - MedFiltD : "+ String(MedFiltD[i]));
#endif
      }

     MedValD = findMedian(tempMedFiltD, D_Lth);
#ifdef SERIAL_DEBUG
        //  Serial.println("=================D==============="); 
        //  for(int i= 0 ; i<D_Lth ; i++)  Serial.println(D[i]);
        //  Serial.println("===============MedFiltD================");
        //  for(int i= 0 ; i<D_Lth ; i++)  Serial.println(MedFiltD[i]);
        //  Serial.println("==================================");
#endif
    #ifdef SERIAL_DEBUG
      //Serial.println(MedValD);
    #endif

      for (int i = 0; i<D_Lth; i++) // delete bias
      {
        MedFiltD[i] = MedFiltD[i] - MedValD;
        D[i] = D[i] - MedValD;
      }

      UpperVal = MedFiltD[0];
      LowerVal = MedFiltD[0];  

      for (int i = 1; i<D_Lth; i++) // find Max value from filtered Value
      {
        if (UpperVal<MedFiltD[i])
        {
          UpperVal = MedFiltD[i];
        }
        
        if (LowerVal>MedFiltD[i])
        {
          LowerVal = MedFiltD[i];
        }
      }

      UpperLim = UpperVal * LimCoeff;
      LowerLim = LowerVal * LimCoeff;

      for (int i = 0; i<D_Lth; i++)
      {
        if (UpperLim<D[i])
        {
          D[i] = UpperLim;
        }
        else if (D[i]<LowerLim)
        {
          D[i] = LowerLim;
        }
      }

      detrendData(MedFiltD,MedFiltD2,D_Lth);

      for (int i = 0; i<D_Lth; i++) // to D_Lth, real data
      {
        //SigTF[i] = D[i];
        SigTF[i] = MedFiltD2[i];
        vImag[i] = 0;
      }

#ifdef SERIAL_DEBUG
         Serial.println("==================================");
         for(int i= 0 ; i<D_Lth ; i++)  Serial.println(D[i]);
         Serial.println("==================================");
         for(int i= 0 ; i<D_Lth ; i++)  Serial.println(MedFiltD[i]);
         Serial.println("===============MedFiltD2=============");
         for(int i= 0 ; i<D_Lth ; i++)  Serial.println(MedFiltD2[i]);
         Serial.println("==================================");
#endif 

      for (int i = D_Lth; i<Z_Lth; i++) // D_Lth to Z_Lth, set zeros
      {
        SigTF[i] = 0;    
        vImag[i] = 0;
      }

      lcd.setCursor(0,1); lcd.print("Analyzing Data...");
      
      //FFT
      //FFT.windowing(SigTF, Z_Lth, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);  
      FFT.windowing(SigTF, Z_Lth, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
      FFT.compute(SigTF, vImag, Z_Lth, FFT_FORWARD);

      PeakIdx = 0;
      PeakVal = 0;

      for (int i = 0; i<F_EndIdx; i++)
      {
        //double magnitude = sqrt(SigF[i]*SigF[i]+vImag[i]*vImag[i]);
        double magnitude = sqrt(SigTF[i]*SigTF[i]+vImag[i]*vImag[i]);

        if (PeakVal < magnitude)
        {
          PeakVal = magnitude;
          PeakIdx = i;
        }
      }

      brpm = F[PeakIdx]*60;

//-------------------Heart Rate---------------------------
      for (int i = 0; i<D_Lth; i++)
      {
        SigTF[i] = D[i] - MedFiltD[i];
        vImag[i] = 0;
      }
      
      for (int i = D_Lth; i<Z_Lth; i++)
      {
        SigTF[i] = 0;
        vImag[i] = 0;
      }

      //FFT.windowing(SigTF, Z_Lth, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);  
      FFT.windowing(SigTF, Z_Lth, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
      FFT.compute(SigTF, vImag, Z_Lth, FFT_FORWARD);

      PeakIdx = bpmHzRngIdx[0];
      PeakVal = 0;

      //for (int i = bpmHzRngIdx[1]; i<bpmHzRngIdx[2]; i++)
//      for (int i = bpmHzRngIdx[1]; i<F_EndIdx; i++)
      for (int i = bpmHzRngIdx[0]; i<bpmHzRngIdx[1]; i++)
      {
        //double magnitude = sqrt(SigTF[i]*SigTF[i]+vImag[i]*vImag[i]);
        double magnitude = sqrt((double)SigTF[i]*(double)SigTF[i]+(double)vImag[i]*(double)vImag[i]);

        if (PeakVal<magnitude)
        {
          PeakVal = magnitude;
          PeakIdx = i;
        }
      }

      bpm = F[PeakIdx]*60;

#ifdef SERIAL_DEBUG
      Serial.print("brpm : ");
      Serial.println(brpm);

      Serial.print("bpm : ");
      Serial.println(bpm);
#endif

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

float findMedian(float arr[], int n) // for DC bias
{     
  for (int i = 0; i<(n-1); i++)
  {
    for (int j = 0; j<(n-i-1); j++)
    {
      if (arr[j]>arr[j+1])
      {
        float temp = arr[j];
        arr[j] = arr[j+1];
        arr[j+1] = temp;
      }
    }
  }
  
  if ((n%2)!=0) 
  { 
    return arr[(n+1)/2 - 1];
  }
  else 
  {
    return (arr[(n/2)-1] + arr[n/2])/2; 
  }  
}


void detrendData(float* inputData, float* outputData, int size) {
  float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
  
  for (int i = 0; i < size; i++) {
    sumX += i;
    sumY += inputData[i];
    sumXY += i * inputData[i];
    sumXX += i * i;
  }

  float slope = (size * sumXY - sumX * sumY) / (size * sumXX - sumX * sumX);
  float intercept = (sumY - slope * sumX) / size;

  for (int i = 0; i < size; i++) {
    outputData[i] = inputData[i] - (slope * i + intercept);
  }
}

void buzzerBeep(int T)
{ 
  analogWrite(BUZZER, 150);
  delay(T);
  analogWrite(BUZZER, 0);
}
