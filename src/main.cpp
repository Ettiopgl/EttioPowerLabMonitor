/***************************************************************************
* Example sketch for the INA226_WE library
*
* This sketch shows how to use the INA226 module in continuous mode. 
*  
* Further information can be found on:
* https://wolles-elektronikkiste.de/ina226P (German)
* https://wolles-elektronikkiste.de/en/ina226-current-and-power-sensor (English)
* 
***************************************************************************/
#include <Wire.h>
#include <INA226_WE.h>
#define I2C_ADDRESS 0x40
//#define I2C_ADDRESS_N 0x41
#include <Arduino.h>
//#include <SoftwareSerial.h>

//+++++++++++++++++++++++++++ For tft9341
#include "Adafruit_GFX.h"
#include <Fonts/FreeSerif9pt7b.h>
#include "Adafruit_ILI9341.h"
#include "Adafruit_I2CDevice.h"
//++++++++++++++++++++++++++++ pin For tft9341
#define TFT_DC 9
#define TFT_CS 10
#define TFT_CLK 13
#define TFT_MISO 12
#define TFT_MOSI 11
#define TFT_RST 8
// LED sul 3.3v
// GND
// VCC sul 3.3
//+++++++++++++++++++++++++++++ color to tft9341
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0X07FF
#define MAGENTA 0XF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
//+++++++++++++++++++++++++++++ Ina variabili
float busVoltage_V = 0;
float current_mAPos = 0;
//float current_mANeg = 0;
float power_mW = 0;
float setCurrentLimit = 0;
int interruptAlarmPin = 2;
// int interruptEncoderPin =3;
volatile bool event = false;
const int alarmPin = 4; // D4 disegno On/ff
volatile bool powerlimit = false;
//+++++++++++++++++++++++++++++ Variabili encoder
boolean setCurrent = HIGH;


//+++++++++++++++++++++++++++++ end variabili encoder
INA226_WE ina226P(I2C_ADDRESS);
//INA226_WE ina226N(I2C_ADDRESS_N);
//+++++++++++++++++++++++++++++ Variable for swap delay to millis
unsigned long previousMillis = 0;
unsigned long interval = 2000;
//+++++++++++++++++++++++++++++ Variable fo debounce botton from Attiny 85
//#define attinyID 0x44  /////////////modificare indirizzo su attiny
#define attinyID 0x08  /////////////modificare indirizzo su attiny
int buttonStateStartStop;
int lastButtonStateStartStop = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int startState = LOW;  //On - Off corrente On= HIGH


//+++++++++++++++++++++++++++++ Funtions
void displayResults(); // dichiarazione di funzione displayResults (visualizzazione)
void alert();
void displayTemplateP();
//void displayTemplateN();
void displayStart();
// void buttonDebounce();
void overLoadCurrent(float maPos, float maNeg, float mAlimit);
// void doEncoder();  //new encoder
//+++++++++++++++++++++++++++++ End Funtions

void setup()
{
  
  //powerlimit = true;  // protezione
  Serial.begin(9600);
  Wire.begin();
 
  
  
  // pinMode(alarmPin, OUTPUT);  // accensione dei Mos
  // pinMode(buttonPin, INPUT);  // pulsante   non più usato 
  //digitalWrite(buttonPin, LOW);
  
  // Ramo Positivo
  ina226P.init();
  ina226P.setAverage(AVERAGE_16);
  ina226P.setConversionTime(CONV_TIME_140);
  ina226P.enableAlertLatch();
  ina226P.setAlertType(CURRENT_OVER, setCurrentLimit);
  //ina226P.setCurrentRange(MA_800);
  ina226P.setResistorRange(0.01, 6);
  //Ramo Negativo
  //ina226N.init();
  //ina226N.setAverage(AVERAGE_16);
  //ina226N.setConversionTime(CONV_TIME_140);
  //ina226N.enableAlertLatch();
  //ina226N.setAlertType(CURRENT_OVER, setCurrentLimit);
  //ina226N.setResistorRange(0.01, 6);
  attachInterrupt(digitalPinToInterrupt(interruptAlarmPin), alert, FALLING);
  //attachInterrupt(1, doEncoder, FALLING);  //New Enc

  tft.begin();
  tft.fillScreen(0);
  tft.setRotation(1); // ORIZZONTALE      0=VERTICALE Rotazione del display
  tft.setTextSize(3); //DIMENSIONI DEL FONT
  //displayStart(); //  gestione puntini
  displayTemplateP();
  //displayTemplateN();
}

void loop()
{
  //Serial.print("Bus Voltage [V+]: "); Serial.println(busVoltage_V);
  

  unsigned long currentMillis = millis();

  //////////////////////// encoder  from Attiny85
Wire.requestFrom(attinyID,2); //REQUEST 1 BUTE

  byte encoderPos = Wire.read();
  byte readingButton = Wire.read();
  if(readingButton != lastButtonStateStartStop)
  {
    lastDebounceTime = millis();
  }
  if((millis() - lastDebounceTime) >  debounceDelay)
  {
    if (readingButton != buttonStateStartStop)
    {
      buttonStateStartStop = readingButton;
      if(buttonStateStartStop == LOW)
      {
        startState = ! startState;
      }
    }
  }
lastButtonStateStartStop = readingButton;
  
  Serial.print("encoderPos="); Serial.println(encoderPos);
  Serial.print("startState="); Serial.println(startState);
      
    setCurrentLimit = encoderPos * 10;  // mA
      
      ina226P.setAlertType(CURRENT_OVER, setCurrentLimit);
    //  ina226N.setAlertType(CURRENT_OVER, setCurrentLimit);
      
     // tft.setCursor(220,5);
     // tft.print(setCurrentLimit); // set current Limit
      displayResults();
   
////////////////// end encoder from Attiny85
   
  //  
//Serial.print("event = "); Serial.println(event );
  if (event)
  {
    //Serial.print("event  [V+]: ");
ina226P.readAndClearFlags();
    displayResults();
 // attachInterrupt(digitalPinToInterrupt(interruptPin), alert, FALLING); 
   attachInterrupt(digitalPinToInterrupt(interruptAlarmPin), alert, FALLING);
    ina226P.readAndClearFlags(); 

    current_mAPos = ina226P.getCurrent_mA(); // qui legge la corrente che supera limit alert
    
    //current_mANeg = ina226N.getCurrent_mA();




    //ina226P.readAndClearFlags(); // reads interrupt and overflow flags and deletes them
    //ina226P.powerDown(); // salva il contenuto del registro di conf e disabilita l'ina
    //ina226P.readAndClearFlags();
    //ina226N.powerDown();
    //ina226N.readAndClearFlags();
  // attachInterrupt(digitalPinToInterrupt(interruptAlarmPin), alert, FALLING);
    //powerlimit = true;
    event = false;
    //delay (1000);
  }
//delay (1000);


  if (currentMillis - previousMillis > interval)
  {
    if (powerlimit)
    {
Serial.print("event= "); Serial.println(event);

      Serial.println("Limit Alert- Power Down");
      Serial.print("OverLoad_Current[mA]_Pos: ");
      Serial.println(current_mAPos);
      //Serial.print("Overload_current[mA]_Neg:");
     //Serial.println(current_mANeg);
     // Serial.print("encoderPos = ");
     // Serial.print(encoderPos);
      //Serial.print("   Start =");
      //Serial.println(startState);
      Serial.print("CurrentLimit_Set=");
      Serial.println(setCurrentLimit);
      // overLoadCurrent(current_mAPos,current_mANeg,setCurrentLimit);
    }
    else
    {
      displayResults();
    }
    previousMillis = currentMillis;
  } //end millis
  

  //// debounce
 //  buttonDebounce();

  
} // end loop

void displayResults()
{
  busVoltage_V = 0.0;
  current_mAPos = 0.0;
  //current_mANeg = 0.0;
  power_mW = 0.0;

  // Ramo Positivo
  busVoltage_V = ina226P.getBusVoltage_V();
  current_mAPos = ina226P.getCurrent_mA();
  power_mW = ina226P.getBusPower();
  Serial.print("Bus Voltage [V+]: "); Serial.println(busVoltage_V);
  
  Serial.print("Current[mA] + : "); Serial.println(current_mAPos);
  Serial.print("Bus Power [mW] +: "); Serial.println(power_mW);



  if (ina226P.limitAlert)
  {
    Serial.println("Limit Alert !!!!");
  }
  if (ina226P.convAlert)
  {
    Serial.println("Conversion Alert!!!!");
  }
  
 
  Serial.print("Bus Voltage [V]: ");
  Serial.println(busVoltage_V);
  Serial.print("Load Voltage [V+]: ");
  Serial.println(busVoltage_V);
  Serial.print("Current[mA+]: ");
  Serial.println(current_mAPos);
  Serial.println("Set-limit-current in mA + = " + String(setCurrentLimit));
  Serial.print("Bus Power [mW+]: ");
  Serial.println(power_mW);

  tft.setCursor(60, 5); //tft.setCursor(70,5);
  tft.setTextColor(WHITE, 0x0000);
  //char xbusVoltage_V [4];
  char x [4];
  dtostrf(busVoltage_V,2,2,x); // visualizzazione con 2interi e 2 cifre decimali
  tft.print(x);
  tft.setCursor(60, 45); //tft.setCursor(70,5);
  float current_APos = current_mAPos / 1000;
  dtostrf(current_mAPos,1,2,x); // visualizzazione con 1interi e 2 cifre decimali
  tft.print(current_APos);

  tft.setCursor(60, 90); //tft.setCursor(70,5);
  float power_W = power_mW / 1000;
dtostrf(power_W,3,1,x);       // visualizzazione con 2interi e 2 cifre decimali
  //tft.print(power_W);
  tft.print(x);
tft.setTextColor(GREEN, 0x0000);
tft.setCursor(225, 5); // y=45
float setCurrentLimitInA =setCurrentLimit/100;
dtostrf(setCurrentLimitInA,1,2,x);
  //tft.print(setCurrentLimit); // set current Limit
tft.print(x); // set current Limit

  if (!ina226P.overflow)
  {
    displayTemplateP();
    //displayTemplateN();
    Serial.println("++++++++++++++Values OK - no overflow");
  }
  else
  {
    //overLoadCurrent();
    Serial.println("++++++++++++++++++++Overflow! Choose higher current range");
  }
  Serial.println();


/*
  // ramo Negativo
  //shuntVoltage_mV = ina226N.getShuntVoltage_mV();
  busVoltage_V = ina226N.getBusVoltage_V();
  current_mANeg = ina226N.getCurrent_mA();
  power_mW = ina226N.getBusPower();
  //loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

  if (ina226N.limitAlert)
  {
    Serial.println("Limit Alert !!!!");
  }
  if (ina226P.convAlert)
  {
    Serial.println("Conversion Alert!!!!");
  }
  
  Serial.print("Shunt Voltage [mV]: ");
  Serial.println(shuntVoltage_mV);
  Serial.print("Bus Voltage [V]: ");
  Serial.println(busVoltage_V);
  
  Serial.print("Load Voltage [V-]: ");
  Serial.println(busVoltage_V);
  Serial.print("Current[mA-]: ");
  Serial.println(current_mANeg);
  Serial.println("Set-limit-current in mA - = " + String(setCurrentLimit));
  Serial.print("Bus Power [mW-]: ");
  Serial.println(power_mW);
  tft.setCursor(60, 125); //tft.setCursor(70,5);
  tft.setTextColor(WHITE, 0x0000);
  tft.print(busVoltage_V);
  tft.setCursor(60, 167); //tft.setCursor(70,5);
  tft.print(current_mANeg);
  tft.setCursor(60, 210); //tft.setCursor(70,5);
  tft.print(power_mW);

  tft.setCursor(200, 167);
  tft.print(setCurrentLimit); // set current Limit

  if (!ina226N.overflow)
  {

    Serial.println("----------- Values OK - no overflow");
  }
  else
  {
    // overLoadCurrent();
    Serial.println("----------- Overflow! Choose higher current range");
  }
  Serial.println();

*/
  // end displayResult
}


void alert()
{
  //delay(100);
  //digitalWrite(alarmPin, LOW); // con il relè deve andare LOW
  event = true;
  detachInterrupt(2);
  // overLoadCurrent();
}

void displayTemplateP()
{
  // ramo positivo
  tft.drawRect(0, 0, 320, 119, 0xF800); //x0,y0,x1,y1,colore
  tft.setTextColor(RED, 0x0000);        // COLORE DEL TESTO 0x0001F E SFONDO DEL TESTO 0x0000
  tft.setCursor(3, 5);
  tft.print("V+=");
  tft.setCursor(3, 90);
  tft.print("W =");
  tft.setCursor(3, 45);
  tft.print("I+");
  tft.setCursor(160, 5);
  tft.setTextColor(GREEN,0x0000);
  tft.print("Il=");
  //tft.setCursor(150, 45);
  //tft.print("Il="); // set current Limit
}

/*
void displayTemplateN()
{
  // ramo negativo

  tft.drawRect(0, 121, 320, 119, 0x07E0); //x0,y0,x1,y1,colore
  tft.setTextColor(BLUE, 0x0000);         // COLORE DEL TESTO 0x0001F E SFONDO DEL TESTO 0x0000
  tft.setCursor(4, 125);
  tft.print("V-=");
  tft.setCursor(4, 167); //tft.setCursor(170,125);
  tft.print("I =");
  tft.setCursor(4, 210);
  tft.print("W =");
  //tft.setCursor(135, 167);
  //tft.print("Il="); // set current Limit
  // end tft
}
*/

/////////////////// gestione puntini su start////////////////
void displayStart()
{
  const int xMax = 320;
  const int yMax = 240;
  int xrandom;
  int yrandom;

  for (int i = 0; i <= 500; i++)
  {
    delay(10);
    xrandom = random(xMax);
    yrandom = random(yMax);
    tft.drawPixel(xrandom, yrandom, WHITE);
  }
  delay(500);
  tft.fillScreen(0); // cancella lo schermo
  tft.setFont(&FreeSerif9pt7b);
  // tft.setFont();  //Carattere di default
  tft.setTextColor(WHITE, 0x0000);
  tft.setCursor(5, 60);
  tft.println("Ettio");
  tft.println("Power");
  tft.print("Lab");
  delay(2000);
  tft.fillScreen(0);
  tft.setFont(); //Carattere di default
}

void overLoadCurrent(float mAPos, float mANeg, float mALimit)
{
  tft.fillScreen(0); // cancella lo schermo
  tft.setTextColor(RED, 0x0000);
  tft.setCursor(3, 5);
  tft.print("+A=");
  tft.print(mAPos);
  tft.setCursor(3, 45);
  tft.print("limit=");
  tft.print(mALimit);
  tft.setCursor(260, 5);
  tft.print("OFF"); // set current Limit

  tft.setTextColor(BLUE, 0x0000); // COLORE DEL TESTO 0x0001F E SFONDO DEL TESTO 0x0000
  tft.setCursor(4, 125);
  tft.print("-A=");
  tft.print(mANeg);
  tft.setCursor(260, 125);
  tft.print("OFF"); // set current Limit
}





