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

/**********************Ettio*******************************
 *
 * Line 62 float setPowerLimit = 10000; //mW  dichiarare un numero divero da 0
 *         altrimenti va in allarme AlertLimit
 * Line 190 ina226P.setAlertType(POWER_OVER, setPowerLimit); registrare il nuovo power limit
 *         alla pressione del pilsante ON
 * ********************Ettio***************************/
#include <Wire.h>
#include <INA226_WE.h>
#include <Arduino.h>
//#include <SoftwareSerial.h>

//+++++++++++++++++++++++++++++ Variabili definizioni e variabili I2C
#define I2C_ADDRESS 0x40
#define I2C_ADDRESS_N 0x41
#define attinyID 0x08             // ind. attiny
INA226_WE ina226P(I2C_ADDRESS);   // ind. ina +
INA226_WE ina226N(I2C_ADDRESS_N); // ind. ina -
//+++++++++++++++++++++++++++++ end variabili encoder

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
float current_mANeg = 0;
// float power_mW = 0;
float setCurrentLimit = 0.0;
float setPowerLimit = 0; // mW

int interruptAlarmPin = 2;
volatile bool event = false;
volatile bool powerlimit = false;

//*****************************Variabili Pulsante On/off
const int inButtonOnOff = 3;  // pin pulsante on/Off
const int outButtonOnOff = 4; // pin out D4 disegno On/ff
int lastButtonStateStartStop = LOW;
int buttonStateStartStop;
int onOffState = LOW; // On - Off corrente On= HIGH
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

//***********************  Variabili tempo

unsigned long previousMillis = 0;
unsigned long timeToRefreshDisplay = 1500;

//+++++++++++++++++++++++++++++ Dichiarazioni Funtions
void displayResults(); // dichiarazione di funzione displayResults (visualizzazione)
void alert();
void displayTemplateP(); // template display +
void displayTemplateN(); // template display -
void displayStart();
// void buttonDebounce();
// void overLoadCurrent(float maPos, float maNeg, float mAlimit);
// void doEncoder();  //new encoder
//+++++++++++++++++++++++++++++ End Funtions

void setup()
{
  // powerlimit = true;  // protezione
  Serial.begin(9600);
  Wire.begin();
  //////////////////////////////// config. pin allarme  "interrupt"
  // pinMode(interruptAlarmPin,INPUT);
  // digitalWrite(interruptAlarmPin,HIGH);

  ////////////////////////////// pulsante
  // pinMode(outButtonOnOff, OUTPUT); // accensione dei Mos
  // pinMode(inButtonOnOff, INPUT);   // collegamento pulsante
  // digitalWrite(outButtonOnOff, onOffState);

  //////////////////////////// Ramo Positivo
  ina226P.init();
  // ina226P.setAverage(AVERAGE_16);
  // ina226P.setConversionTime(CONV_TIME_140);
  // ina226P.setCorrectionFactor(1.2);//      Fattore di correzione = corrente erogata da apparecchiatura calibrata / corrente erogata da INA226
  ina226P.setResistorRange(0.01, 5); // choose resistor 10 mOhm and gain range up to 6 A
  ina226P.enableAlertLatch();
  setPowerLimit = 10000; // mW valore di defoult per accendere l'alimentatore
  ina226P.setAlertType(POWER_OVER, setPowerLimit);

  // ina226P.enableConvReadyAlert(); // In this example we also enable the conversion ready alert interrupt

  ////////////////////////////// Ramo Negativo
  //  ina226N.init();
  //  ina226N.setAverage(AVERAGE_16);                      // media di valori
  //  ina226N.setConversionTime(CONV_TIME_140);            //tempo di conversione
  // ina226N.enableAlertLatch();                          //attiva il pin di allarme
  // ina226N.setAlertType(CURRENT_OVER, setCurrentLimit); // settaggio quale allarme
  // ina226N.setResistorRange(0.01, 6);
  // ina226N.enableConvReadyAlert(); // In this example we also enable the conversion ready alert interrupt

  attachInterrupt(digitalPinToInterrupt(interruptAlarmPin), alert, FALLING); // abilitando qui lo schermo diventa bianco
  /////////////////////////////// display
  tft.begin();
  tft.fillScreen(0);
  tft.setRotation(1); // ORIZZONTALE      0=VERTICALE Rotazione del display
  tft.setTextSize(3); // DIMENSIONI DEL FONT
  displayStart();     //  gestione puntini
  displayTemplateP();
  displayTemplateN();
  // attachInterrupt(digitalPinToInterrupt(interruptAlarmPin), alert, FALLING); // abilitando qui lo schermo diventa bianco
}

void loop()
{

  //////////////////////////////////////////////////////////////////////////////
  unsigned long currentMillis = millis(); // variabile tempo per la gesione reflesh display

  ///////////////////////// pulsante
  int readingButton = digitalRead(inButtonOnOff);
  if (readingButton != lastButtonStateStartStop) // se ?? diverso da
  {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay)

  {
    if (readingButton != buttonStateStartStop) // se ?? diverso da
    {
      buttonStateStartStop = readingButton;
      if (buttonStateStartStop == HIGH)
      {
        onOffState = !onOffState;
      }
    }
  }
  lastButtonStateStartStop = readingButton;
  // digitalWrite(outButtonOnOff,onOffState);
  ///////////

  if (onOffState == HIGH)
  {
    Serial.println(" pulsante ON");
    tft.setTextColor(RED, 0x0000);
    tft.setCursor(220, 45);
    tft.print("     ");
    tft.setCursor(220, 167);
    tft.print("     ");
    tft.setCursor(220, 90);
    tft.setTextColor(RED, 0x0000);
    tft.print("ON   ");
    digitalWrite(outButtonOnOff, HIGH);

    // setCurrentLimit = ina226P.getCurrent_mA();
    setCurrentLimit = 100.0;
    busVoltage_V = ina226P.getBusVoltage_V();
    setPowerLimit = (busVoltage_V * setCurrentLimit);
    ina226P.setAlertType(POWER_OVER, setPowerLimit);

    if (ina226P.limitAlert)
    {
      ina226P.limitAlert = false;
      tft.setCursor(220, 45); // 220,90
      tft.print("     ");
    }
  }
  else
  {
    Serial.println("pulsante Off");
    tft.setCursor(220, 90);
    tft.setTextColor(GREEN, 0x0000);
    tft.print("OFF  ");
    digitalWrite(outButtonOnOff, LOW);
  }
  /*/////////

  //////////////////////// encoder  from Attiny85

  Wire.requestFrom(attinyID, 2); //REQUEST 2 ByTE
  unsigned int encoderPos = Wire.read();
  encoderPos = encoderPos/4;

  //unsigned int readingButton = Wire.read();   // dall'attiny
  Serial.print("encoderPos=");
  Serial.println(encoderPos);

  if (encoderPos <= 100)
  {
    setCurrentLimit = encoderPos * 10; // mA
  }
  else if (encoderPos >= 115)
  {
    setCurrentLimit = encoderPos * 20; // mA
  }
  else
  {
    setCurrentLimit = encoderPos * 12;
  }
  ina226P.setAlertType(CURRENT_OVER, setCurrentLimit);
  ina226N.setAlertType(CURRENT_OVER, setCurrentLimit);


*/

  if (currentMillis - previousMillis > timeToRefreshDisplay)
  {
    displayResults(); // refresh display

    previousMillis = currentMillis;
  }
  ////////////////// end encoder from Attiny85
  ///////////////////////////////////////////////////////////////////////////////
  //
  if (event)
  {

    ina226P.readAndClearFlags();
    // ina226N.readAndClearFlags();
    displayResults();

    attachInterrupt(digitalPinToInterrupt(interruptAlarmPin), alert, FALLING);

    event = false;
    ina226P.readAndClearFlags();

    // current_mAPos = ina226P.getCurrent_mA(); // qui legge la corrente che supera limit alert
    // current_mANeg  = ina226N.getCurrent_mA();
    // ina226P.readAndClearFlags(); // reads interrupt and overflow flags and deletes them

    /*
    ina226P.powerDown(); // salva il contenuto del registro di conf e disabilita l'ina
    ina226P.readAndClearFlags();
    ina226N.powerDown();
    ina226N.readAndClearFlags();
*/
    // attachInterrupt(digitalPinToInterrupt(interruptAlarmPin), alert, FALLING);
    // powerlimit = true;

    // overLoadCurrent(current_mAPos,current_mANeg, setCurrentLimit);
  }
  // delay(50);

  // event = false;

} //++++++++++++++++++++++++++++ end loop+++++++++++++++++++++++++++//

void displayResults()
{
  float power_mW = 0.0;
  busVoltage_V = 0.0;
  current_mAPos = 0.0;
  current_mANeg = 0.0;

  // Ramo Positivo
  busVoltage_V = ina226P.getBusVoltage_V();
  current_mAPos = ina226P.getCurrent_mA();
  power_mW = ina226P.getBusPower();
  tft.setCursor(60, 5); // tft.setCursor(70,5);
  tft.setTextColor(WHITE, 0x0000);
  char x[4];
  dtostrf(busVoltage_V, 2, 2, x); // visualizzazione con 2interi e 2 cifre decimali
  tft.print(x);
  tft.setCursor(60, 45); // tft.setCursor(70,5);
  // float current_APos = current_mAPos / 1000;
  dtostrf(current_mAPos, 1, 2, x); // visualizzazione con 1interi e 2 cifre decimali
  // tft.print(current_APos);
  tft.print(x);
  tft.setCursor(60, 90); // tft.setCursor(70,5);
  float power_W = power_mW / 1000;
  dtostrf(power_W, 3, 1, x); // visualizzazione con 2interi e 2 cifre decimali
  // tft.print(power_W);
  tft.print(x);
  tft.setTextColor(GREEN, 0x0000);
  tft.setCursor(225, 5); // y=45
  // float setCurrentLimitInA = setCurrentLimit / 1000;
  float setCurrentLimitInA = setCurrentLimit;
  dtostrf(setCurrentLimitInA, 1, 1, x); // visualizzazione con 1 interi e 1 cifre decimali
  // tft.print(setCurrentLimit); // set current Limit
  tft.print(x); // set current Limit
  if (ina226P.limitAlert)
  {
    Serial.println("Limit Alert !!!!");
    Serial.print("Vout+=");
    Serial.print(busVoltage_V);
    Serial.print("/");
    Serial.println(current_mAPos);

    tft.setCursor(220, 45); // 220,90
    tft.setTextColor(RED, 0x0000);
    tft.print("Alert");

    delay(500);
  }

  Serial.print("Vout+=");
  Serial.print(busVoltage_V);
  Serial.print("/");
  Serial.println(current_mAPos);

  /*
                // ramo Negativo
  busVoltage_V = ina226N.getBusVoltage_V();
  current_mANeg = ina226N.getCurrent_mA();
  power_mW = ina226N.getBusPower();
  if (ina226P.limitAlert)
  {
    Serial.println("Limit Alert !!!!");
  }
  if (ina226P.convAlert)
  {
    Serial.println("Conversion Alert!!!!");
  }

  tft.setCursor(60, 125); //tft.setCursor(70,5);
  tft.setTextColor(WHITE, 0x0000);
//char x [4];
  dtostrf(busVoltage_V, 2, 2, x); // visualizzazione con 2interi e 2 cifre decimali
  tft.print(x);
  tft.setCursor(60, 167);          //tft.setCursor(70,5);
                                   //float current_ANeg = current_mANeg / 1000;
  dtostrf(current_mANeg, 1, 2, x); // visualizzazione con 1interi e 2 cifre decimali
  Serial.print("Current[mA-]: ");
  Serial.println(current_mANeg);
  //tft.print(x);
tft.print("123");
  tft.setCursor(60, 210); //tft.setCursor(70,5);
  power_W = power_mW / 1000;
  dtostrf(power_W, 3, 1, x); // visualizzazione con 2interi e 2 cifre decimali
  tft.print(x);

*/

  /////////////// spostato da funzione controllo tasto ON/OFF
  /*
tft.setTextColor(RED, 0x0000);
        tft.setCursor(220,45);
    tft.print("     ");
    tft.setCursor(220,167);
    tft.print("     ");
*/
  /////////////////////
  if (!ina226P.overflow)
  {
    displayTemplateP();
    // displayTemplateN();
    Serial.println("++++++++++++++Values OK - no overflow");
    delay(100);
  }
  else
  {
    // overLoadCurrent();
    Serial.println("++++++++++++++++++++Overflow! Choose higher current range");
  }
  /*
  if (!ina226N.overflow)
  {
    //Serial.println("----------- Values OK - no overflow");
  }
  else
  {
    // overLoadCurrent();
    Serial.println("----------- Overflow! Choose higher current range");
  }
  // end displayResult
*/
}

void alert()
{
  event = true;
  detachInterrupt(2);

  /*
  current_mAPos = ina226P.getCurrent_mA(); // qui legge la corrente che supera limit alert
  char x[4];
    dtostrf(busVoltage_V, 2, 2, x); // visualizzazione con 2interi e 2 cifre decimali
    tft.print(x);
    tft.setCursor(60, 45); // tft.setCursor(70,5);
    // float current_APos = current_mAPos / 1000;
    dtostrf(current_mAPos, 1, 2, x); // visualizzazione con 1interi e 2 cifre decimali
    // tft.print(current_APos);
    tft.print(x);
  */

  onOffState = LOW; // pulsante on/off
                    // Serial.println("allarme");

  // ina226P.readAndClearFlags();
  // current_mAPos = ina226P.getCurrent_mA(); // qui legge la corrente che supera limit alert
  // ina226N.readAndClearFlags();
  // current_mANeg = ina226N.getCurrent_mA();

  // delay(100);
  /*
    tft.fillScreen(0); // cancella lo schermo
    tft.setCursor(150, 90);
    tft.print(current_mAPos);
    tft.setCursor(150, 140);
    tft.print(current_mANeg);
    tft.print("OFF");

    char y[4];
    tft.setCursor(220, 90);
    tft.setTextColor(RED, 0x0000);
    tft.print("Alarm");
    tft.setCursor(220, 45);
    dtostrf(current_mAPos, 1, 2, y);
    tft.print(y);
    dtostrf(current_mANeg, 1, 2, y);
    tft.setCursor(220, 167);
    tft.print(y);

    digitalWrite(outButtonOnOff, LOW); // con il rel?? deve andare LOW

    /*
    while(value == LOW)
    {
      Serial.println( "allarme");
      value = digitalRead(outButtonOnOff);
      if(value == HIGH)
      {
        break;
      }
    }
    */

  // overLoadCurrent();
}

void displayTemplateP()
{
  // ramo positivo
  tft.drawRect(0, 0, 320, 119, 0xF800); // x0,y0,x1,y1,colore
  tft.setTextColor(RED, 0x0000);        // COLORE DEL TESTO 0x0001F E SFONDO DEL TESTO 0x0000
  tft.setCursor(3, 5);
  tft.print("V+=");
  tft.setCursor(3, 45);
  tft.print("I+");

  tft.setCursor(3, 90);
  tft.print("W =");

  tft.setCursor(160, 5);
  tft.setTextColor(GREEN, 0x0000);
  tft.print("Il=");
}

void displayTemplateN()
{
  // ramo negativo
  tft.drawRect(0, 121, 320, 119, 0x07E0); // x0,y0,x1,y1,colore
  tft.setTextColor(BLUE, 0x0000);         // COLORE DEL TESTO 0x0001F E SFONDO DEL TESTO 0x0000
  tft.setCursor(4, 125);
  tft.print("V-=");
  tft.setCursor(4, 167); // tft.setCursor(170,125);
  tft.print("I =");
  tft.setCursor(4, 210);
  tft.print("W =");
}

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
  ///////////////////////////////////////////////// delay(500);
  tft.fillScreen(0); // cancella lo schermo
  tft.setFont(&FreeSerif9pt7b);
  // tft.setFont();  //Carattere di default
  tft.setTextColor(WHITE, 0x0000);
  tft.setCursor(5, 60);
  tft.println("Ettio");
  tft.println("Power");
  tft.print("Lab");
  ///////////////////////////////////////////////delay(2000);
  tft.fillScreen(0);
  tft.setFont(); // Carattere di default
} ///////////End puntini ///////////////

/*
void overLoadCurrent(float mAPos, float mANeg, float mALimit)
{

  tft.setCursor(60, 90);
  tft.print(mAPos);
tft.setCursor(167, 90);
  tft.print(mANeg);

  delay(5000);

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
*/