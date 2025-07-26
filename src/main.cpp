#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

#include <Adafruit_SH110X.h>
#include <thermistor.h>


#define i2c_Address 0x3c //initialize with the I2C addr 0x3C typically for OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO 
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // declaring OLED name
//OLED_SSD1306_Chart display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define pins on board
#define button  4   // rotary button pin
#define dt 5        // roraty dt pin
#define clk 7       // rotory clk pin
#define SSR 10      // solid state relay pin
#define Thermistor_PIN A0 // analog pin for thermistor

thermistor therm1(Thermistor_PIN, 0); // thermistor pin 


int counter = 150;          //counter for rotary to set target temp, start value
int currentState;           // current state for rotory
int LastState;              // last state for rotoary
int butState;               // current button state
int lastbutState = 1;       // last button state
unsigned long last_run = 0; // timer for rotary

double targetTemp;          // target temperature
double currentTemp;         // current temperature

int func = 0;               // state of program
static int timeOut = 180; // time out for saftey - if temp does not reach target value within this time, errors out

// VARIABLES //
unsigned long millis_before, millis_before_2;    //Loop refresh rate
unsigned long millis_now = 0;
unsigned long millisGraph=0;
unsigned long millisTimer=0;
unsigned long lastTempChangeTime = 0;
unsigned long millis_safe = 0;

float refresh_rate = 500;                       //LCD refresh rate.
float pid_refresh_rate  = 50;                   //PID Refresh rate
float seconds = 0;                              //Variable used to store the elapsed time
float temperature = 0;                          //Store the temperature value here
float preheat;                                  //preheat value
float soak;                                     //Soak value
float reflow;                                   //Reflow Value
float temp_setpoint = 0;                        //Used for PID control
float pwm_value = 255;                          //The SSR is OFF with HIGH, so 255 PWM would turn OFF the SSR
float MIN_PID_VALUE = 0;
float MAX_PID_VALUE = 255;                      //Max PID value. 
float coolDown = 40;                            //temperature to cool down to

///////////////////// PID VARIABLES ///////////////////////
/////////////////////////////////////////////////////////
float Kp = 50;              // Gain value for the Proportional term - further you are from the setpoint, the more power is applied
float Ki = 0.00;            // Error integral gain value for the Integral term - added to the output to eliminate steady state error
float Kd = 0;               // Derivative gain value for the Derivative term - used to dampen the output and reduce overshoot
float PID_Output = 0;
float PID_P, PID_I, PID_D;
float PID_ERROR, PREV_ERROR;
/////////////////////////////////////////////////////////


///////////////////////////////// FUNCTIONS ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// PID calculator

float PID_Calc(float temp_setpoint, float temperature) {

  //  if (millis_now - millis_before_2 > pid_refresh_rate) {  //Refresh rate of the PID
  //    millis_before_2 = millis();


  PID_ERROR =  temp_setpoint - temperature;
  PID_P = Kp * PID_ERROR;
  //  PID_I = PID_I + (Ki * PID_ERROR); // Integral term accumulates error over time - was couasing overshoot
  PID_D = Kd * (PID_ERROR - PREV_ERROR);
  PID_Output = PID_P + PID_I + PID_D;

  PREV_ERROR = PID_ERROR;

  constrain(PID_Output, 0, 255);

  pwm_value = floor(constrain(PID_Output, 0, 255));

  return floor(pwm_value);

}


// Rotory knob and button

void rotory() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5) {
    currentState = digitalRead(clk); // Reads the "current" state of the clk pin
    
      // If the previous and the current state of the clk are different, that means a Pulse has occured
    if (currentState != LastState && currentState == 0 ) {
      
      //    If the dt pin state is different to the clk state, that means the encoder is rotating clockwise
     if (digitalRead(dt) != currentState ) {
        if ( counter < 500) {
          counter = counter + 5;
          Serial.println("CW");
        }

      }


      if (digitalRead(dt) == currentState) {
        if (counter > 20) {
          counter = counter - 5;
          Serial.println("CCW");
        }
      }
      Serial.print("Position: ");
      Serial.println(counter);
    }
    LastState = currentState;

  }
}

void Button() {
  butState = digitalRead(button);


  if (butState != lastbutState) {

    if (butState == LOW) {

      func++;
    }
    else {
    }
  }
  lastbutState = butState;
  // return counter;
}

void ButtonRST() {
  butState = digitalRead(button);


  if (butState != lastbutState) {

    if (butState == LOW) {

      analogWrite(SSR, LOW);

      func = 0 ;
    }
    else {
    }
  }
  lastbutState = butState;
  // return counter;
}
// function for plotting temp

void plotTemp(float temp, String tempProfile, int target) {

  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temp);
  display.print("C ");
  display.println(tempProfile);
  display.print("Time: ");
  display.print((millis_now - millisTimer) / 1000); // Time in seconds
  display.print("s ");
  display.print("target:");
  display.print(target);
  display.println(" ");

  // Draw a line chart
  int x = map(millis() - millisGraph, 0, 360000, 0, SCREEN_WIDTH);
  int y = map(temp, 0, 500, SCREEN_HEIGHT, 0);
  display.drawPixel(x, y, SH110X_WHITE);

  // Display the updated screen
  display.display();
  
}


// SAFTY FUNCTION

void saftey(int currentT, float target) {
 
  if ((millis_now - lastTempChangeTime) / 1000 >= timeOut) {
    if (currentT != target) {
      display.setCursor(0, 0);
      display.clearDisplay();
      display.setCursor(50, 25);
      display.setTextColor(SH110X_WHITE);
      display.println("ERROR!!!!!");
      display.display();
      display.print("ERROR!!!!!");
      Serial.println("ERROR!!! FROM FUNCTION");
      analogWrite(SSR, 0);
      while (1);
    }

    else {
      Serial.println("saftey");

      lastTempChangeTime = millis_now;
    }

  }
}

void setup() {

  // pin mode for rotary
  pinMode (clk, INPUT_PULLUP);
  pinMode (dt, INPUT_PULLUP);
  pinMode (button, INPUT_PULLUP);

  LastState = digitalRead(clk);
  lastbutState = digitalRead(button);

  attachInterrupt(digitalPinToInterrupt(clk), rotory, LOW); // interruprt when clk pin triggers low in rotory function

  // pin mode for solid state relay
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, LOW);        //Make sure we start with the SSR OFF (is on with HIGH)


  pinMode(Thermistor_PIN, INPUT); // thermistor anolog pin input

  Serial.begin(9600);

  delay(250); // wait for the OLED to power up
  display.begin(i2c_Address, true); // Address 0x3C default
  //display.setContrast (0); // dim display

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  // millis_before = millis();
  // millis_now = millis();

}

void loop() {

  currentTemp = therm1.analog2temp();

  switch (func) {

    case 0:   // case for setting pre-heat temp

      rotory();
      display.setCursor(0, 0);
      display.clearDisplay();
      display.println("Pre-heat temp:");
      display.setCursor(50, 25);
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
      display.println(counter);
      display.display();
      Button();

      preheat = counter;


      break;

    case 1:  // case for saok temp

      rotory();
      display.setCursor(0, 0);
      display.clearDisplay();
      display.println("soak temp:");
      display.setCursor(50, 25);
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
      display.println(counter);
      display.display();
      Button();

      soak = counter;


      break;

 
    case 2:  // case for reflow temp

      rotory();
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("reflow temp:");
      display.setCursor(50, 25);
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
      display.println(counter);
      display.display();
      Button();

      reflow = counter;
     
      break;

    case 3: // case to clear screen anc set millis variables
      
      display.clearDisplay();

      millis_before = millis();
      lastTempChangeTime = millis();
      millisGraph = millis();
      millisTimer = millis();
      millis_now  = millis();
      func = 4;

      break;

    case 4:  // heating plate to pre-heat temp

      millis_now = millis();

      saftey(currentTemp, preheat);


      pwm_value = PID_Calc(preheat, currentTemp);
      analogWrite(SSR, pwm_value);

      
      Serial.print("error: ");
      Serial.print(PID_ERROR);
      Serial.print("  PMW Value: ");
      Serial.print(pwm_value);
      Serial.print("  PID out: ");
      Serial.print(PID_Output);
      Serial.println("      PREHEAT ");

      Serial.println(millis_now - lastTempChangeTime);

      plotTemp(currentTemp, "PreHeat",preheat);

      display.display();

      ButtonRST();

  

      if (currentTemp >= preheat)  {     

        millis_before = millis_now;
        millisTimer = millis_now;
        lastTempChangeTime = millis_now;
        func = 5;
      }

      break;

    case 5:  // heating plate to soak temp

      millis_now = millis();

      saftey(currentTemp, soak);

      pwm_value = PID_Calc(soak, currentTemp);
      analogWrite(SSR, pwm_value);

      plotTemp(currentTemp, " soak  ",soak);

      Serial.print("error: ");
      Serial.print(PID_ERROR);
      Serial.print("  PMW Value: ");
      Serial.print(pwm_value);
      Serial.print("  PID out: ");
      Serial.println(PID_Output);

      ButtonRST();

      

        if  ((millis_now - millis_before) / 1000 >= 60) {
        millis_before = millis_now;
        millisTimer = millis_now;
        lastTempChangeTime = millis_now;

        func = 6;

        }

      
      break;

    case 6:  // heating plate to reflow temp

      millis_now = millis();

      saftey(currentTemp, reflow);
      pwm_value = PID_Calc(reflow, currentTemp);
      analogWrite(SSR, pwm_value);

      Serial.print("error: ");
      Serial.print(PID_ERROR);
      Serial.print("  PMW Value: ");
      Serial.print(pwm_value);
      Serial.print("  PID out: ");
      Serial.println(PID_Output);

      plotTemp(currentTemp, "reflow ",reflow);

      // Serial.println((millis_now - millis_before) / 1000);
      display.display();

      ButtonRST();

      

        if ((millis_now - millis_before) / 1000 >= 60) {

         millis_before = millis_now;
         millisTimer = millis_now;
         lastTempChangeTime = millis_now;
         func = 7;

        }

      

      break;

    case 7: // cooling plate

      millis_now = millis();

      analogWrite(SSR, LOW);

      Serial.print("error: ");
      Serial.print(PID_ERROR);
      Serial.print("  PMW Value: ");
      Serial.print(pwm_value);
      Serial.print("  PID out: ");
      Serial.println(PID_Output);

      plotTemp(currentTemp, "cool   ", 40);

      ButtonRST();


      
      display.display();

      if ( currentTemp <= coolDown) {

        millisTimer = millis_now;
        func = 8;
        
      }

      break;

    case 8:

      display.clearDisplay();
      display.setCursor(50, 25);
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
      display.println("DONE!");
      display.display();

      ButtonRST();

      // delay(10000);
      // func = 0;

      break;

      

  }

}