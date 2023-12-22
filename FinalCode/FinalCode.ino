#include <Arduino.h>
#include <ezButton.h>
#include <driver/adc.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //OLED
#include <LiquidCrystal_I2C.h> //LCD
#include "Adafruit_SHT31.h" //Humidity sensor
#include "FS.h" //Writing to SD
#include "SD.h" //Writing to SD
#include "SPI.h" //SPI
#include "OLEDbitmaps.h" //file that holds bitmaps
#include <WiFi.h>
#include "time.h"

//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Humidity Sensor
Adafruit_SHT31 hmSen = Adafruit_SHT31();

//I2C object
LiquidCrystal_I2C I2C_LCD1(0x27, 16, 2);

//SPI pins
#define SCK  18
#define MISO  19
#define MOSI  23
#define CS  5

//SPI object
SPIClass spi = SPIClass(VSPI);

//https://github.com/timonbraun02/digital_filter_arduino
//Above is library used for analog filtering
#include "filter_lib.h"
lowpass_filter tempFilter(5); // cutoff frequency in Hz
lowpass_filter lightFilter(5); // default 1 Hz is used as cutoff frequency
lowpass_filter hallFilter(5);
lowpass_filter smokeFilter(8);
lowpass_filter humidFilter(5);

#define configUSE_TIMERS 1

#define ALARM_PERIOD pdMS_TO_TICKS(1000)
//Time for time out timer, 120,000 milliseconds, or 120 seconds, or 2 minutes
#define TIMEOUT_PERIOD pdMS_TO_TICKS(120000)
#define POLL_PERIOD pdMS_TO_TICKS(100)

//Pin Assignments
//White Led 1 associated with Switch
#define WHITE_LED1 32 //32
#define IRin 25 //Inside sensor A1 yellow
#define IRout 34 //outside sesnor A2

Servo motor;
#define SERV 17

//switch object
ezButton switchy(14);

//White LED 2 associated with button
#define WHITE_LED2 15 //15

//button object
ezButton button(26);

#define I2C_Freq 100000
#define SDA0pin 21
#define SCL0pin 22

//RGB LEDs
#define GREEN_LED 27 //27 - shortest pin 4th pin is green
#define RED_LED 12 //12 - 3rd pin is red
#define BLUE_LED 13 //13 - 1st pin is blue
//longest pin is ground

//Buzzer/speaker
#define BUZZER 33 //33

#define FAN 16

//Hall Effect Sensor
#define HAL 39 //A3

SemaphoreHandle_t xLightSenMutex;
#define LIGHTSENPIN 4 //A5

SemaphoreHandle_t xTempSenMutex;
#define SMOKESENPIN 36 //A4

//Enum for defining the different possible states for the alarmState variable
enum State
{
  INACTIVE,
  ACTIVE,
  INTRUDER
};

//FreeRTOS task handles
TaskHandle_t IndLED1;
TaskHandle_t OutLED2;
TaskHandle_t AlmStat;
TaskHandle_t DoorStat;
TaskHandle_t DALStat;
TaskHandle_t BtnStat;
TaskHandle_t IrIns;
TaskHandle_t IrOuts;

//Queues for status changes
QueueHandle_t xIndoorLightChange;
QueueHandle_t xOutdoorLightChange;
QueueHandle_t xInsideDoorChange;
QueueHandle_t xGarageDoorChange;
QueueHandle_t xAlarmStatusChange;

//Semaphore handle for the alarmState binary semaphore
SemaphoreHandle_t xAlarmBiSem;
//default for alarmstate
enum State alarmState = INACTIVE;

//Binary semaphore for protecting the (short) button press
SemaphoreHandle_t xButtonBiSem;
bool BTN_State = false;

//Binary semaphore for protecting the switch state
SemaphoreHandle_t xSwitchBiSem;
int SWTCH_State = 0;

//Binary semaphore for the state of the Hall sensor
SemaphoreHandle_t xHallBiSem;
int HALL_State = 0;

//Button and switch debounce time
#define DEBOUNCE_TIME 50/portTICK_PERIOD_MS

//defintion of the long press time for the setButtonState task
#define LONG_PRESS_TIME 1000/portTICK_PERIOD_MS

//Mutex for the longPressDetected variable used
//currently used in setAlarmState
SemaphoreHandle_t xLongPressMutex;
bool longPressDetected = false;

//temp and light sen global variables
float tempy= 1;
int lightSen = 0;

//Queues for receing from interrupts
QueueHandle_t xTaskOut;
QueueHandle_t xTaskIn;

//I2C Mutex
SemaphoreHandle_t xI2CMutex;

//Alarm related semaphores
SemaphoreHandle_t xRGBLEDMutex;
SemaphoreHandle_t xBuzzerMutex;

//Servo car presence variables and mutexes
SemaphoreHandle_t xCPOMutex;
bool carPresenceOutside = false;
SemaphoreHandle_t xCPIMutex;
bool carPresenceInside = false;

//Alarm timer handles
QueueHandle_t xAlarmTimerQueue;
TimerHandle_t xAlarmTimer;

//Time out timer handles
QueueHandle_t xAlarmTimeOutTimerQueue;
TimerHandle_t xAlarmTimeOutTimer;

//Sensor polling handle
TimerHandle_t xSensorPollingTimer;
QueueHandle_t xTempPollQueue;
QueueHandle_t xLightPollQueue;

//Semaphore for smoke variable
SemaphoreHandle_t xSmokeMutex;


//Alarm timer callback that goes off every 1 second after starting
//Sends a message to the alarm timer queue which tells the alarm task to flip the buzzer
void alarmTimerCallback(TimerHandle_t alarmTimer)
{
  bool alarmLocal = true;
  //No block time for callback functions, gotta keep them short
  xQueueSend(xAlarmTimerQueue, &alarmLocal, 0);
}


//Time out timer callback function, is called 2 minutes after starting
//Sends a message to the time out queue to tell it to disable the buzzer in the alarm task
void alarmTimeOutCallback(TimerHandle_t alarmTimeOutTimer)
{
  bool timeOutLocal = true;
  xQueueSend(xAlarmTimeOutTimerQueue, &timeOutLocal, 0);
}


//Sensor polling timer callback function, sends messages to the sensor queues to tell them to poll the sensors
void sensorPollCallback(TimerHandle_t sensorPollTimer)
{
  bool sensorPollLocal = true;
  xQueueSend(xTempPollQueue, &sensorPollLocal, portMAX_DELAY);
  xQueueSend(xLightPollQueue, &sensorPollLocal, portMAX_DELAY);
}

//Function to make updating variables with mutexes easier
template <typename T>
void updateVariable(T& variable, const T& value, SemaphoreHandle_t mutex)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    variable = value;
    xSemaphoreGive(mutex);
}


//Checks for a long press and writes to the longpress variable
void setButtonState(void * parameters)
{
  //local variables for pressed and released time
  unsigned long pressedTime = 0;
  unsigned long releasedTime = 0;
  while(1)
  {
    //begins the button loop
    button.loop();
     
    //if the button is pressed, we record the time it is pressed in milliseconds
    if(button.isPressed())
    {
      pressedTime = millis()/portTICK_PERIOD_MS;
    }


    //if button is released, we record that time in milliseconds
    if (button.isReleased())
    {
      releasedTime = millis()/portTICK_PERIOD_MS;

      //at the time of release, we calculate the duration of the press
      long pressDuration = releasedTime - pressedTime;

      //if the duration is longer than the required time for the long press, we set that variable to true
      if(pressDuration > LONG_PRESS_TIME)
      {
        //we check here for the long press mutex, and set the variable accordingly if it can
        //and gives it back afterwards
        if(xSemaphoreTake(xLongPressMutex, portMAX_DELAY) == pdTRUE)
        {
          longPressDetected = true;
          xSemaphoreGive(xLongPressMutex);
        }
      }
      else if (pressDuration<= LONG_PRESS_TIME)
      {
        updateVariable(BTN_State, true, xButtonBiSem);
      }
    }
    else
    {
      updateVariable(BTN_State, false, xButtonBiSem);
    }
   
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

bool preIL = false;
bool postIL = false;
//Checks for the switch's state, changes the switch state variable, and writes that state to LED1
void setIndoorLED1(void * parameters)
{
  
  while(1)
  {  
    switchy.loop();
    int swLocal;
    //Reads the state of switch and sets the output of LED1 accordingly
    swLocal = !switchy.getState();
    updateVariable(SWTCH_State, swLocal, xSwitchBiSem);
    digitalWrite(WHITE_LED1,SWTCH_State);
    postIL = swLocal;

    if (postIL != preIL)
    {
      xQueueSend(xIndoorLightChange, &postIL, 0);
    }
    preIL = postIL;

    vTaskDelay(10/portTICK_PERIOD_MS);

  }
}


//1.5Kohm resistors and .1microFarad capacitors used for IR sensors (1Hz cutoff)
void setIndoorIR(void * parameters)
{
  while(1)
  {
    bool IRActive;
    //Sees if it receives anything from the IR queue, and if so, we update the car presence variable
    if (xQueueReceive(xTaskIn, &IRActive, portMAX_DELAY))
    {
      if (IRActive == true)
      {
        updateVariable(carPresenceInside, IRActive, xCPIMutex);
      }
    }
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
 
}

//1.5Kohm resistors and .1microFarad capacitors used for IR sensors (1Hz cutoff)
void setOutdoorIR(void * parameters)
{
  while(1)
  {
    bool IRActive;
    //Sees if it receives anything from the IR queue, and if so, we update the car presence variable
    if (xQueueReceive(xTaskOut, &IRActive, portMAX_DELAY))
    {
      if (IRActive == true)
      {
        updateVariable(carPresenceOutside, IRActive, xCPOMutex);
      }
    }
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}


//Possible states for garage door
enum doorState
{
  TOCLOSE,
  TOOPEN,
  CLOSING,
  OPENING,
  CLOSED,
  OPENED,
  OPENINGTOCLOSING,
  CLOSINGTOOPENING
};


SemaphoreHandle_t xDoorStateMutex;
//global variable but only for servo motor use
enum doorState curStat = CLOSED;
//If you wanted to query what direction the motor is moving
int pos;
//Variables to detect change
int preS = 5; //1 is TOCLOSE, 2 is TOOPEN, 3 is CLOSING, 4 is OPENING, 5 is CLOSED, 6 is OPENED, 7 is OPENINGTOCLOSING, 8 is CLOSINGTOOPENING
int postS = 5;

//Garage door task
void setServo(void * parameters)
{
  //Local variables to control the servo motor and its position according to time
  unsigned long intermediateEndTime;
  unsigned long startTime;
  unsigned long now;
  unsigned long endTime = 1000/portTICK_PERIOD_MS;

  
  while(1)
  {
    //Serial.println("In setServo code");
    bool cPO;
    bool cPI;
    updateVariable(cPO, carPresenceOutside, xCPOMutex);
    updateVariable(cPI, carPresenceInside, xCPIMutex);

    enum doorState garLoc;
    updateVariable(garLoc, curStat, xDoorStateMutex);


    //stores the button state in a local variable so we can use it elsewhere
    bool buttony;
    updateVariable(buttony, BTN_State, xButtonBiSem);

    //If statements that open the servo when a car is outside and the garage is empty, or closes the garage when a car is inside and there's no one outside
    if ( ( (garLoc == CLOSED) && (cPO == true) && (cPI == false) ) )
    {
      garLoc = TOOPEN;
    }
    else if ( ( (garLoc == OPENED) && (cPO == false) && (cPI == true) ) )
    {
      garLoc = TOCLOSE;
    }
   
    //If statements that switch the servo from opening to closing or closing to opening if car presence is detected or button is pressed
    //Closing to opening if button is pressed or car is outside
    if((buttony == true || cPO == true) && garLoc == CLOSING)
    {
      garLoc = CLOSINGTOOPENING;
      intermediateEndTime = now;
      startTime = xTaskGetTickCount();
      now = xTaskGetTickCount() - startTime;
      pos = 180;
      motor.write(pos);
    }
    //Opening to closing if button is pressed or car is inside
    else if ((buttony == true || cPI == true) && garLoc == OPENING)
    {
      garLoc = OPENINGTOCLOSING;
      intermediateEndTime = now; //Amount of time the door needs to go to close
      startTime = xTaskGetTickCount(); //The time the door switches direction
      now = xTaskGetTickCount() - startTime; // Recalibrates now to be 0 basically
      pos = 0; // makes close
      motor.write(pos);
    }
    //If it's pressed (or detected) again then these if statements switch the states again, but using different maths
    else if((buttony == true || cPO == true) && garLoc == OPENINGTOCLOSING)
    {
      intermediateEndTime = endTime - (intermediateEndTime - now); //10 - (4 - 2)
      startTime = xTaskGetTickCount();
      now = xTaskGetTickCount() - startTime;
      garLoc = CLOSINGTOOPENING;
      pos = 180;
      motor.write(pos);
    }
    else if ((buttony == true || cPI == true) && garLoc == CLOSINGTOOPENING)
    {
      intermediateEndTime = endTime - (intermediateEndTime - now); //10 - (4 - 2)
      startTime = xTaskGetTickCount();
      now = xTaskGetTickCount() - startTime;
      garLoc = OPENINGTOCLOSING;
      pos = 0;
      motor.write(pos);
    }

    //These if statements simply open or close the garage with a button press when the garage is opened or closed
    if ((buttony == true) && (garLoc == CLOSED))
    {
      garLoc = TOOPEN;
    }
    else if ((buttony == true) && (garLoc == OPENED))
    {
      garLoc = TOCLOSE;
    }
   
    //Used car presence variables so setting them to false so they can be used again
    updateVariable(carPresenceOutside, false, xCPOMutex);
    updateVariable(carPresenceInside, false, xCPIMutex);



    //switch for garage door status to open, close, etc.
    switch(garLoc)
    {
      case(TOCLOSE): // Basically initializes the closing process and prevents adverse interactions with other if statements by providing an initial state that isn't closing
        startTime = xTaskGetTickCount();
        garLoc = CLOSING;
        pos = 0;
        motor.write(pos);
        postS = 1;
        break;

      case(TOOPEN): // Basically initializes the opening process and prevents adverse interactions with other if statements by providing an initial state that isn't opening
        startTime = xTaskGetTickCount();
        garLoc = OPENING;
        pos = 180;
        motor.write(pos);
        postS = 2;
        break;

      case(CLOSING)://Closes the garage for the set duration
        now = xTaskGetTickCount() - startTime;
        if (now >= endTime)
        {
          garLoc = CLOSED;
          pos = 90;
          motor.write(90);
        }
        postS = 3;
        break;


      case(OPENING)://Opens the garage for the set duration
        now = xTaskGetTickCount() - startTime;
        if (now >= endTime)
        {
          garLoc = OPENED;
        }
        postS = 4;
        break;
     
      case(OPENINGTOCLOSING): //Provides an intermediate state that moves the servo from opening to closing
        now = xTaskGetTickCount() - startTime;
        if (now>=intermediateEndTime)
        {
          garLoc = CLOSED;
        }
        postS = 7;
        break;
     
      case(CLOSINGTOOPENING): // Provides an intermediate state that moves the servo from closing to opening
        now = xTaskGetTickCount() - startTime;
        if (now>=intermediateEndTime)
        {
          garLoc = OPENED;
        }
        postS = 8;
        break;


      case(CLOSED)://Closed state, it's closed, pretty simple
        pos = 90;
        motor.write(pos);
        postS = 5;
        break;


      case(OPENED)://It's OPEN to receiving cars, get it?
        pos = 90;
        motor.write(pos);
        postS = 6;
        break;
    }
   
    updateVariable(curStat, garLoc, xDoorStateMutex);

    if (postS != preS)
    {
      xQueueSend(xGarageDoorChange, &postS, 0);
    }
    preS = postS;
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
 
}


// 0: full speed clockwise
// 90: stopped
//180: full speed counter clockwise

//Updates the light sensor and value
void setLightSen(void*parameters)
{
  while(1)
  {
    //Checks to see if poll timer has gone off, if so, it gets info from sensor every 100ms
    bool lightLocal;
    if(xQueueReceive(xLightPollQueue, &lightLocal, 0) == pdTRUE)
    {
      float lLocal = lightFilter.filter(analogRead(LIGHTSENPIN));
      int liLocal = int(lLocal); 
      updateVariable(lightSen, liLocal, xLightSenMutex);
    }
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

//Takes control of the LED if the area outside is dark and outputs a certain brightness on the LED
bool preLD = false; //variables to detect change
bool postLD = false;
void setLEDDark(void*parameters)
{ 
  while(1)
  {
    int lighty;
    updateVariable(lighty, lightSen, xLightSenMutex);

    //If the light sensor detects that it is dark enough outside, it turns on the outside led at a brightness of 50
    if(lighty<2500)
    {
      analogWrite(WHITE_LED2, 50);
      postLD = true;
    }
    else
    {
      //turns it off if too bright outside
      analogWrite(WHITE_LED2, 0);
      postLD = false;
    }

    //If pre doesn't equal post, we know the state has changed
    if (postLD != preLD)
    {
      xQueueSend(xOutdoorLightChange, &postLD, 0);
    }
    preLD = postLD;
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

//Updates the temperature sensor and value
void setTempSen(void * parameters)
{
  while(1)
  {
    bool tempLocal;
    //Checks to see if poll timer has gone off, if so, it gets info from sensor every 100ms
    if(xQueueReceive(xTempPollQueue, &tempLocal, 0) == pdTRUE)
    {
      //gets the value from the temp sensor and writes it to local variable, then we write it to tempy using mutex

      if(xSemaphoreTake(xI2CMutex, 40/portTICK_PERIOD_MS) == pdPASS)
      {
        float tmLocal;
        tmLocal = tempFilter.filter(hmSen.readTemperature());
        updateVariable(tempy, tmLocal, xTempSenMutex);
        xSemaphoreGive(xI2CMutex);
      }
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

//Task to read from smoke sensor and run fan if necessary
float smokeGlobal = 0;
void setSmokeSen(void * parameters)
{
  while(1)
  {
    //get smoke info from sensor and update global variable
    float smokeLocal = smokeFilter.filter(analogRead(SMOKESENPIN));
    updateVariable(smokeGlobal, smokeLocal, xSmokeMutex);

    //If smoke is high enough, we run the fan for 20 seconds
    //Because the smoke sensor's dedicated purpose is to tell the fan when to run, it's okay to use vtaskdelay
    //After delay, the sensor checks again and runs the fan more if necessary
    if(smokeLocal > 1000)
    {
      digitalWrite(FAN, HIGH);
      vTaskDelay(20000/portTICK_PERIOD_MS);
      digitalWrite(FAN, LOW);
    }

    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}


bool controlVar = true;
bool contro = false;

bool alarmSpeakerState = false;
SemaphoreHandle_t xSpeakerStateMutex;

int postAS = 1;
int preAS = 1; //1 is inactive, 2 is active, 3 is intruder for status changes read in the SD card task
//Checks conditions and changes alarmState accordingly, and the code for a particular state in the switch
void setAlarmState(void * parameters)
{
  
  while(1)
  {
    //Takes the Alarm binary semaphore
    enum State alarmLocal;
    updateVariable(alarmLocal, alarmState, xAlarmBiSem);
    bool lPDLocal;
    updateVariable(lPDLocal, longPressDetected, xLongPressMutex);
   
    //If statement to switch state from inactive to active if a long press is detected
    if((alarmLocal==INACTIVE) && (lPDLocal == true))
    {
      updateVariable(alarmState, ACTIVE, xAlarmBiSem);
      postAS = 2;
    }


    //If statements to switch state from active or intruder to inactive if long press is detected
    else if((alarmLocal == ACTIVE) && (lPDLocal == true))
    {
      updateVariable(alarmState, INACTIVE, xAlarmBiSem);
      postAS = 1;
    }
    else if ((alarmLocal == INTRUDER) && (lPDLocal == true))
    {
      updateVariable(alarmState, INACTIVE, xAlarmBiSem);
      postAS = 1;
    }


    //If statement to switch state from active to intruder if door is open
    else if((alarmLocal == ACTIVE) && (HALL_State <1850))
    {
      updateVariable(alarmState, INTRUDER, xAlarmBiSem);
      postAS = 3;
      controlVar = true;
      contro = false;
      //Starts the alarm and time out timer
     
      while(xTimerStart(xAlarmTimeOutTimer, 0) != pdPASS)
      {}
      while(xTimerStart(xAlarmTimer, 0) != pdPASS)
      {}
     
      xQueueReset(xAlarmTimerQueue);
      xQueueReset(xAlarmTimeOutTimerQueue);
     
    }
    //Updates long press variable to false
    updateVariable(longPressDetected, false, xLongPressMutex);

    //If pre doesn't equal post, we know the state has changed
    if (postAS != preAS)
    {
      xQueueSend(xAlarmStatusChange, &postAS, 0);
    }
    preAS = postAS;


    //Switch for the alarmState variable so it runs the code for the appropriate state
    switch(alarmLocal)
    {
      //If inactive, only the blue LED is on and alarm is off
      case(INACTIVE):
        //Updating rgb using mutex
        xSemaphoreTake(xRGBLEDMutex, portMAX_DELAY);
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);
        xSemaphoreGive(xRGBLEDMutex);

        //Updating buzzer
        digitalWrite(BUZZER, LOW);
        break;


      //If active, only the green LED is on and alarm is off
      case (ACTIVE):
        //Updating RGB using mutex
        xSemaphoreTake(xRGBLEDMutex, portMAX_DELAY);
        digitalWrite(RED_LED, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        xSemaphoreGive(xRGBLEDMutex);

        //Updating buzzer
        digitalWrite(BUZZER, LOW);
        break;


      case(INTRUDER):
        //semaphore for changing the LEDs
        xSemaphoreTake(xRGBLEDMutex, portMAX_DELAY);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
        xSemaphoreGive(xRGBLEDMutex);
       
        bool alarmTimerLocal;
        bool timeOutLocal;


        //If we receive a message from the time out timer, we turn off the code that makes the buzzer go on
        //and off and instead turn the buzzer off
        if(xQueueReceive(xAlarmTimeOutTimerQueue, &timeOutLocal, 0) == pdTRUE)
        {
          contro = true;
        }
       
        //If no message is received from time out timer, we run the buzzer code
        if(contro == false)
        {
          //If we receive a message from the alarm timer, we flip the alarm state using controlVar
          if(xQueueReceive(xAlarmTimerQueue, &alarmTimerLocal, 0) == pdTRUE)
          {
            controlVar = !controlVar;
          }
          if(controlVar == true)
          {
            digitalWrite(BUZZER, HIGH); // Buzzer on
            updateVariable(alarmSpeakerState, true, xSpeakerStateMutex);
          }
          else if (controlVar == false)
          {
            digitalWrite(BUZZER, LOW); // Buzzer off
            updateVariable(alarmSpeakerState, false, xSpeakerStateMutex);
          }
        }
        else if (contro == true) //Time out active
        {
          digitalWrite(BUZZER, LOW); //buzzer off until we go to a different state and come back
          updateVariable(alarmSpeakerState, false, xSpeakerStateMutex);
        }
        break;
    }  
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}



//variables to help detect change in door status
bool preDS = false;
bool postDS = false;
//Updates the door/hall sensor - higher value means magnet detected i.e. closed, low means less magnet detected i.e. open
void setDoorState(void * parameters)
{
  while(1)
  {
    //Local variable that stores hall sensor info, then we write it to HALL_State global variable
    int hallLocaly = hallFilter.filter(analogRead(HAL));
    updateVariable(HALL_State, hallLocaly, xHallBiSem);
    vTaskDelay(10/portTICK_PERIOD_MS);

    if(hallLocaly >= 1850)
    {
      postDS = true; //closed
    }
    else if(hallLocaly < 1850)
    {
      postDS = false; //opened
    }

    //If pre doesn't equal post, we know the state has changed
    if (postDS != preDS)
    {
      xQueueSend(xInsideDoorChange, &postDS, 0);
    }
    preDS = postDS;
  }
}


float humidPercent; //global humidity variable
SemaphoreHandle_t xHmdMutex;
//I2C - reading humidity sensor via I2C
void setHumiditySensor(void * parameters)
{
  while(1)
  {
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    float hmLocal = humidFilter.filter(hmSen.readHumidity());
    xSemaphoreGive(xI2CMutex);
    updateVariable(humidPercent, hmLocal, xHmdMutex);
    
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}


//Small screen I2C
//Display door, garage, and alarm status
void setOLED(void * parameters)
{
  while(1)
  {
    //Local variable for alarm status
    enum State alarmLocal;
    updateVariable(alarmLocal, alarmState, xAlarmBiSem);

    //Local variable for door status
    int hallLocal;
    updateVariable(hallLocal, HALL_State, xHallBiSem);

    //Local variable for garage door status
    enum doorState garDoorLoc;
    updateVariable(garDoorLoc, curStat, xDoorStateMutex);

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    //Reset screen for new writes
    oled.clearDisplay();
    oled.setTextColor(WHITE);

    //Writes door status at 0,0
    oled.setCursor(0,0);
    //Door state if statement
    if (hallLocal>1850)
    {
      oled.println("Door Closed");
    }
    else if (hallLocal<=1850)
    {
      oled.println("Door Open");
    }


    //Writes alarm status at 0,10
    oled.setCursor(0,10);
    if(alarmLocal == INACTIVE)
    {
      oled.println("Alarm Inactive");
    }
    else if(alarmLocal == ACTIVE)
    {
      oled.println("Alarm Active");
    }
    else if(alarmLocal == INTRUDER)
    {
      bool alarmss;
      updateVariable(alarmss, alarmSpeakerState, xSpeakerStateMutex);
      if(alarmSpeakerState ==true)
      {
        oled.println("Alarm Intruder");
        oled.drawBitmap(0, 30, spk_bitmap_Speaker_Icon, 30, 30, WHITE);
      }
      else if(alarmSpeakerState == false)
      {
        oled.println("Alarm Intruder");
        oled.drawBitmap(0, 30, spk_bitmap_Speaker_off, 30, 30, WHITE);
      }
      
    }
   
    //Write Garage status at 0,20
    oled.setCursor(0,20);
    switch(garDoorLoc)
    {
      case(TOCLOSE):
        oled.println("Garage Closing");
        break;
      case(TOOPEN):
        oled.println("Garage Opening");
        break;
      case(CLOSING):
        oled.println("Garage Closing");
        break;
      case(OPENING):
        oled.println("Garage Opening");
        break;
      case(OPENED):
        oled.println("Garage Opened");
        break;
      case(CLOSED):
        oled.println("Garage Closed");
        break;
      case(OPENINGTOCLOSING):
        oled.println("Garage Closing");
        break;
      case(CLOSINGTOOPENING):
        oled.println("Garage Opening");
        break;
    }

    //Displays these onto OLED
    oled.display();
    xSemaphoreGive(xI2CMutex);
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}


//Temperature, humidity status
//Large Screen I2C
void setLCD(void * parameters)
{
  while(1)
  {
    //Local variables for temperature
    float taku;
    updateVariable(taku, tempy, xTempSenMutex);
    String tm = String(taku);

    //Getting the humidity value to print
    float humidLocal;
    updateVariable(humidLocal, humidPercent, xHmdMutex);
    String hm = String(humidLocal);
    
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    //Clears LCD, writes the temp value at 0,0 and humidity at 0,1
    I2C_LCD1.clear();
    I2C_LCD1.print("Temp:");
    I2C_LCD1.print(tm);
    
    I2C_LCD1.setCursor(0,1);
    I2C_LCD1.print("Humidity:" + hm);
    xSemaphoreGive(xI2CMutex);
    vTaskDelay(30/portTICK_PERIOD_MS);
    
  }
}

//function to append value to a file
void appendFile(fs::FS &fs, const char * path, String message){
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}



//Function to get current timecode from time derrived from NTP server
String getTimeCode()
{

  //We grab the different time aspects we want and concatenate them into a string that the function returns.
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);

  char timeMonth[10];
  strftime(timeMonth, 10, "%B", &timeinfo);

  char timeMonthDay[3];
  strftime(timeMonthDay, 3, "%d", &timeinfo);

  char timeYear[5];
  strftime(timeYear, 5, "%Y", &timeinfo);

  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);

  char timeMinute[3];
  strftime(timeMinute, 3, "%M", &timeinfo);

  char timeSecond[3];
  strftime(timeSecond, 3, "%S", &timeinfo);

  String gm = String(timeWeekDay) + ", " + String(timeMonth) + " " + String(timeMonthDay) + " " + String(timeYear) + " " + String(timeHour) + ":" + String(timeMinute) + ":" + String(timeSecond) + " - ";
  return gm;
}

//Logs indoor light change, outdoor light change, house door status change, garage door status change, and alarm status change to SD
void writeSD(void * parameters)
{
  while(1)
  {
    //variables to store values received from the queues

    String TC = getTimeCode();
    uint8_t cardType = SD.cardType();
    //If there is a card inside, then we write logs to it
    if(cardType != CARD_NONE)
    {
      //Receive sensor info from queues and write to SD
      //Receiving a value from these queues means the state of that value has changed, so we write that it has changed to the SD card
      //If we receive an item in the indoor light change queue, log it in the sd file using the append function

      bool ILreceive;
      if(xQueueReceive(xIndoorLightChange, &ILreceive, 0) == pdTRUE)
      {
        String msg;
        switch(ILreceive)
        {
          case(true):
            msg = TC + "Indoor Light is turned on\n";
            break;
          case(false):
            msg = TC + "Indoor Light is turned off\n";
            break;
        }
        appendFile(SD, "/logs.txt", msg);
      }


      //Outdoor light change receive
      bool OLreceive;
      if(xQueueReceive(xOutdoorLightChange, &OLreceive, 0) == pdTRUE)
      {
        String msg;
        if(OLreceive == true)
        {
          msg = TC + "Night Light is turned on\n";
        }
        else if(OLreceive == false)
        {
          msg = TC + "Night Light is turned off\n";
        }
        appendFile(SD, "/logs.txt", msg);
      }

      bool IDreceive;
      //House door change receive
      if(xQueueReceive(xInsideDoorChange, &IDreceive, 0) == pdTRUE)
      {
        String msg;
        if(IDreceive == true)
        {
          msg = TC + "Front door is closed\n";
        }
        else if(IDreceive == false)
        {
          msg = TC + "Front door is opened\n";
        }
        appendFile(SD, "/logs.txt", msg);
      }

      int GDreceive;
      //Garage door change receive
      if(xQueueReceive(xGarageDoorChange, &GDreceive, 0) == pdTRUE)
      {
        String msg;
        switch(GDreceive)
        {
          //Switch statement to print specific status statements to SD about the garage state
          //1 is TOCLOSE, 2 is TOOPEN, 3 is CLOSING, 4 is OPENING, 5 is CLOSED, 6 is OPENED, 7 is OPENINGTOCLOSING, 8 is CLOSINGTOOPENING
          case(1):
            msg = TC + "Garage Door is about to close\n";
            break;
          case(2):
            msg = TC + "Garage Door is about to open\n";
            break;
          case(3):
            msg = TC + "Garage Door is closing\n";
            break;
          case(4):
            msg = TC + "Garage Door is opening\n";
            break;
          case(5):
            msg = TC + "Garage Door is closed\n";
            break;
          case(6):
            msg = TC + "Garage Door is opened\n";
            break;
          case(7):
            msg = TC + "Garage Door switched from opening to closing\n";
            break;
          case(8):
            msg = TC + "Garage Door switched from closing to opening\n";
            break;
        }
        appendFile(SD, "/logs.txt", msg);
      }

      int ASreceive;
      //Alarm status change receive
      if(xQueueReceive(xAlarmStatusChange, &ASreceive, 0) == pdTRUE)
      {
        String msg;
        switch(ASreceive)
        {
          //1 is inactive, 2 is active, 3 is intruder
          case(1):
            msg = TC + "Alarm State is inactive\n";
            break;
          case(2):
            msg = TC + "Alarm State is active\n";
            break;
          case(3):
            msg = TC + "Alarm State is intruder\n";
            break;
        }
        appendFile(SD, "/logs.txt", msg);
      }
    }


    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

//prints the Door And LED (DAL) states every 10 seconds 
void repDALState(void * parameters)
{
  while(1)
  {
    //Code to report whether or not the door is open
   
    int hallLocal;
    updateVariable(hallLocal, HALL_State, xHallBiSem);
    if (hallLocal>1900)
    {
      Serial.println("Door Closed");
    }
    else if (hallLocal<=1900)
    {
      Serial.println("Door Open");
    }



    //Code to report whether the inside door is on
    int switchLocal;
    updateVariable(switchLocal, SWTCH_State, xSwitchBiSem);
    if (switchLocal == 1)
    {
      Serial.println("Indoor light is on");
    }
    else if (switchLocal==0)
    {
      Serial.println("Indoor light is off");
    }


   
    //Code to report whether the nightlight is on or not
    int lightOutsideLocal;
    updateVariable(lightOutsideLocal, lightSen, xLightSenMutex);
    if(lightOutsideLocal<2500)
    {
      Serial.println("Night light is on");
    }
    else
    {
      Serial.println("Night light is off");
    }



    //Code to see the state of the alarm
    enum State alarmLocal;
    updateVariable(alarmLocal, alarmState, xAlarmBiSem);
    if(alarmLocal == INACTIVE)
    {
      Serial.println("In Inactive mode");
    }
    else if(alarmLocal == ACTIVE)
    {
      Serial.println("In Active mode");
    }
    else if(alarmLocal == INTRUDER)
    {
      Serial.println("In Intruder mode");
    }
   
   
    //Code to report the temperature in Celsius
    Serial.print("Temperature in Celsius: ");
    float taku;
    updateVariable(taku, tempy, xTempSenMutex);
    Serial.println(taku);
   
    Serial.println("");

    //delays this task for 10 seconds
    vTaskDelay(10000/portTICK_PERIOD_MS);
    }
}


//Interrupt for indoor IR
void IRAM_ATTR Ext_INT1_ISR()
{
  //local variable for queue send
  bool inyIRActive = true;
  //Sends a value to a the xTaskIn queue which tells the setIndoorIR task that the sensor is active!
  xQueueSendFromISR(xTaskIn, &inyIRActive, NULL);
}


//Interrupt for outdoor IR
void IRAM_ATTR Ext_INT2_ISR()
{
  //Local variable for queue send
  bool outyIRActive = true;
  //Sends a value to a the xTaskOut queue which tells the setOutdoorIR task that the sensor is active!
  xQueueSendFromISR(xTaskOut, &outyIRActive, NULL);
}

const char* ssid     = "Apollos";
const char* password = "password";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;

 //Setup code
void setup ()
{
  //Setting up baud rate and write message to signal start of serial writing
  Serial.begin(115200);
  Serial.println("Serial Writing start");
  
  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int timeOut = 0;
  int skipLocalTime = 0;
  //Tries to connect to wifi, but timesout if it can't after about 10 seconds
  while ((WiFi.status() != WL_CONNECTED) && timeOut< 20) {
    delay(500);
    Serial.print(".");
    timeOut++;
  }
  Serial.println("");
  //If we connect, we send a message, otherwise we turn off the wifi mode 
  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi connected.");
  }
  else
  {
    Serial.println("WiFi not connected.");
    skipLocalTime = 1;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //Only try to get local time if wifi is connected
  //If wifi is connected and getting local time doesn't work immediately, give it about 10 secs, then we disconnect wifi
  struct tm timeinfo;
  if((skipLocalTime == 0) && (getLocalTime(&timeinfo) != true)){
    delay(10000/portTICK_PERIOD_MS);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  //Setting the pin modes of the LEDS amd Buzzer to output and the switch and Hall Effect Sensor to pull down config
  //Because the button and switch are EZbutton objects, they're in pull up configs
  pinMode(WHITE_LED1, OUTPUT);
  pinMode(WHITE_LED2, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(SMOKESENPIN, INPUT);
  pinMode(LIGHTSENPIN, INPUT);
  pinMode(IRin, INPUT_PULLUP);
  pinMode(IRout, INPUT_PULLUP);

  //Creating some queues
  xTaskIn = xQueueCreate(10, sizeof(bool));
  xTaskOut = xQueueCreate(10, sizeof(bool));
  xAlarmTimerQueue = xQueueCreate(10, sizeof(bool));
  xAlarmTimeOutTimerQueue = xQueueCreate(10, sizeof(bool));
  xTempPollQueue = xQueueCreate(10, sizeof(bool));
  xLightPollQueue = xQueueCreate(10, sizeof(bool));

  //Status change queues
  xIndoorLightChange = xQueueCreate(10, sizeof(bool));
  xOutdoorLightChange = xQueueCreate(10, sizeof(bool));
  xInsideDoorChange = xQueueCreate(10, sizeof(bool));
  xGarageDoorChange = xQueueCreate(10, sizeof(int));
  xAlarmStatusChange = xQueueCreate(10, sizeof(int));

  //Creating the timers
  xAlarmTimer = xTimerCreate("AlarmTimer", ALARM_PERIOD, pdTRUE, 0, alarmTimerCallback); //1 second, auto
  xAlarmTimeOutTimer = xTimerCreate("TimeOutTimer", TIMEOUT_PERIOD, pdFALSE, 0, alarmTimeOutCallback); //120 seconds, one shot
  xSensorPollingTimer = xTimerCreate("SensorPollTimer", POLL_PERIOD, pdTRUE, 0, sensorPollCallback); //100 ms, auto

  //creating the mutexes
  xAlarmBiSem = xSemaphoreCreateMutex();
  xButtonBiSem = xSemaphoreCreateMutex();
  xSwitchBiSem = xSemaphoreCreateMutex();
  xHallBiSem = xSemaphoreCreateMutex();
  xCPOMutex = xSemaphoreCreateMutex();
  xCPIMutex = xSemaphoreCreateMutex();
  xLongPressMutex = xSemaphoreCreateMutex();
  xRGBLEDMutex = xSemaphoreCreateMutex();
  xBuzzerMutex = xSemaphoreCreateMutex();
  xLightSenMutex = xSemaphoreCreateMutex();
  xTempSenMutex = xSemaphoreCreateMutex();
  xDoorStateMutex = xSemaphoreCreateMutex();
  xHmdMutex = xSemaphoreCreateMutex();
  xI2CMutex = xSemaphoreCreateMutex();
  xSpeakerStateMutex = xSemaphoreCreateMutex();
  xSmokeMutex = xSemaphoreCreateMutex();

  //SPI - SD card stuff
  spi.begin(SCK, MISO, MOSI, CS);
  SD.begin(CS,spi,25000000); //25MHz
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
  }
  else
  {
    String msg = getTimeCode() + "System Boot\n";
    appendFile(SD, "/logs.txt", msg);
  }
  

  //PWM timers for servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor.setPeriodHertz(50);    // standard 50 hz servo
  motor.attach(SERV, 900, 2000); //900 to 2000 pulse width

  //Interrupt falling attaches to IR pins
  attachInterrupt(IRin, Ext_INT1_ISR, FALLING);
  attachInterrupt(IRout, Ext_INT2_ISR, FALLING);
  //sets the debounce time for the button and switch
  button.setDebounceTime(DEBOUNCE_TIME);
  switchy.setDebounceTime(DEBOUNCE_TIME);

  //I2C_0.begin(SDA0pin, SCL0pin, I2C_Freq);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  //Humidity sensor begin
  hmSen.begin(0x44);

  // Initialize The I2C LCD
  I2C_LCD1.init();
  // Turn ON The Backlight
  I2C_LCD1.backlight();


  //Starts the tasks for the program
  xTaskCreate(setTempSen, "TempSenState", 3000, NULL, 1, NULL);
  xTaskCreate(setLightSen, "LightSenState", 3000, NULL, 1, NULL);
  
  xTaskCreate(setHumiditySensor, "HumiditySensor", 3000, NULL, 1, NULL);
  xTaskCreate(writeSD, "SDLogs", 6000, NULL, 2, NULL);
  
  xTaskCreate(setDoorState, "DoorState", 3000, NULL, 2, &DoorStat);
  xTaskCreate(setSmokeSen, "SmokeSen", 5000, NULL, 1, NULL);

  xTaskCreate(setAlarmState, "AlarmState", 6000, NULL, 1, &AlmStat);
  xTaskCreate(setIndoorLED1, "IndoorLED1", 3000, NULL, 1, &IndLED1);

  xTaskCreate(setButtonState, "ButtonState", 3000, NULL, 2, &BtnStat);
  xTaskCreate(repDALState, "DALState", 3000, NULL, 1, &DALStat);

  xTaskCreate(setLEDDark, "LedDark", 3000, NULL, 1, NULL);
  xTaskCreate(setServo, "ServoState", 3000, NULL, 2, NULL);

  xTaskCreate(setIndoorIR, "IndoorIR", 3000, NULL, 1, &IrIns);
  xTaskCreate(setOutdoorIR, "OutdoorIR", 3000, NULL, 1, &IrOuts);

  xTaskCreate(setOLED, "SetOLEDScreen", 3000, NULL, 1, NULL);
  xTaskCreate(setLCD, "SetLCDScreen", 3000, NULL, 1, NULL);

  //Where we start the sensor polling timer
  while(xTimerStart(xSensorPollingTimer, 0) != pdPASS)
  {}

  //deletes looptask
  vTaskDelete(NULL);
}

//loop code is empty
void loop()
{}













