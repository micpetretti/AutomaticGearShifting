/*
  Automatic gear shifting.
  
Authors:  Michael Petretti (michael@petretti.de)
          Björn Hagemeister
          @ University of Freiburg, Germany, 2015.
  
**************************************************************************************
    Front and rear in seperated Arrays now. 
    Front: Small Chainblade is left and therefor no. 1 in Array.
    Rear: Biggest is left and therefor no. 1 in Array.
    Shifting up can be done by simply increase a position in either array. 
    -> means currentFront++ is shifting to a bigger gear the same way as
    currentRear-- is shifting down and the other way round.
    
    To prevent wearing out the mechanics I decided to not use the gears 1-9, 1-10, 2-1, 2-2,
    where the first digit is front chainblade and second is rear chainblade.
    
    To avoid unnecessary shift operations I choose the following plan to shift gears:
    Shifting up: 1-1, 1-2, .... , 1-8, 2-4, 2-5, .... , 2-10,
    Shifting down: 2-10, 2-9, .... , 2-3, 1-7, 1-6, .... , 1-1
    
    I have decided to write a "void generateSignal(int count, int mode)" Methode. 
    count = the number of gears one wants to shift
    mode: 1 = front up, 2 = front down, 3 = rear up, 4 = rear down  
    
    Connections at arduino:
    Arduino <--> Relais:
        VCC -- 5V
        IN4 -- Pin 8  (rear up)
        IN3 -- Pin 12 (rear down)
        IN2 -- Pin 4  (front up)
        IN1 -- Pin 7  (front down)
    Arduino <--> board:
        TX(bluetooth) -- Pin 10
        RX(bluetooth) -- Pin 11
        Velocity senor pin -- Pin 2
        crank frequency sensor pin -- Pin 3
    Relais <--> shifting (bike)
        Connect the wires of the back shifting to the left from left to right:
        (free) red blue (free) green yellow
        
        Connect the wires of the front shifting to the right from left to right:
        (free) red blue (free) green yellow
**************************************************************************************
Bluetooth - communication:
    Using SoftwareSerial(RX, TX) for bluetooth communication to beware hardware serial
    for debugging information (current speed, RPM, ...).
    
    set up a new serial port:
        SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
    What should be communicated to/from the smartphone via bluetoot?
    --> Key words are necessary for receiving and sending Key=Value - Pairs.
        Sending information:
                - current velocity                                                --> Key = VELOCITY
                - current crank frequency                                         --> Key = CURR_FREQUENCY
                - current gear (front gear) [                                      --> Key = FRONT_GEAR
                - current gear (rear gear)                                        --> Key = REAR_GEAR
                - current gradient at hill (right now no sensor, sending 0)       --> Key = GRADIENT
        Receiving information:
                - target crank frequency                                          --> Key = TAR_FREQUENCY
                - frequency divergence at a hill [high, middel, low]              --> Key = HILL_DIV
                - general frequency divergence (sensitivity) [%]                  --> Key = GENERAL_DIV
                - maybe RESET - signal for entering defined state (first gear).   --> Key = RESET
                - wheel circumference [zoll] --> intern calculation in [mm]       --> Key = CIRCUMFERENCE

    IDEA for SENDING:
          Interrupt timer: sending every X seconds VELOCITY, CURR_FREQUENCY, GEAR to smartphone
    IDEA for RECEIVING:
          Listen to serial port in main loop:
      
    How to send/receive data:
          Key=Value - Pairs. --> Delimiter is necessary to catch end of message.
          Use '!' as delimiter, example would be for sending velocity: "VELOCITY=35.5!"
          That the smartphone knows which data are from the arduino surround the information by "#...info...$".
          --> its easier for gino to extract the incoming information on the smartphone.
    
    Using pin 10 and 11 for SoftwareSerial.
    
**************************************************************************************
Bisher:
    Messung der Zeit zwischen zwei Peaks des Reed - Sensors, was der Zeit einer Rad - 
    drehung entspricht.
    Geschwindigkeit = Radumfang[mm] / Zeit[ms] * 3.6 = km/h
    Trittfrequenz = 60000 / (Zeit[ms] * Übersetzungsverhältnis(aktueller Gang)) = U/min
    
    Reedsensor ist entprellt, so dass nur auf eine Steigende Flanke bei Zeit reagiert wird.
    
    Geschwindigkeitsmessung passt sehr gut zu gekauftem Tacho.
*/
#define ARDUINO
#include <SoftwareSerial.h>
#include "usedMethods.h"

// Create new SoftwareSerial for bluetooth communication.
SoftwareSerial
  bluetoothSerial = SoftwareSerial(bluetoothRX, bluetoothTX);


// Variables only used in the arduino file.
long
  timePreviousHigh,            // time of last peak of wheel sensor [ms]
  timeActualHigh,              // time of current peak of wheel sensor [ms]
  crankTimePreviousHigh,       // time of last peak of crank sensor [ms]
  crankTimeActualHigh;         // time of current peak of crank sensor [ms]

// WheelDebounceTime:
float wheelDebouncingTime;


// Defining inputString, key and value.
  String
  inputString = "";    // inputString for reading from bluetooth
  String
  key = "";            // key for storing incoming key String
  String
  value = "";          // key for storing incoming value String
// Defining keys for bluetooth communication.
  const String
  VELOCITY = "VELOCITY";
// Sending information.
  const String
  CURR_FREQUENCY = "CURR_FREQUENCY";
  const String
  FRONT_GEAR = "FRONT_GEAR";
  const String
  REAR_GEAR = "REAR_GEAR";
  const String
  GRADIENT = "GRADIENT";
// Receiving information.
  const String
  TAR_FREQUENCY = "TAR_FREQUENCY";
  const String
  HILL_DIV = "HILL_DIV";
  const String
  GENERAL_DIV = "GENERAL_DIV";
  const String
  RESET = "RESET";
  const String
  CIRCUMFERENCE = "CIRCUMFERENCE";

// Defining values for bluetooth communication.
// Sending information.
  String
  VELOCITY_VAL = "";
  String
  CURR_FREQUENCY_VAL = "";
  String
  FRONT_GEAR_VAL = "";
  String
  REAR_GEAR_VAL = "";
  String
  GRADIENT_VAL = "0";
// Receiving information.
  String
  TAR_FREQUENCY_VAL = "";
  String
  HILL_DIV_VAL = "";
  String
  GENERAL_DIV_VAL = "";
  String
  RESET_VAL = "";
  String
  CIRCUMFERENCE_VAL = "";

// Separator.
const String SEPARATOR = "!";
// Equalsign.
const String EQUAL = "=";

// Begin of information
const String BEGIN = "#";
// End of information
const String END = "$";


void setup() {
  // initialize the input pins as INPUT.
  pinMode(inputPinS1, INPUT_PULLUP);
  pinMode(crankRpmInputPin, INPUT_PULLUP);
  // initilize output pins for shifting.
  pinMode(frontUpPin, OUTPUT);
  pinMode(frontDownPin, OUTPUT);
  pinMode(rearUpPin, OUTPUT);
  pinMode(rearDownPin, OUTPUT);
  // initialize pins for sending/receiving data via bluetooth.
  pinMode(bluetoothRX, INPUT);
  pinMode(bluetoothTX, OUTPUT);
  
  digitalWrite(frontUpPin, HIGH);
  digitalWrite(frontDownPin, HIGH);
  digitalWrite(rearUpPin, HIGH);
  digitalWrite(rearDownPin, HIGH);

  // using interrupt for measuring time between to edges.
  // 0 stands for inputPin = 2
  // 1 stands for inputPin = 3
  attachInterrupt(0, triggerSensor1, FALLING);
  attachInterrupt(1, triggerSensor2, FALLING);
  
  
  // Setup Timer for timer interrupt.
  cli(); // stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*prescaler) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts
  
  // Initilize some default values before receiving first information from smartphone.
  TAR_FREQUENCY_VAL = "90";
  GENERAL_DIV_VAL = "10";
  RESET_VAL = "0";
  CIRCUMFERENCE_VAL = "28";
  rad = 2110; // Circumference for 28" wheel.
  
  //initialize serial communication.
  Serial.begin(9600);
  bluetoothSerial.begin(9600);

/******************************************************************************************/ 
/******************************************************************************************/ 
  // IMPORTANT: AFTER ACTIVATING THE ARDUINO YOU HAVE 5 SECONDS BEFORE THE GEARS START SHIFTING
  // YOU MUST TURN THE PEDALS UNTILL THE DEFINED STATE OF 1-1 IS REACHED
/******************************************************************************************/ 
  delay(5000);
  generateSignal(1,frontDownPin);
  generateSignal(10,rearDownPin);

  currentFront = 1;
  currentRear = 1;
/******************************************************************************************/ 
  
  timePreviousHigh = millis();
  crankTimePreviousHigh = millis();
  
  // If we expecting a maximum speed of 70 km/h = 19,4 m/s, we get 19,4 m / 2,110 m = 9,2
  // interrupts per second. So the interrupt occures all 1000 / 9.2 = 108 ms.
  wheelDebouncingTime = 1000.0 / (19.4 / (float)rad);
}

void loop() {
  // Receive information from smartphone.
  receiveInformation();
  
  // Display speed and frequency only every second.
  delay(1000);
  Serial.print("Speed: ");
  Serial.print(currentSpeed);
  Serial.println(" km/h");
    
  Serial.print("Computed Crankfreq: ");
  Serial.print(frequency);
  Serial.println(" U/min");

  Serial.print("Measured Crankfreq: ");
  Serial.println(crankFrequency);
  
  Serial.print("Current Front: ");
  Serial.println(currentFront);
  
  Serial.print("Current Rear: ");
  Serial.println(currentRear);
  Serial.println("");
  
  mainIt();
  
  if (timePreviousHigh < (millis() - 1500)) {
    currentSpeed = 0.0;
    frequency = 0.0;
  }

  // target frequency - general Div.
  int tarFreqMinusDiv = TAR_FREQUENCY_VAL.toInt() - (((float)TAR_FREQUENCY_VAL.toInt() / 100.0f) * (float)GENERAL_DIV_VAL.toInt());
  // target frequency + general Div.
  int tarFreqPlusDiv = TAR_FREQUENCY_VAL.toInt() + (((float)TAR_FREQUENCY_VAL.toInt() / 100.0f) * (float)GENERAL_DIV_VAL.toInt());
  
   // Adapt to changes in current frequency.
  if (frequency < tarFreqMinusDiv) {
    gearDown();
  }
  if (frequency > tarFreqPlusDiv) {
    gearUp();
  }
}

// define interrupt service routine for timer declared above.
ISR(TIMER1_COMPA_vect){
  sendInformation();
}

// Method for triggering sensor one and cumpute current speed
// and calculate current crank frequency.
void triggerSensor1() {
  timeActualHigh = millis();
  difference = timeActualHigh - timePreviousHigh;
  
  // If not, wait for next occuring interrupt.
  if (difference >= wheelDebouncingTime) {
    computeSpeedAndFrequency();
    timePreviousHigh = millis();
  }
}

// Method for handling triggered Signal of sensor two and cumputing actual
// crank frequency.
void triggerSensor2() {
  crankTimeActualHigh = millis();
  crankDifference = crankTimeActualHigh - crankTimePreviousHigh;
  
  // Only compute new crank frequency, if crank difference is bigger than crankDebouncingTime.
  // If not, ignore current measurement and wait for next occuring interrupt.
  if (crankDifference >= crankDebouncingTime) {
    crankFrequency = 60000 / (float) crankDifference;
    
    crankTimePreviousHigh = millis();
  }
}

// Send all current information (velocitiy, current frequency, front gear, rear gear
// and gradient) to the smartphone via bluetooth.
void sendInformation() {
  // Serial.print("Sending information ... ");
  // Begin sending information.
  bluetoothSerial.print(BEGIN);
  // VELOCITY
  bluetoothSerial.print(VELOCITY);
  bluetoothSerial.print(EQUAL);
  bluetoothSerial.print(currentSpeed);
  bluetoothSerial.print(SEPARATOR);
  
  // CURR_FREQUENCY
  bluetoothSerial.print(CURR_FREQUENCY);
  bluetoothSerial.print(EQUAL);
  bluetoothSerial.print(frequency);
  bluetoothSerial.print(SEPARATOR);
  
  // FRONT_GEAR
  bluetoothSerial.print(FRONT_GEAR);
  bluetoothSerial.print(EQUAL);
  bluetoothSerial.print(currentFront);
  bluetoothSerial.print(SEPARATOR);
  
  // REAR_GEAR
  bluetoothSerial.print(REAR_GEAR);
  bluetoothSerial.print(EQUAL);
  bluetoothSerial.print(currentRear);
  bluetoothSerial.print(SEPARATOR);
  
  // GRADIENT
  bluetoothSerial.print(GRADIENT);
  bluetoothSerial.print(EQUAL);
  bluetoothSerial.print(GRADIENT_VAL);
  bluetoothSerial.print(SEPARATOR);
  //End sending information.
  bluetoothSerial.print(END);
  // Serial.println("done");
}

// Store received values to variables for controlling the shifting.
void storeToValue() {
  if (key.equals(TAR_FREQUENCY)) {
      TAR_FREQUENCY_VAL = value;
  } else if (key.equals(HILL_DIV)) {
      HILL_DIV_VAL = value;
  } else if (key.equals(GENERAL_DIV)) {
      GENERAL_DIV_VAL = value;
  } else if (key.equals(RESET)) {
      RESET_VAL = value;
  } else if (key.equals(CIRCUMFERENCE)) {
      CIRCUMFERENCE_VAL = value;
      if (CIRCUMFERENCE_VAL.equals("20")) {
        rad = circumference[0];
      } else if (CIRCUMFERENCE_VAL.equals("24")) {
        rad = circumference[1];
      } else if (CIRCUMFERENCE_VAL.equals("26")) {
        rad = circumference[2];
      } else if (CIRCUMFERENCE_VAL.equals("28")) {
        rad = circumference[3];
      }
  }
  if (RESET_VAL.equals("1")) {
    // Reset the bike. Shift front to first and rear to first.
    resetGears();
    RESET_VAL = "0";
  }
}

// Handling the receiving information.
void receiveInformation() {
  while (bluetoothSerial.available()) {
    char inChar = (char)bluetoothSerial.read();
    // bluetoothSerial.print(inChar);
    Serial.print(inChar);
      if (inChar != '=' && inChar != '!') {
        inputString += inChar;
      } else if (inChar == '=') {
        key = inputString;
        //Serial.print("key = ");
        //Serial.println(key);
        inputString = "";
      } else if (inChar == '!') {
        value = inputString;
        inputString = "";
        storeToValue();
        // bluetoothSerial.println("");
        // Serial.println("");
        // stringComplete = true;
        break;
      }
  }
}




  



