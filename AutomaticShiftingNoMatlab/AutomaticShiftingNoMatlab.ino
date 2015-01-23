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
    
    Connection from arduino to relay
    (http://www.amazon.de/Kanal-Relais-Module-Arduino-TTL-Logik/dp/B00ALNJN72/ref=sr_1_7?ie=UTF8&qid=1422019451&sr=8-7&keywords=relais):
    VCC -- 5V
    IN4 -- Pin 8  (rear up)
    IN3 -- Pin 12 (rear down)
    IN2 -- Pin 4  (front up)
    IN1 -- Pin 7  (front down)
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
*/

//***********************************  BLUETOOTH SETTINGS ****************************************
#include <SoftwareSerial.h>  // library for using SoftwareSerial

// Defining keys for bluetooth communication.
const String 
// Sending information.
  VELOCITY = "VELOCITY",
  CURR_FREQUENCY = "CURR_FREQUENCY",
  FRONT_GEAR = "FRONT_GEAR",
  REAR_GEAR = "REAR_GEAR",
  GRADIENT = "GRADIENT",
// Receiving information.
  TAR_FREQUENCY = "TAR_FREQUENCY",
  HILL_DIV = "HILL_DIV",
  GENERAL_DIV = "GENERAL_DIV",
  RESET = "RESET",
  CIRCUMFERENCE = "CIRCUMFERENCE";

// Defining values for bluetooth communication.
String
// Sending information.
  VELOCITY_VAL = "",
  CURR_FREQUENCY_VAL = "",
  FRONT_GEAR_VAL = "",
  REAR_GEAR_VAL = "",
  GRADIENT_VAL = "0",
// Receiving information.
  TAR_FREQUENCY_VAL = "",
  HILL_DIV_VAL = "",
  GENERAL_DIV_VAL = "",
  RESET_VAL = "",
  CIRCUMFERENCE_VAL = "";

// Separator.
const String SEPARATOR = "!";
// Equalsign.
const String EQUAL = "=";

// Begin of information
const String BEGIN = "#";
// End of information
const String END = "$";

// defining pins for SoftwareSerial:
const int
  bluetoothTX = 10,
  bluetoothRX = 11;
  
// Create new SoftwareSerial for bluetooth communication.
SoftwareSerial
  bluetoothSerial = SoftwareSerial(bluetoothRX, bluetoothTX);

// Defining inputString, key and value.
String
  inputString = "",    // inputString for reading from bluetooth
  key = "",            // key for storing incoming key String
  value = "";          // key for storing incoming value String

// Define stringComplete flag for knowing the last character.
boolean stringComplete = false;

// Send the sending information every x seconds.
const int every_X_Seconds = 1;

//************************************************************************************************


//************************************* MEASURING SETTINGS ***************************************
// Using Pin2 of digital pins for input of reed sensor at wheel.
const int inputPinS1 = 2;
// Using Pin13 of digital pins for input of reed sensor at crank.
const int crankRpmInputPin = 3;

long
  timePreviousHigh,            // time of last peak of wheel sensor [ms]
  timeActualHigh,              // time of current peak of wheel sensor [ms]
  difference,                  // time difference between last and current peak wheel sensor [ms]
  crankTimePreviousHigh,       // time of last peak of crank sensor [ms]
  crankTimeActualHigh,         // time of current peak of crank sensor [ms]
  crankDifference;             // time difference between last current peak crank sensor [ms]

int
  currentFront,                // current front chainblade
  currentRear;                 // current rear chainblade
  
float
  currentSpeed,                // current speed [km/h]
  currentRoundsWheel,          // current rounds of wheel [U/min]
  frequency = 50.0f,           // computed crank frequency [U/min]
  crankFrequency;              // measured crank frequency [U/min]

// the gears if correction happens to bee needed
int
  currentFrontForCorrection,   // front gear, which should be current gear for correction
  currentRearForCorrection,    // rear gear, which should be current gear for correction
  correctionNeededCounter = 0; // counter for how often is measured that correction is needed

// WheelDebounceTime:
// If we expecting a maximum speed of 70 km/h = 19,4 m/s, we get 19,4 m / 2,035 m = 9,5
// interrupts per second. So the interrupt occures all 104ms.
int wheelDebouncingTime = 104;

// crankDebouncingTime:
// We assume, that there is nobody who makes more than 120 RPM at the crank so our
// crankDebouncingTime needs to 500 ms, for getting max 2 interrupts per second by the crank
// sensor.
int crankDebouncingTime = 500;
//*************************************************************************************************

// ****************************** SHIFTING SETTINGS **********************************
// Using Pins 4,7,8,12 as Outputpins to wire up the Relais
const int
  frontUpPin = 4,    // pin 4 for shifting front up
  frontDownPin = 7,  // pin 7 for shifting front down
  rearUpPin = 8,     // pin 8 for shifting rear up
  rearDownPin = 12;  // pin 12 for shifting rear down

const float
  frontGear[] = {34, 50},
  rearGear[] = {25, 23, 21, 19, 17, 16, 15, 14, 13, 12};

// Circumference in mm for 20", 24", 26", 28"
const int
  circumference[] = {1530, 1860, 1940, 2110};
int rad = 2110;    // wheel circumstance in mm 28"
// ***********************************************************************************

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
  TAR_FREQUENCY_VAL = "50";
  GENERAL_DIV_VAL = "10";
  RESET_VAL = "0";
  
  // Reserve space for inputString, key and value.
  inputString.reserve(50);
  key.reserve(50);
  value.reserve(50);
  
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
  
  // The idea is that if we are in a wrong gear the error can only be a 
  // discrete value because of the gears fix number of teeth.
  // If we are not turning the crank at the maximum possible that error will 
  // change. If it does not change we should do a correction.
  // We assume that we are not more than 1 Gear away from the real one. 
  // If so we should better implement a reset function.
  // It only checks if the crankSensor has a signal with about 60RPM that 
  // means that 3 singals are 3 seconds. -> 3rd "sucessful" check will do the
  // correction.
  if (frequency < crankFrequency * 0.96f || frequency > crankFrequency * 1.04f) {
    checkGear();
  }
}

void triggerSensor1() {
  timeActualHigh = millis();
  difference = timeActualHigh - timePreviousHigh;
  
  // Only compute new speed, if difference is bigger than wheelDebouncingTime.
  // If not, wait for next occuring interrupt.
  if (difference >= wheelDebouncingTime) {
    // compute speed with km/h = (mm / ms) * 3.6
    currentSpeed = ((float)rad / (float)difference) * 3.6f;
    // compute rounds of wheel U / min = 60000 / difference[ms].
    currentRoundsWheel = 60000 / (float)difference;
    // compute frequency of crank.  
    frequency = currentRoundsWheel / (frontGear[currentFront - 1] / rearGear[currentRear - 1]);
    timePreviousHigh = millis();
  }
}

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

// define interrupt service routine for timer declared above.
ISR(TIMER1_COMPA_vect){
  sendInformation();
}

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

void receiveInformation() {
  while (bluetoothSerial.available()) {
    char inChar = (char)bluetoothSerial.read();
    // bluetoothSerial.print(inChar);
    Serial.print(inChar);
    if (inChar != '=' && inChar != '!') {
      inputString += inChar;
    } else if (inChar == '=') {
      key = inputString;
      inputString = "";
    } else if (inChar == '!') {
      value = inputString;
      inputString = "";
      storeToValue();
      bluetoothSerial.println("");
      Serial.println("");
      stringComplete = true;
      break;
    }
  }
}

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
      } else if (CIRCUMFERENCE_VAL.equals("20")) {
        rad = circumference[3];
      }
  }
  if (RESET_VAL.equals("1")) {
    // Reset the bike. Shift front to first and rear to first.
    resetGears();
    RESET_VAL = "0";
  }
}

void resetGears() {
  generateSignal(1,frontDownPin);
  generateSignal(10,rearDownPin);

  currentFront = 1;
  currentRear = 1;
}
  
  
  
void gearUp(){
  // small front chainblade.
  if (currentFront == 1){
    if (currentRear < 8){
      // generate Signal to shift up rear by one gear.
      generateSignal(1, rearUpPin);
      currentRear++;
      return;
    } 
    // rear has reached 8, so we have to shift to bigger front chainblade.
    else {
      // generate Signal to shift up Front by one gear and shift down rear by 4 gears.
      generateSignal(1, frontUpPin);
      generateSignal(4, rearDownPin);
      currentRear = 4;
      currentFront = 2;
      return;
    }
  }
  // big front chainblade
  else {
    // already in biggest gear.
    if (currentRear == 10){
      return;
    }
    // not yet in biggest gear. Shift up.
    else {
      // generate Signal to shift up rear by one gear.
      generateSignal(1, rearUpPin);
      currentRear++;
      return;      
    }
  }  
}


void gearDown(){
  // small front chainblade.
  if (currentFront == 1){
    // already smallest gear. Do nothing.
    if (currentRear == 1){
      return;
    } 
    // singel rear downshift.
    else {
      // generate signal to shift down rear by 1 gear.
      generateSignal(1, rearDownPin);
      currentRear--;
      return;
    }
  }
  // big front chainblade
  else {
    if (currentRear > 3){
      // generate Signal to shift down rear by one gear.
      generateSignal(1, rearDownPin);
      currentRear--;
      return;
    } 
    // rear has reached 3, so we have to shift to smaller front chainblade.
    else {
      // generate Signal to shift down Front by one gear and shift up rear by 4 gears.
      generateSignal(1, frontDownPin);
      generateSignal(4, rearUpPin);
      currentRear = 7;
      currentFront = 1;
      return;
    }
  }  
}

/*
  Generates <count> positive signals on by <mode> specified pin.
  Delay time between two positive signals is for the mechanical shifting time.
*/
void generateSignal(int count, int mode) {
  for ( int i = 0; i < count; i++) {
    digitalWrite(mode, LOW);
    delay(200);
    digitalWrite(mode, HIGH);
    delay(400);
  }
}

void checkGear(){
  int oneMatch = 0;
  float tmp = 0;  
  if (currentRear != 1) {
    // check difference to crankFrequency if rear would be shifted down by one gear
    tmp = currentRoundsWheel / (frontGear[currentFront - 1] / rearGear[currentRear - 2]);
    if (tmp > crankFrequency * 0.98f && tmp < crankFrequency * 1.02f) {
      if (currentFrontForCorrection == currentFront && currentRearForCorrection == (currentRear - 1)) {
        correctionNeededCounter++;
      } else {
        correctionNeededCounter = 1;
        currentFrontForCorrection = currentFront;
        currentRearForCorrection = currentRear - 1;
      }
      oneMatch = 1;
    }
  }
  
  if (currentRear != 10) {
    // check difference to crankFrequency if rear would be shifted up by one gear
    tmp = currentRoundsWheel / (frontGear[currentFront - 1] / rearGear[currentRear]);
    if (tmp > crankFrequency * 0.98f && tmp < crankFrequency * 1.02f) {
      if (currentFrontForCorrection == currentFront && currentRearForCorrection == (currentRear + 1)) {
        correctionNeededCounter++;
      } else {
        correctionNeededCounter = 1;
        currentFrontForCorrection = currentFront;
        currentRearForCorrection = currentRear + 1;
      }
      oneMatch = 1;
    }
  }
    
  if (currentFront == 1) {
    // check difference to crankFrequency if front would be shifted up by one gear
    tmp = currentRoundsWheel / (frontGear[currentFront] / rearGear[currentRear - 1]);
    if (tmp > crankFrequency * 0.98f && tmp < crankFrequency * 1.02f) {
      if (currentFrontForCorrection == (currentFront + 1) && currentRearForCorrection == currentRear) {
        correctionNeededCounter++;
      } else {
        correctionNeededCounter = 1;
        currentFrontForCorrection = currentFront + 1;
        currentRearForCorrection = currentRear;
      }
      oneMatch = 1;
    }
  } else {
    // check difference to crankFrequency if front would be shifted down by one gear
    tmp = currentRoundsWheel / (frontGear[currentFront - 2] / rearGear[currentRear - 1]);
    if (tmp > crankFrequency * 0.98f && tmp < crankFrequency * 1.02f) {
      if (currentFrontForCorrection == (currentFront - 1) && currentRearForCorrection == currentRear) {
        correctionNeededCounter++;
      } else {
        correctionNeededCounter = 1;
        currentFrontForCorrection = currentFront - 1;
        currentRearForCorrection = currentRear;
      }
      oneMatch = 1;
    }
  }
  
  // the error was not one of the discrete values
  if (!oneMatch) {
    correctionNeededCounter = 0;
  }
  
  // if we have encountered the error often enough and it was allways the same we need a gear shift
  if (correctionNeededCounter == 3) {
    if (currentFrontForCorrection > currentFront) {
      // shift Front up by one.
      generateSignal(1, frontUpPin);
      currentFront++;
      Serial.println("gear UP correction");
    }
    if (currentFrontForCorrection < currentFront) {
      // shift Front down by one.
      generateSignal(1, frontDownPin);
      currentFront--;
      Serial.println("gear DOWN correction");
    }
    if (currentRearForCorrection > currentRear) {
      // shift Rear up by one.
      generateSignal(1, rearUpPin);
      currentRear++;
      Serial.println("gear UP correction");
    }
    if (currentRearForCorrection < currentRear) {
      // shift Rear down by one.
      generateSignal(1, rearDownPin);
      currentRear--;
      Serial.println("gear DOWN correction");
    }
  }
  correctionNeededCounter = 0;
}

