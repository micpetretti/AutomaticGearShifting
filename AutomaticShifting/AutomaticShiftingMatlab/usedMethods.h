
/*
Authors:  Michael Petretti (michael@petretti.de)
          Björn Hagemeister
          @ University of Freiburg, Germany, 2015.
*/

//*************************************************************************************************
//***********************************    DECLARATIONS     *****************************************
//*************************************************************************************************

//***********************************  BLUETOOTH SETTINGS *****************************************
#ifndef ARDUINO
#else
#include <Arduino.h>
#endif

#include <stdlib.h>

#ifndef ARDUINO
void printSomething() {
	printf("Hallo\n\r");
}
#else
#endif

#ifndef ARDUINO// Variables, neccessary for Matlab simulation.
double* u; // input
double* y; // output
double* x; // states
#else
double u[6];
double y[6];
double x[6];
#endif

// defining pins for SoftwareSerial:
const int
  bluetoothTX = 10,
  bluetoothRX = 11;
  



// Send the sending information every x seconds.
const int every_X_Seconds = 1;

//*************************************************************************************************


//************************************* MEASURING SETTINGS ****************************************
// Using Pin2 of digital pins for input of reed sensor at wheel.
const int inputPinS1 = 2;
// Using Pin13 of digital pins for input of reed sensor at crank.
const int crankRpmInputPin = 3;

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

// crankDebouncingTime:
// We assume, that there is nobody who makes more than 120 RPM at the crank so our
// crankDebouncingTime needs to 500 ms, for getting max 2 interrupts per second by the crank
// sensor.
int crankDebouncingTime = 500;

long
  difference,                  // time difference between last and current peak wheel sensor [ms]
  crankDifference;             // time difference between last current peak crank sensor [ms]

//*************************************************************************************************

// ****************************** SHIFTING SETTINGS ***********************************************
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
// ************************************************************************************************
// *********************************** END DECLARATIONS *******************************************
// ************************************************************************************************



// ************************************************************************************************
// **************************************  METHODS  ***********************************************
// ************************************************************************************************

// *********************************************  GENERATE SIGNAL *********************************
#ifndef ARDUINO
#else
// Generates <count> positive signals on by <mode> specified pin.
// Delay time between two positive signals is for the mechanical shifting time.
void generateSignal(int count, int mode) {
  for ( int i = 0; i < count; i++) {
    digitalWrite(mode, LOW);
    delay(200);
    digitalWrite(mode, HIGH);
    delay(400);
  }
}
#endif

// *********************************************  CHECK GEAR  ****************************************
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
      currentFront++;
      #ifdef ADRUINO
      generateSignal(1, frontUpPin);
      Serial.println("gear UP correction");
      #endif

    }
    if (currentFrontForCorrection < currentFront) {
      // shift Front down by one.
      currentFront--;
      #ifdef ADRUINO
      generateSignal(1, frontDownPin);
      Serial.println("gear DOWN correction");
      #endif

    }
    if (currentRearForCorrection > currentRear) {
      // shift Rear up by one.
      currentRear++;
      #ifdef ADRUINO
      generateSignal(1, rearUpPin);
      Serial.println("gear UP correction");
      #endif

    }
    if (currentRearForCorrection < currentRear) {
      // shift Rear down by one.
      currentRear--;
      #ifdef ADRUINO
      generateSignal(1, rearDownPin);
      Serial.println("gear DOWN correction");
      #endif

    }
  }
  correctionNeededCounter = 0;
}


//****************************************  GEAR UP  *********************************
// Method for shifting upwards.
void gearUp(){
  // small front chainblade.
  if (currentFront == 1){
    if (currentRear < 8){
      // generate Signal to shift up rear by one gear.
      #ifdef ARDUINO
      generateSignal(1, rearUpPin);
      #endif
      currentRear++;
      return;
    } 
    // rear has reached 8, so we have to shift to bigger front chainblade.
    else {
      // generate Signal to shift up Front by one gear and shift down rear by 4 gears.
      #ifdef ARDUINO
      generateSignal(1, frontUpPin);
      generateSignal(4, rearDownPin);
      #endif
      currentFront = 2;
      currentRear = 4;
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
      #ifdef ARDUINO
      generateSignal(1, rearUpPin);
      #endif
      currentRear++;
      return;      
    }
  }  
}

// *****************************************  GEAR DOWN  **********************************
// Method for shifting downwards.
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
      #ifdef ARDUINO
      generateSignal(1, rearDownPin);
      #endif
      currentRear--;
      return;
    }
  }
  // big front chainblade
  else {
    if (currentRear > 3){
      // generate Signal to shift down rear by one gear.
      #ifdef ARDUINO
      generateSignal(1, rearDownPin);
      #endif
      currentRear--;
      return;
    } 
    // rear has reached 3, so we have to shift to smaller front chainblade.
    else {
      // generate Signal to shift down Front by one gear and shift up rear by 4 gears.
      #ifdef ARDUINO
      generateSignal(1, frontDownPin);
      generateSignal(4, rearUpPin);
      #endif
      currentFront = 1;
      currentRear = 7;
      return;
    }
  }  
}


// ******************************  RESET GEAR  ***************************************************
#ifndef ARDUINO
// Reset gears means shifting to 1, 1.
// Method called after receiving reset - Signal by smartphone.
void resetGears() {
  currentFront = 1;
  currentRear = 1;
  
  y[0] = 1; // currentFront
  y[1] = 1; // currentRear
}
#else
// Reset gears means shifting to 1, 1.
// Method called after receiving reset - Signal by smartphone.
void resetGears() {
  generateSignal(1,frontDownPin);
  generateSignal(10,rearDownPin);

  currentFront = 1;
  currentRear = 1;
}
#endif

// *************************  COMPUTE SPEED AND CRANK FREQUENCY  *****************************************
void computeSpeedAndFrequency() {
  // compute speed with km/h = (mm / ms) * 3.6
  currentSpeed = ((float)rad / (float)difference) * 3.6f;
  // compute rounds of wheel U / min = 60000 / difference[ms].
  currentRoundsWheel = 60000 / (float)difference;
  // compute frequency of crank.  
  frequency = currentRoundsWheel / (frontGear[currentFront - 1] / rearGear[currentRear - 1]);
}


// ********************************************  MAIN LOOP  ***************************************************
void mainIt() {
  #ifndef ARDUINO
  // Read input from simulation;
  if (u[1] == 20) {
    rad = circumference[0];
  } else if (u[1] == 24) {
    rad = circumference[1];
  } else if (u[1] == 26) {
    rad = circumference[2];
  } else if (u[1] == 28) {
    rad = circumference[3];
  }
  
  difference = u[3];
  // crankDifference = u[4];
  
  // target frequency - general Div.
  int tarFreqMinusDiv = u[0] - ((u[0] / 100.0f) * u[2]);
  // target frequency + general Div.
  int tarFreqPlusDiv = u[0] + ((u[0] / 100.0f) * u[2]);
  
  computeSpeedAndFrequency();
  crankFrequency = frequency;
  // crankFrequency = 60000 / (float) crankDifference;
  y[0] = currentFront;
  y[1] = currentRear;
  y[2] = currentSpeed;
  y[3] = frequency;
  y[4] = frequency;
  
  // Adapt to changes in current frequency.
  if (frequency < tarFreqMinusDiv) {
    gearDown();
  }
  if (frequency > tarFreqPlusDiv) {
    gearUp();
  }
  #endif
  
  
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
  
  #ifndef ARDUINO
  if (u[5] == 1) {
    resetGears();
  }
  #endif
}

void initMethod() {
  currentFront = 1;
  currentRear = 1;
}
