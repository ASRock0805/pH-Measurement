/*
  pH Measurement by Dual Slope

      The output voltage of the sensor is converted by ADC to obtain the ph value.
  pH 7 - pH 10
  pH 7 - pH 4

  Created 3 Dec. 2019
  by Yi-Xuan Wang

  References:
  https://en.wikipedia.org/wiki/PH_meter
*/

/*--- Preprocessor ---*/
#define phPin A0 // pH meter ADC pin

#define N 800    // Measurment sampling number for smoothing

/*--- Constants ---*/
const unsigned long baudSpeed = 115200;               // Sets the data rate in bits per second (baud) for serial data transmission
const unsigned long period = 1000;                    // The value is a number of milliseconds

const byte vIn = 5;                                   // Supply voltage from Arduino
const byte resBits = 10;                              // Resolution of ADC (10 bits)
const float vConv = vIn / (pow(2.0, resBits) - 1.0);  // Voltage of ADC level (2^bits)

const float nrPh = 7.0;             // Neutral pH
const float offSet = 2.535;         // Reference output voltage of pH 7
const float phStep4  = 0.178666667; // pH 7: 2.535 V, pH 4: 3.071 V, (3.071 - 2.535) / 3
const float phStep10 = 0.156333333; // pH 7: 2.535 V, pH 10: 2.066 V, (2.535 - 2.066) / 3
/*--- Global Variables ---*/
unsigned long startTime;    // Start time
unsigned long currentTime;  // Current time

float vOut;                 // Output of the ADC
float phValue;              // Value of pH

/*--- Function Prototype ---*/
float phCal(float );
void phMeas(byte );
void setup(void);
void loop(void);

/*--- Functions Definition ---*/
// Implementation of pH Calculation
float phCal(float volt) {
  if (volt > offSet) {   // Acidic
      phValue = nrPh + ((offSet - volt) / phStep4);
  } if (volt < offSet) { // Basic
      phValue = nrPh + ((offSet - volt) / phStep10);
  } else { // Neutral
      phValue = nrPh;
  }

  return phValue;
}

// pH Sensor
void phMeas(byte signalPin) {  // Read analog signals, and converts into digtal signals
  for (unsigned int i = 0; i < N; ++i) {    // Get samples for smooth the value
    vOut = vOut + analogRead(signalPin);
    delay(1);                               // delay in between reads for stability
  }
  vOut = (vOut * vConv) / N;                // ADC of voltage meter output voltage

  /*--- Convert the output voltage into pH value ---*/
  phValue = phCal(vOut);                    // Calculate pH

  if (isinf(phValue) || isnan(phValue)) {
    phValue = -1;
  }
}

/*--- Initialization ---*/
void setup(void) {
  Serial.begin(baudSpeed);  // Initializes serial port
  pinMode(phPin, INPUT);    // Initializes potentiometer pin
  startTime = millis();     // Initial start time

  // pH Sensor Initialization
  vOut = 0.0;
  phValue = 0.0;
}

/*--- Measurement ---*/
void loop(void) {
  // Every second, calculate and print the measured value
  currentTime = millis();                     // Get the current "time"

  if ((currentTime - startTime) >= period) {  // Test whether the period has elapsed
    // pH Sensor
    phMeas(phPin);
  
    /*--- Sensor prompt ---*/
    Serial.print("Voltage: ");
    Serial.println(vOut, 4);
    Serial.print("pH: ");
    Serial.println(phValue, 2);

    /*--- System Return ---*/
    startTime = currentTime;  // Save the start time of the current state
  } else {
    return;
  }
}
