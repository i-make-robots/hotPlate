/**
 * Shirt iron Hot plate logic
 */
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define RELAY_PIN           9  // digital 9
#define THERMISTOR_PWR      6  // digital 6
#define THERMISTOR_SENSOR   0  // analog 0

#define BAUD                57600

#define TEMP_R1             100000  // 100k thermistor
// thermistor coefficient
#define TEMP_BCOEFFICIENT   4100//4267//4092//3950
// resistance at nominal temperature
#define THERMISTORNOMINAL   100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL  25   

double KP = 0.1;
double KI = 0.0;
double KD = 0.01;
double minimumThreshold=0.725;

double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
byte ATuneModeRemember=2;

double target=40;
double input, output;
boolean tuning=false;

PID myPID(&input, &output, &target, KP,KI,KD, DIRECT);
PID_ATune aTune(&input, &output);

int waitTime = 50;


void setup() {
  Serial.begin(BAUD);
  
  // send power to the thermistor circuit
  pinMode(THERMISTOR_PWR,OUTPUT);
  digitalWrite(THERMISTOR_PWR,HIGH);

  // prepare to send to the relay
  pinMode(RELAY_PIN,OUTPUT);
  
  myPID.SetMode(AUTOMATIC);
  if(tuning) {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
}


void loop() {
  if(Serial.available() > 0) {
    char c = Serial.read();
    switch(c) {
      case 'p': case 'P': KP=Serial.parseFloat(); break;
      case 'i': case 'I': KI=Serial.parseFloat(); break;
      case 'd': case 'D': KD=Serial.parseFloat(); break;
      case 't': case 'T': {
          double newTarget = Serial.parseFloat();
          if( !isnan(newTarget) && newTarget>0 && newTarget<255 ) {
            target=newTarget;
          }
        }
        break;
      case 'm': case 'M': {
          double newTarget = Serial.parseFloat();
          if( !isnan(newTarget) && newTarget>0 && newTarget<255 ) {
            minimumThreshold=newTarget;
          }
        }
        break;
      default: {
          while(Serial.available()>0) Serial.read();  
        }
        break;
    }
  }
  
  float sensorReading = analogRead(THERMISTOR_SENSOR);
  input = SteinhartHartEquation(sensorReading);

  if(tuning==true) {
    byte val = aTune.Runtime();
    if(val!=0) {
      Serial.println("\n\nAUTOTUNE FINISHED\n");
      tuning=false;
      // set the tuning parameters
      KP = aTune.GetKp();
      KI = aTune.GetKi();
      KD = aTune.GetKd();
      myPID.SetTunings(KP,KI,KD);
      AutoTuneHelper(false);
    }
  } else {
    myPID.Compute();
  }
  
  digitalWrite(RELAY_PIN,output>minimumThreshold ? HIGH: LOW);
  
  Serial.print(target);            Serial.print("\t");
  Serial.print(input);             Serial.print("\t");
  Serial.print(output);            Serial.print("\t");
  Serial.print(KP);                Serial.print("\t");
  Serial.print(KI);                Serial.print("\t");
  Serial.print(KD);                Serial.print("\t");
  Serial.print(minimumThreshold);  Serial.print("\t");
  Serial.print(tuning?'*':' ');    Serial.println();
  
  delay(waitTime);
}


float SteinhartHartEquation(float arg0) {
  float a2 = TEMP_R1 * (1023.0 / arg0 - 1.0);
  
  float steinhart;
  steinhart = a2 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= TEMP_BCOEFFICIENT;              // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  return steinhart - 273.15;                   // convert to C
}


void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}
