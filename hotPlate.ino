/**
 * Shirt iron Hot plate logic
 */

#define RELAY_PIN           9  // digital 9
#define THERMISTOR_PWR      6  // digital 6
#define THERMISTOR_SENSOR   0  // analog 0

#define BAUD                57600

#define TEMP_R1             100000  // 100k thermistor
// Steinhart-Hart coefficients
float P = 16.13;
float I = 1.16;
float D = 56.23;
  
#define TEMP_BCOEFFICIENT   4066//3950//4267
// resistance at 25 degrees C
#define THERMISTORNOMINAL   100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL  25   

float target=21;

float integral = 0;
float previousError = 0;

int   waitTime = 100;


void setup() {
  Serial.begin(BAUD);
  
  // send power to the thermistor circuit
  pinMode(THERMISTOR_PWR,OUTPUT);
  digitalWrite(THERMISTOR_PWR,HIGH);

  // prepare to send to the relay
  pinMode(RELAY_PIN,OUTPUT);
}


void loop() {
  if(Serial.available() > 0) {
    float newTarget = Serial.parseFloat();
    if( !isnan(newTarget) && newTarget>0 ) {
      target=newTarget;
      integral=0;
    }
  }
  
  float sensorReading = analogRead(THERMISTOR_SENSOR);
  float celcius = SteinhartHartEquation(sensorReading);

  float iterationTime = waitTime/1000.0;
  
  float error = target - celcius;
  integral += error * iterationTime; 
  float derivative = (error - previousError)/iterationTime;
  previousError = error;

  // @see https://reprap.org/wiki/PID_Tuning
  //Prusa 3 response to M301 says p:16.13 i:1.16 d:56.23 c:1.00
  float output = P*error + I*integral + D*derivative;
  
  digitalWrite(RELAY_PIN,output>0 ? HIGH: LOW);
  
  Serial.print(target);
  Serial.print("\t");
  Serial.println(celcius);
  
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
