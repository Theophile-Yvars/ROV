#include "OneWire.h"
#include "DallasTemperature.h"

// =============================================================== Commandes

int potentiometre;

// =============================================================== PWM

//Ports de commande du moteur B // https://arduino.blaisepascal.fr/pont-en-h-l298n/
int motorDroitPin1 = 8;
int motorDroitPin2 = 9;
int enableMotorDroitPin = 5;
int value;

// =============================================================== Temperature

int temp;
int temperature_celcius;
OneWire oneWire(A1);
DallasTemperature ds(&oneWire);

// =============================================================== Voltage

// Define analog input
#define ANALOG_IN_PIN A2
// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0;  
// Float for Reference Voltage
float ref_voltage = 5.0;
// Integer for ADC value
int adc_value = 0;
float voltage_value;

// =============================================================== Programme

void setup() {
  Serial.println("SETUP ....");
  Serial.begin(9600);
  pinMode(motorDroitPin1, OUTPUT);
  pinMode(motorDroitPin2, OUTPUT);
  pinMode(enableMotorDroitPin, OUTPUT);
}

void loop() {
  delay(1000); // attend une seconde
  PWM();
  temperature_celcius = temperature();
  voltage_value = voltage();
}


void PWM(){ // https://arduino.blaisepascal.fr/pont-en-h-l298n/
  digitalWrite(motorDroitPin1, HIGH); 
  digitalWrite(motorDroitPin2, LOW);

  //sets the value (range from 0 to 255):
  value = 0;
  potentiometre = analogRead(A0);
  value = map(potentiometre, 0, 1023, 0, 255);
  //Serial.println(value);

  analogWrite(enableMotorDroitPin, value);
}

int temperature(){ // https://arduino-france.site/ds18b20-arduino/
  ds.requestTemperatures();
  temp = ds.getTempCByIndex(0);

  //Serial.print("Temperature : ");
  //Serial.print(temp);
  //Serial.println(" Celcius");
  //Serial.println("-----------------------------");

  return temp;
}

float voltage(){ // https://lastminuteengineers.com/voltage-sensor-arduino-tutorial/#google_vignette
  // Read the Analog Input
  adc_value = analogRead(ANALOG_IN_PIN);
  // Determine voltage at ADC input
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;
  // Calculate voltage at divider input
  in_voltage = adc_voltage*(R1+R2)/R2;
  // Print results to Serial Monitor to 2 decimal places
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);

  return in_voltage;
}
