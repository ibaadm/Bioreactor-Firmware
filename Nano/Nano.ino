#include <ArduinoJson.h>
#include <HardwareSerial.h>

#define Motor_input_pin D6
#define Motor_output_pin D10
#define heatInputPin A1
#define heatOutputPin D8

String devkitc_input_buffer = "";
bool devkitc_input_complete = false;

/*==========================================================MOTOR VARIABLES=================================================*/
double RPM = 0;
double prev_RPM = 0; /* previous RPM for smoothing*/
double* current_RPMs = new double[10];
int RPM_index = 0;
double RPM_mean = NAN;
int pulse_count;
unsigned long motor_time; /* time measuremetns for motor calculation*/
unsigned long motor_prev_time = 0;
int motor_interval = 100000; /*Interval in microseconds beteeen readings of RPM*/
double revs;
double revs_per_sec;
float motor_setpoint = 60;

/* Control values*/
float motor_error;
float prev_motor_error;

float motor_prop;
float motor_prop_coeff = 1;

float motor_integ = 0;
float motor_integ_coeff = 0.001;

float motor_output = 1;
int motor_state = 0;
/* PWN variables*/
int motor_pwm_interval = 10; /* microseconds*/
int motor_count = 0;
unsigned long motor_pwm_time;
unsigned long motor_pwm_time_prev;

bool motor_killed = false;

/*==============================================HEATING===========================================================*/
/* temperature calcualation stuff*/
float R2 = 10000;      /*Reistance of resistor in series*/
float RT;              /*Thermistor resistance*/
float RT_base = 10000; /* Thermistor base resistancw (25C)*/
float b = 4220;        /*beta constant*/
float T0 = 25;         /* base temperature for thermistor equation*/
double temperature;
double* current_temperatures = new double[10];
int temperature_index = 0;
double temperature_mean = NAN;
float Vin_max = 3.3;

float Vout;
float heat_deadband = 0;
/* measurement timing*/
unsigned long heat_time = 0; /* In milliseconds*/
unsigned long heat_time_prev = 0;
int heat_interval = 100;

/* PWN VALUES*/
int heat_state_pwm = 0;

unsigned long heat_pwm_time;
int heat_pwm_interval = 100;
unsigned long heat_pwm_time_prev;
float heat_set_point = 33.0;
float heat_error;
float deadband = 0;
int heat_resolution = 4095;
float heat_transfer = Vin_max / heat_resolution;
int heat_output = 0;

bool heating_killed = false;

/*==========================================PH VARIABLES================================================================*/
const float KpH = 5 / 4095;  // 12 bit resolution for ESP 32 hense 4095( from 2^12 -1 ) the calibration values are 2.1046, 1.6755
//Also need to put in pH7 and adjust trimmer potentionmeter so when is pH7 gives about 5/2V
//KpH var is what convert ADC reading to pH voltage
const byte pHPin = A0, AcidPin = D3, BasePin = D2;  // indicates what pin pH sensor connected to on arduino change if necessary and Base/Acid pumps-> D4 and D6 I'm not sure if there is a specfic way round for them
int Timems, T2, LastTimeAcidOn, LastTimeBaseOn;
float pH, pHSmoothed, AimpH = 7.0, DeltapH = 0.1, Bs;  //pH gives the raw pH recorded by pH probe(after interpretation in later line) whilst pHSmoothed uses IIR to get more stable value for pH(will explain later)
//AimpH variable determines the target pH and DeltapH is the deviation from target before acid/base is added-> they are all placeholders rn do whatever later
bool AcidOn = 0, BaseOn = 0;  //variables to keep track of if pump is on or not
unsigned long pH_time = 0;
unsigned long pH_time_prev = 0;
int pH_invertal = 100; /* interval in milliaseconds*/

double* current_pHs = new double[10];
int pH_index = 0;
double pH_mean = NAN;

bool pumps_killed = false;


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, D0, D1);
  delay(1000);

  /*===========================Motor setup=======================================*/
  pinMode(Motor_input_pin, INPUT);
  pinMode(Motor_output_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Motor_input_pin), pulse_counter, RISING); /* setups interrupt routine for the hall effect sensor*/
                                                                                  /*=====================================Heating setup======================================*/
  analogReadResolution(12);                                                       /* analog read resolution is  4095 max value */
  pinMode(heatInputPin, INPUT);
  pinMode(heatOutputPin, OUTPUT);
  /*========================================PH SETUP===========================*/
  pinMode(pHPin, INPUT);     //sets the pin mentioned above as an input pin
  pinMode(AcidPin, OUTPUT);  //sets the pin for acid as output pin
  pinMode(BasePin, OUTPUT);  //sets the pin for base as output pin
  digitalWrite(AcidPin, 0);  //sets pump for acid to off
  digitalWrite(BasePin, 0);  //sets pump for base to off
  //I don't know how interfacing with pumps work could be low/high or bool I'm not sure
  //Digitalwrite function is demanding/slow but it's fine as it is being run only once in the setup
}

void readDevKitCData() {
  while (Serial1.available() > 0) {
    char inChar = (char)Serial1.read();
    if (inChar == '\n') {
      devkitc_input_complete = true;
    } else {
      devkitc_input_buffer += inChar;
    }
  }

  if (devkitc_input_complete) {

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, devkitc_input_buffer);

    if (error) {
      Serial.print(F("JSON Parsing Failed: "));
      Serial.println(error.f_str());
      return;
    }

    if (doc.containsKey("ph")) {
      //set ph
      AimpH = doc["ph"];
      heat_set_point = doc["temperature"];
      motor_setpoint = doc["rpm"];
      Serial.printf("Updated setpoints\npH: %.2f | Temp: %.2f | RPM: %d\n",
                    (float)doc["ph"],
                    (float)doc["temperature"],
                    (int)doc["rpm"]);
    } else {
      pumps_killed = doc["pumps_killed"];
      heating_killed = doc["heating_killed"];
      motor_killed = doc["motor_killed"];
      Serial.printf("Updated kill states\npH: %s | Temp: %s | RPM: %s",
                    doc["pumps_killed"] ? "true" : "false",
                    doc["heating_killed"] ? "true" : "false",
                    doc["motor_killed"] ? "true" : "false");
    }

    devkitc_input_buffer = "";
    devkitc_input_complete = false;
  }
}

void calculatepHMean(double current) {
  current_pHs[pH_index] = current;
  pH_index++;

  if (pH_index >= 10) {
    double sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += current_pHs[i];
    }
    pH_mean = sum / (double)10;
    pH_index = 0;
  }
}

void calculateTemperatureMean(double current) {
  current_temperatures[temperature_index] = current;
  temperature_index++;

  if (temperature_index >= 10) {
    double sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += current_temperatures[i];
    }
    temperature_mean = sum / (double)10;
    temperature_index = 0;
  }
}

void calculateRPMMean(double current) {
  current_RPMs[RPM_index] = current;
  RPM_index++;

  if (RPM_index >= 10) {
    double sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += current_RPMs[i];
    }
    RPM_mean = sum / (double)10;
    RPM_index = 0;
  }
}

void sendReadingsToDevKitC() {
  if (!isnan(temperature_mean) && !isnan(RPM_mean)) {
    Serial1.printf("{\"ph\":%f,\"temperature\":%f,\"rpm\":%f}\n", 0.0, temperature_mean, RPM_mean);
    Serial.printf("Sent readings to DevKitC: {\"ph\":%.2f,\"temperature\":%.2f,\"rpm\":%.2f}\n", 0.0, temperature_mean, RPM_mean);
    temperature_mean = NAN;
    RPM_mean = NAN;
    pH_mean = NAN;
  }
}

void loop() {

  /*Ive replaced the old counter i used to use for pwn by just checking how long its been since the last pulse*/
  /*=========================================MOTOR PWM================================================*/
  motor_pwm_time = micros();

  if (motor_pwm_time - motor_pwm_time_prev <= motor_output) {
    if (motor_state != 1) { /* if the motor isn't already on, this prevents digital write being called unnecesarily it is demanding apprently according to preston*/
      digitalWrite(Motor_output_pin, HIGH);
      motor_state = 1;
    }
  } else {
    if (motor_state != 0) { /* if the motor isn't already off*/
      digitalWrite(Motor_output_pin, LOW);
      motor_state = 0;
    }
  }
  /* sets the difference of the two times back to zero*/
  if (motor_pwm_time - motor_pwm_time_prev >= 1000) {
    motor_pwm_time_prev = motor_pwm_time;
  }
  /*========================HEATING PWM============================================*/
  heat_pwm_time = micros();
  /* PWM for the heating , ON is 60% duty cycle, OFF is 0%*/
  if (heat_pwm_time - heat_pwm_time_prev <= heat_output * 10) { /*  the output is multipled by 10 becasuse 100 micro seconds
                                                                  because 100 microseconds is too short for reliable pwm, I don't think the esp32 will complete one loop in 1 microseconds*/
    if (heat_state_pwm != 1) {                                  /* if the output isn't already on, prevents the pin being turned on repeatedly*/
      digitalWrite(heatOutputPin, HIGH);
      heat_state_pwm = 1;
    }
  }

  if (heat_pwm_time - heat_pwm_time_prev > heat_output * 10) {
    //OFF section of the pwm wave
    if (heat_state_pwm != 0) { /* if the output isn't already off, same reason as before*/
      digitalWrite(heatOutputPin, LOW);
      heat_state_pwm = 0;
    }
  }
  if (heat_pwm_time - heat_pwm_time_prev > 1000) { /* if the counter is greater than 1000, go back to zero to repeat the pulse*/
    heat_pwm_time_prev = heat_pwm_time;
  }

  /*=======================MOTOR CALCULATING=======================================*/
  if (motor_killed) {
    motor_output = 0.0;
  }
  else {
    /*Determining output*/
    if (motor_time - motor_prev_time >= motor_interval) {
      noInterrupts(); /* turns off interrupt routine while taking the amount of pulses counted*/
      double pulses = pulse_count;
      pulse_count = 0;
      interrupts();
      /*calculating RPM*/
      revs = pulses / 70;
      revs_per_sec = (revs / (motor_time - motor_prev_time)) * 1000000;
      RPM = revs_per_sec * 60;
      RPM = (RPM * 0.6) + (prev_RPM * 0.4); /* smoothes out the RPM reading quite heavily because without it the readings are even more unstable*/
      motor_error = motor_setpoint - RPM;
      motor_prop = motor_error * motor_prop_coeff; /*proportional part*/

      motor_integ += motor_integ_coeff * (0.5) * ((motor_time - motor_prev_time) / 1000) * (motor_error); /* trapezium rule for integral part*/


      prev_motor_error = motor_error;

      motor_output = motor_prop + motor_integ;
      //Serial.println(motor_output);
      motor_output = constrain(motor_output, -1000, 1000);
      motor_output = map(motor_output, -1000, 1000, 0, 1000);
      /*converts output to a percentage, -500 and 500 are arbitrary and can be changed during testin, 1000 because the pwn uses unsigned long which are whole numbers*/
      //Serial.println(motor_prop);
      //Serial.print("RPM: ");    Serial.print(RPM, 1);
      calculateRPMMean(RPM);
      //Serial.print("  Error: "); Serial.print(motor_error, 1);
      //Serial.print("  PWM: ");  Serial.print(motor_output);
      //Serial.println("%");
      prev_RPM = RPM;
      motor_prev_time = motor_time;
    }
    motor_time = micros();
  }

  /*================================HEATING CALCULATION==============================*/
  if (heating_killed) {
    heat_output = -1;
  }
  else {
    /* calcuates temperature and the response to setpoint*/
    heat_time = millis();
    if (heat_time - heat_time_prev >= heat_interval) {
      Vout = (analogRead(heatInputPin) * heat_transfer); /* Measures the potential after the pd drop across the resistor in series with thermistor/
      /* max voltage is 3.3V, thre pin returns 4096 possible inputs*/

      RT = (R2 * (Vout / Vin_max)) / (1.0 - Vout / Vin_max);                               /* Calculates the resitance of the thermistor using potential divider*/
      temperature = (b * (T0 + 273.0)) / ((T0 + 273.0) * log((RT / RT_base)) + b) - 273.0; /* uses thermistor's resistance to calculate tenperature*/
                                                                                          /*Bang Bang control*/
      if (temperature <= (heat_set_point - heat_deadband)) {                               /* heat deaband is zero, but keeping it here just in case we miraculously have time to fiddle with heating again*/
        heat_output = 60;
      } else {
        heat_output = -1; /* making the duty cycle negative somehow makes the code a lot simpler*/
      }

      heat_error = heat_set_point - temperature;
      //Serial.println("Temperature"); Serial.print(temperature);
      calculateTemperatureMean(temperature);
      //Serial.println("Heating:"); Serial.print(heat_output);
      heat_time_prev = heat_time;
    }
  }


  /*========================================pH calculation and control================================================*/
  // pH_time = millis();

  //if (pH_time-pH_time_prev>=pH_interval){
  if (pumps_killed) {
    BaseOffAction();
    AcidOffAction();
  }
  else {
    Bs = analogRead(pHPin);
    /*ph calculation ------- may need to change the variables in testing*/
    pH = ((0.00932 * Bs) - 19.2);                  //gets pH by doing process mentioned line 3 might want to adjust I'm not too sure about the logic here
                                                  // Serial.println("pH"); Serial.print(pH);
    pHSmoothed = (0.9 * pHSmoothed) + (0.1 * pH);  //IIR smoothing works by reducing the weight of the latest reading- finding like an average of all readings->to help with noise from probe-> less random changes


    /*I've commented out this code here that prints out pH smoothed after an invterval because I've already added an interval for the whole pH, but keeping it here just in case*/


    /*Timems=millis();         //number of milliseconds since ESP32 MCU started



    if (Timems-T2>0){
      T2=T2+1000;      //prints pH value once a second as only when 1000 milliseconds( 1 second ) has passed will Timems-T2 be greater than 0
    Serial.println(pHSmoothed);  //prints next line for clarity
    }  */
    //Following is bang bang might want to check logic of it later
    //If the pH too high
    if (pHSmoothed > AimpH + DeltapH) {
      BaseOffAction();
      AcidOnAction();
      //LastTimeAcidOn=pH_time; //These variables for last time on can be useful later I think if we add safety precautions
    }

    // //If the pH too low
    else if (pHSmoothed < AimpH - DeltapH) {
      AcidOffAction();
      BaseOnAction();  //turn base pump on
                      //       LastTimeBaseOn=pH_time; //These variables for last time on can be useful later I think if we add safety precautions
    }
    // //If pH is in hysteresis band as you can tell it is now late and I can't be asked to annotate code in depth
    else {
      AcidOffAction();
      BaseOffAction();
    }
    //   pH_time_prev = pH_time;

    //   /*Replacement code for the code I commented out*/

    //   Serial.println("pH Smooth"); Serial.print(pH);

    //   Serial.println("pH Smoothed"); Serial.print(pHSmoothed);

    //   //Can add code for testing printing stuff later if needed :)))
    //   /* Potential for kill switches*/
  }



  readDevKitCData();
  sendReadingsToDevKitC();
} /* end of main loop*/


void pulse_counter() { /* Interupt routine for the hall effect sensor*/
  pulse_count++;
}

/*ph functions*/
//They assume that the pump works by boolean and not HIGH/LOW
void AcidOnAction() {
  digitalWrite(AcidPin, 1);
}
void AcidOffAction() {
  digitalWrite(AcidPin, 0);
}
void BaseOnAction() {
  digitalWrite(BasePin, 1);
}
void BaseOffAction() {
  digitalWrite(BasePin, 0);
}