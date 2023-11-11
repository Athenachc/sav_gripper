#include <Wire.h>

/*
  pressure_range (kPa)   K value
  131<P≤260               32
  65<P≤131                64
  32<P≤65                 128
  16<P≤32                 256
  8<P≤16                  512
  4<P≤8                   1024
  2≤P≤4                   2048
  1≤P<2                   4096
  P<1                     8192
*/
#define K 64              //Check with the table above
#define I2C_address 0x6D  // Unfortunately, it does not seem possible to change it. To use multiple sensors, use the analog version (XGZP6897A)

#define pump_en 39          //pump enable
#define pump_in1 38         //pump +ve
#define pump_in2 37         //pump -ve
#define valve_in1 5         //vacuum +ve
#define valve_in2 6         //vacuum -ve
#define pressure_reading 2  //pressure sensor
#define pwm_cmd 2           //pwm command from servo tester/ROS

int pwm_val;  //reading of pwm from servo tester/ROS
int pwm_test;
double pwm;   //pwm of pump

// --------------------- PID control parameters ---------------------
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setpressure;
double cumError, rateError;
double kp; // kp = max. pwm of motor / setpressure
double ki;
double kd ;
double kp_offset = 175;
double kp_out;
int motor_min_pwm;
double fp;
// --------------------- PID control parameters ---------------------

void setup() {
  Serial.begin(9600);
  pinMode(pwm_cmd, INPUT);
  pinMode(pump_in1, OUTPUT);
  pinMode(pump_in2, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(pump_en, 0);
  digitalWrite(pump_in1, HIGH);
  digitalWrite(pump_in2, LOW);

  pinMode(valve_in1, OUTPUT);
  pinMode(valve_in2, OUTPUT);
  pinMode(pressure_reading, INPUT);
  Serial.print("Testing Testing~");

  Wire.begin();
  Wire.setClock(100000);

  delay(100);
  Serial.print("start");
}

// ----------------------------------- PID -----------------------------------
// double computePID(double inp) {
//   currentTime = millis();
//   elapsedTime = (double)(currentTime - previousTime);
//   error = setpressure - inp;                      // determine error
//   cumError += error * elapsedTime;                // Compute integral
//   rateError = (error - lastError) / elapsedTime;  // Compute derivative

//   //double out = kp * error;                      // P controller
//   Serial.print("cumError: ")  ;
//   Serial.println(cumError);

//   double out = kp*error + ki*cumError;           // PI controller
//   //double out = kp*error + ki*cumError + kd*rateError; // PID output
//   lastError = error;           // Record current error
//   previousTime = currentTime;  // Record current time
//   return out;
// }

// ------------------------------ Reading pressure  ---------------------------------
double readPressure() {
  unsigned char pressure_H, pressure_M, pressure_L, temperature_H, temperature_L;  // Temporary variables of pressure and temperature
  long int pressure_adc, temperature_adc;                                          // Values of pressure and temperature converted by the sensor's ADC
  double pressure, temperature;                                                    // Calibrated values of pressure and temperature
  write_one_byte(I2C_address, 0x30, 0x0A);

  // Indicate a combined conversion (once temperature conversion immediately followed by once sensor signal conversion)
  // More measurement method, check Register 0x30

  long timeout = 0;
  while ((Read_One_Byte(I2C_address, 0x30) & 0x08) > 0)  // && timeout < 65000)
  {
    timeout++;
    if (timeout > 5000) {
      Serial.println("Timeout! Please check the connections!");
      break;
    }
  }
  // Judge whether data collection is over
  delay(20);
  pressure_H = Read_One_Byte(I2C_address, 0x06);
  pressure_M = Read_One_Byte(I2C_address, 0x07);
  pressure_L = Read_One_Byte(I2C_address, 0x08);

  // Read ADC output data of pressure
  pressure_adc = pressure_H * 65536 + pressure_M * 256 + pressure_L;
  //Compute the value of pressure converted by ADC
  if (pressure_adc > 8388608) {
    pressure = (pressure_adc - 16777216) / K;  //unit: Pa, Rmb to select appropriate K value according to preesure range
  } else {
    pressure = pressure_adc / K;  //unit: Pa, Rmb to select appropriate K value according to preesure range
  }

  // For debugging
  //Serial.print("Pressure: ");
  //Serial.print(int(pressure) / 1000);
  //Serial.println(" kPa");

  return pressure;
}
//----write One Byte of Data,Data from Arduino to the sensor----
// Write "thedata" to the sensor's address of "addr"
void write_one_byte(uint8_t device_address, uint8_t addr, uint8_t thedata) {
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.write(thedata);
  Wire.endTransmission();
}

//----Read One Byte of Data,Data from the sensor to the Arduino ----
uint8_t Read_One_Byte(uint8_t device_address, uint8_t addr) {
  uint8_t nb_bytes = 1;
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(device_address, nb_bytes);
  return Wire.read();  // Receive a byte as character
}

// ----------------------------- Valves control -----------------------------
void inflation() {
  digitalWrite(valve_in1, HIGH);
  digitalWrite(valve_in2, LOW);
  //Serial.print("inflation");
}


void deflation() {
  digitalWrite(valve_in1, LOW);
  digitalWrite(valve_in2, HIGH);
  //Serial.print("deflation");
}

void stopValves() {
  digitalWrite(valve_in1, LOW);
  digitalWrite(valve_in2, LOW);
}

void loop() {
  double pressure = readPressure();
  // ----------------------------------------- PWM input -----------------------------------------
  pwm_val = pulseIn(pwm_cmd, HIGH);
  Serial.print("PWM input: ");
  Serial.println(pwm_val);
  if (pwm_val > 1493) {
    inflation();
    setpressure = 85;
    motor_min_pwm = 220;
    kp = (255 - motor_min_pwm) / setpressure;
    //ki = 0.001;
    fp = 0.8;
//    if (pressure < 0){
//    fp = 1/setpressure;
//    }
//    else {fp = 0.8;}
  } //+90 kPa
  if (pwm_val < 1493 && pwm_val > 1000) {
    deflation();
    setpressure = -25;
    motor_min_pwm = 160;
    kp = (255 - motor_min_pwm) / setpressure;
    //ki = 0.001;
    fp = 1/setpressure;
  }//-40 kPa
  if (pwm_val < 950) {
    //stopValves();
    setpressure = 0;
    if (pressure > 0){
      deflation();
    }
    else {stopValves();}
  }


  // ----------------------------------------- PID control -----------------------------------------
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);
  input = int(pressure) / 1000;
  error = setpressure - input;
  cumError += error * elapsedTime;
  
  //rateError = (error - lastError) / elapsedTime;
  //kd;
  //output = kp * error + motor_min_pwm; //P control
  output = kp * error + motor_min_pwm + fp * input; //feedforward P control
  //output = kp * error + motor_min_pwm + ki * cumError; //PI control
  //output = kp * error + motor_min_pwm + ki * cumError + fp * input; //feedforward PI control
  lastError = error;
  previousTime = currentTime;
  if (output > 255) {
    output = 255;
  }
  if (output < 0 ) {
    output = 0;
  }
//  if (error < 0) {
//    deflation();
//  }
//  if (error > 0){
//    inflation();
//  }
  
  analogWrite(pump_en, output);
  //if (error = 0) stopValves();

//  Serial.print(-50);  //lower limit
//  Serial.print(" ");
//  Serial.print(120);  //upper limit
//  Serial.print(" ");
  static unsigned long last_time = 0;
  unsigned long current_time = millis();
  if (current_time - last_time >=50){
    last_time = current_time;
    Serial.print("set_P: ");
    Serial.print(setpressure);
    Serial.print(" ");
    Serial.print("reading_P: ");
    Serial.println(input);
    //Serial.println(ki);
  }
  
  //Serial.println(" ");
  //Serial.println(output);
}
