#include <Servo.h>             //Servo library
#include <SharpIR.h>           // IR Library

/*
  Function to do what I want but like python: data.split(separator)[index]
*/

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int servo_dirs = 45; //Initialize servo direction

/*
  PWM Motor
*/
// PWM output pin (SPEED)
#define pwm      3
// Motor direction pin (DIRECTION)
#define dir      2
// Variable to represent PWM value
int pwm_value = 0;   
String wtf;
/* 
 *  SERVO
 */
#define dir_servo   7
#define break_servo   8

/*
 * IR Sensors
 */

#define ir_back_left_pin A0
#define ir_back_right_pin A1 
#define ir_side_pin A2

#define model 430 //GP2YA41SK0F


//Initialize IR Objects
SharpIR ir_back_left = SharpIR(ir_back_left_pin, model);
SharpIR ir_back_right = SharpIR(ir_back_right_pin, model);
SharpIR ir_side = SharpIR(ir_side_pin, model);


//Initialize Servo Object
Servo car_direction;
Servo car_break;

void setup(){

  //Serial Rate
  Serial.begin(115200);
  Serial.setTimeout(50);
  
  // Define Pins
  pinMode(pwm,OUTPUT);
  pinMode(dir,OUTPUT);

  //Attach Servo
  car_direction.attach(dir_servo);
  car_break.attach(break_servo);

  //Set pwm and dir pins to 0
  analogWrite(pwm, 0);
  digitalWrite(dir, 0);
  
  direction_driver(45); //Align car
}

void motor_driver(int speed_value, int change_direction) 
{
  // Ensure PWM value ranges from 0 to 255
  pwm_value = speed_value;
  if (pwm_value > 255) {
    pwm_value = 255;  
  } else if (pwm_value < 0) {
    pwm_value = 0;  
  }
  analogWrite(pwm, pwm_value);
  //Check for direction change
  digitalWrite(dir,change_direction);
  if(change_direction == 3) 
  {
    analogWrite(pwm, pwm_value/2);
    delay(500);
    analogWrite(pwm, 0); //Stop
  }
  Serial.print("Motor_Driver: PWM: ");
  Serial.print(pwm_value);
  Serial.print(" DIRECTION: ");
  Serial.print(digitalRead(dir));
  Serial.print("\n");
} 

void direction_driver(int srvdir) 
{
  if (srvdir > 180) 
  {
    srvdir = 180;
  }
  if (srvdir < 0) 
  {
    srvdir = 0;
  }
  
  car_direction.write(srvdir);
  Serial.print("Direction_Driver: ");
  Serial.print(srvdir);
  Serial.print("\n");
}

void break_driver(int srvbreak) 
{
  if (srvbreak > 180) 
  {
    srvbreak = 180;
  }
  if (srvbreak < 0) 
  {
    srvbreak = 0;
  }
  
  car_break.write(srvbreak);
  Serial.print("Break_Driver: ");
  Serial.print(srvbreak);
  Serial.print("\n");
}

int get_ir() {
  int distance_back_left = ir_back_left.distance();
  int distance_back_right = ir_back_right.distance();
  int distance_side = ir_side.distance();
  return distance_back_left; //We dont need this right now
}


void loop()
{
    int vel;
    int dirs;
    String method;
    String ir_code;
    String datafromUser;
    direction_driver(servo_dirs); //LOCK Servo direction
    if(Serial.available() > 0)
    {
      datafromUser = Serial.readString();
      method = getValue(datafromUser, ':', 0);
      String datapkt = getValue(datafromUser, ':', 1); //Python FOR THE WIN, it's like datafromUser.split(":")[1]
      if (method == "POST") 
      {
        String motorCTRL = getValue(datapkt, 'S', 0);
        vel = getValue(motorCTRL, '&', 0).toInt();
        dirs = getValue(motorCTRL, '&', 1).toInt();
        servo_dirs = getValue(datapkt, 'S', 1).toInt();
        if (vel == 0 && dirs == 0) { //Ignore callback - 0 0 IS NOT STOP, USE - 0 3 INSTEAD
          
        } else {
          motor_driver(vel, dirs);
        }
      } 
      else if (method == "GET")
      {
        ir_code = datapkt;
        Serial.print("{'ir_back_left':");
        Serial.print(get_ir());
        Serial.print(",'ir_back_right':");
        Serial.print("0");
        Serial.print(",'ir_side':");
        Serial.print("0");
        Serial.print("}");
        Serial.print(" = ");
        Serial.println(ir_code);
      }
      else if (method == "SHOW_LAST"){ //For debug
        Serial.println("LAST METHOD ENTERED WAS:");
        Serial.println(wtf);
      }
      wtf = datafromUser;
    }
}
