// #include <IRremoteInt.h>
#include <IRremote.h>
#include <Servo.h>

const int pin_mcu_int = 2;
const int pin_mcc_enb = 3;
const int pin_mcc_in4 = 4;
const int pin_mcc_in3 = 5;
const int pin_mcc_in2 = 6;
const int pin_mcc_in1 = 7;
const int pin_mcc_ena = 8;

const int pin_ping_echo = 9;
const int pin_ping_trig = 10;

const int pin_servo = 11;

const int pin_ir_rx = 12;

const int pin_left_forward = pin_mcc_in2;
const int pin_left_reverse = pin_mcc_in1;
const int pin_right_forward = pin_mcc_in4;
const int pin_right_reverse = pin_mcc_in3;


// globals
Servo servo;
IRrecv ir_rx(pin_ir_rx);
decode_results ir_results;
int speed = 255; // 0-255
int desired_heading = 0;
char heading_command = 'S'; // stop all
unsigned long last_loop_ms = 0;

// using headlights command from car_rc app to turn MPU on and off
bool front_lights_on = false;
bool back_lights_on = false;
bool horn_on = false;
bool extra_on = false;


void set_speed() {
  analogWrite(pin_mcc_ena, speed);
  analogWrite(pin_mcc_enb, speed);
}


void setup() {
  Serial.begin(9600);

  pinMode(pin_mcc_ena, OUTPUT);
  pinMode(pin_mcc_enb, OUTPUT);
  pinMode(pin_mcc_in1, OUTPUT);
  pinMode(pin_mcc_in2, OUTPUT);
  pinMode(pin_mcc_in3, OUTPUT);
  pinMode(pin_mcc_in4, OUTPUT);
  pinMode(pin_ping_trig, OUTPUT);
  pinMode(pin_ping_echo, INPUT);

  set_speed();
  
  servo.attach(pin_servo);

  ir_rx.enableIRIn(); // Start the receiver
}

// set servo to angle, zero is center, negative is ccw, range is -60 to 60
void set_servo_angle(int degrees) {
  const int center = 1350;
  const int range = 900; // range from center to extreme
  
  servo.writeMicroseconds(
    center + range* degrees / 90);
}

void trace(String s) {
  Serial.println(s);
}

void servo_forward() {
  set_servo_angle(0);
}

void servo_left() {
  set_servo_angle(45);
}

void servo_right() {
   set_servo_angle(-45);
}


double ping_distance() {
   digitalWrite(pin_ping_trig, LOW);
   delayMicroseconds(2);
   digitalWrite(pin_ping_trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(pin_ping_trig, LOW);
   unsigned long timeout_us = 60000;
   double duration = pulseIn(pin_ping_echo, HIGH, timeout_us);
   double distance = (duration/2) / 29.1/2.54;
   trace((String)"ping distance: "+distance);
   return distance;
}

void go_to_wall() {
  set_servo_angle(0);
  delay(60);
  while(true) {
    double distance = ping_distance();
    
    if (distance>18) { 
      forward();
    } else if (distance<16) {  
      reverse();
    }
    else {
      stop();
      break;
    }
    delay(60); // go forward a little and also wait for ping to settle
    coast();   // have to be coasting when we do the next ping because of noise
    //delay(60); // go forward a little and also wait for ping to settle
  }
}



void turn_left() {
  trace("left");
  servo_left();
  digitalWrite(pin_left_forward, LOW);
  digitalWrite(pin_left_reverse, HIGH);
  digitalWrite(pin_right_forward, HIGH);
  digitalWrite(pin_right_reverse, LOW);
}

void turn_right() {
  trace("right");
  servo_right();
  digitalWrite(pin_left_forward, HIGH);
  digitalWrite(pin_left_reverse, LOW);
  digitalWrite(pin_right_forward, LOW);
  digitalWrite(pin_right_reverse, HIGH);
}


void forward() {
  const double heading_tolerance = 3;
  trace("forward");
  servo_forward();
  double heading_error = 0;

  digitalWrite(pin_left_forward, HIGH);
  digitalWrite(pin_left_reverse, LOW);

  digitalWrite(pin_right_forward, HIGH);
  digitalWrite(pin_right_reverse, LOW);
}

void reverse() {
  trace("reverse");
  digitalWrite(pin_left_forward, LOW);
  digitalWrite(pin_left_reverse, HIGH);
  digitalWrite(pin_right_forward, LOW);
  digitalWrite(pin_right_reverse, HIGH);
}


void stop() {
  trace("stop");
  digitalWrite(pin_left_forward, HIGH);
  digitalWrite(pin_left_reverse, HIGH);
  digitalWrite(pin_right_forward, HIGH);
  digitalWrite(pin_right_reverse, HIGH);
}

void coast() {
  trace("coast");
  digitalWrite(pin_left_forward, LOW);
  digitalWrite(pin_left_reverse, LOW);
  digitalWrite(pin_right_forward, LOW);
  digitalWrite(pin_right_reverse, LOW);
}


void remote_control() {
  if (Serial.available() > 0) {
    int key = Serial.read();
    switch(key) {
      case 'F':       // forward
        heading_command = key;
        break;
      case 'B':       // back
        heading_command = key;
        break;
      case 'L':       // left
      case 'G':       // forward left
      case 'H':       // back left
        heading_command = key;
        break;
      case 'R':       // right
      case 'I':       // forward right
      case 'J':       // back right
        heading_command = key;
        break;
      case 'S':       // stop
      case 'D':       // stop all
        heading_command = key;
        stop();
        break;
      case '0':  // speeds, 0% - 90%
        speed = (int) (255 * .0);
        set_speed();
        break;
      case '1':
        speed = (int) (255 * .1);
        set_speed();
        break;
      case '2':
        speed = (int) (255 * .2);
        set_speed();
        break;
      case '3':
        speed = (int) (255 * .3);
        set_speed();
        break;
      case '4':
        speed = (int) (255 * .4);
        set_speed();
        break;
      case '5':
        speed = (int) (255 * .5);
        set_speed();
        break;
      case '6':
        speed = (int) (255 * .6);
        set_speed();
        break;
      case '7':
        speed = (int) (255 * .7);
        set_speed();
        break;
      case '8':
        speed = (int) (255 * .8);
        set_speed();
        break;
      case '9':
        speed = (int) (255 * .9);
        set_speed();
        break;
      case 'q': // full speed
        speed = 255;
        set_speed();
        break;
      case 'W':
        go_to_wall();
        front_lights_on = true;
        break;
      case 'w':
        front_lights_on = false;
        break;
      case 'U':
        back_lights_on = true;
        break;
      case 'u':
        back_lights_on = false;
        break;
      case 'V':
        horn_on = true;
        break;
      case 'v':
        horn_on = false;
        break;
      case 'X':
        extra_on = true;
        break;
      case 'x':
        extra_on = false;
        break;
      default:
        break;
    }
  }

  switch(heading_command) {
  case 'F':       // forward
    forward();
    break;
  case 'B':       // back
    reverse();
    break;
  case 'L':       // left
  case 'G':       // forward left
  case 'H':       // back left
    turn_left();
    break;
  case 'R':       // right
  case 'I':       // forward right
  case 'J':       // back right
    turn_right();
    break;
  case 'S':       // stop
  case 'D':       // stop all
    stop();
    break;
}

}

// returns true if loop time passes through n ms boundary
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;

}

void loop() {
  go_to_wall();
  return;
  unsigned long loop_ms = millis();
  bool every_second = every_n_ms(last_loop_ms, loop_ms, 1000);
  bool every_100_ms = every_n_ms(last_loop_ms, loop_ms, 100);
  bool every_10_ms = every_n_ms(last_loop_ms, loop_ms, 10);

  if(every_10_ms) {
    remote_control();
  }
  last_loop_ms = loop_ms;
}
