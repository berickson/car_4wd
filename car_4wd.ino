// #include <IRremoteInt.h>
#include <IRremote.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <Wire.h>

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

#include "Mpu9150.h"


// globals
Servo servo;
IRrecv ir_rx(pin_ir_rx);
decode_results ir_results;
int speed = 255; // 0-255
Mpu9150 mpu;
int desired_heading = 0;
char heading_command = 'S'; // stop all
unsigned long last_loop_ms = 0;

// using headlights command from car_rc app to turn MPU on and off
bool front_lights_on = false;
bool back_lights_on = false;
bool horn_on = false;
bool extra_on = false;
#define use_mpu front_lights_on


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

  set_speed();
  
  servo.attach(pin_servo);

  mpu.setup();        // start mpu
  //ir_rx.enableIRIn(); // Start the receiver

}


void trace(String s) {
  return;
  Serial.println(s);
}

void servo_forward() {
  servo.writeMicroseconds(1350);
}

void servo_left() {
  servo.writeMicroseconds(1700);
}

void servo_right() {
   servo.writeMicroseconds(900);
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
  if(use_mpu)  {
    heading_error = mpu.ground_angle() - desired_heading + 360;
    while (heading_error > 180) {
      heading_error -= 360;
    }
  }

  // too far to the right?
  if(use_mpu && heading_error < -heading_tolerance) {
     digitalWrite(pin_left_forward, LOW);  // turn left by stalling left wheel
  } else {
     digitalWrite(pin_left_forward, HIGH);
  }
  digitalWrite(pin_left_reverse, LOW);

  // too far to the left?
  if(use_mpu && heading_error > heading_tolerance) {
     digitalWrite(pin_right_forward, LOW); // turn right by stalling right wheel
  } else {
     digitalWrite(pin_right_forward, HIGH);
  }
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


void remote_control() {
  if (Serial.available() > 0) {
    int key = Serial.read();
    switch(key) {
      case 'F':       // forward
        if(heading_command != key) {
          desired_heading = mpu.ground_angle();
          heading_command = key;
        }
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
  unsigned long loop_ms = millis();
  bool every_100_ms = every_n_ms(last_loop_ms, loop_ms, 100);
  bool every_10_ms = every_n_ms(last_loop_ms, loop_ms, 10);


  if(use_mpu) { 
    if(every_100_ms) {
      Serial.println(mpu.ground_angle());
    }
    mpu.execute();
  }

  if(every_10_ms) {
    remote_control();
  }
  /*
  if (ir_rx.decode(&ir_results)) {
   trace("received IR: ");
   Serial.println(ir_results.value, HEX);
   ir_rx.resume(); // Receive the next value
  }
  */
  last_loop_ms = loop_ms;
}
