// #include <IRremoteInt.h>
#include <IRremote.h>


#include <Servo.h>

const int pin_mcc_enb = 2;
const int pin_mcc_in4 = 3;
const int pin_mcc_in3 = 4;
const int pin_mcc_in2 = 5;
const int pin_mcc_in1 = 6;
const int pin_mcc_ena = 7;

const int pin_ping_echo = 8;
const int pin_ping_trig = 9;

const int pin_servo = 10;

const int pin_ir_rx = 11;

const int pin_left_forward = pin_mcc_in4;
const int pin_left_reverse = pin_mcc_in3;
const int pin_right_forward = pin_mcc_in2;
const int pin_right_reverse = pin_mcc_in1;

// globals
Servo servo;
IRrecv ir_rx(pin_ir_rx);
decode_results ir_results;


void setup() {
  Serial.begin(115200);
  Serial.println("setup");
  // put your setup code here, to run once:
  pinMode(pin_mcc_ena, OUTPUT);
  pinMode(pin_mcc_enb, OUTPUT);
  pinMode(pin_mcc_in1, OUTPUT);
  pinMode(pin_mcc_in2, OUTPUT);
  pinMode(pin_mcc_in3, OUTPUT);
  pinMode(pin_mcc_in4, OUTPUT);
  
  digitalWrite(pin_mcc_ena, HIGH);
  digitalWrite(pin_mcc_enb, HIGH);

  servo.attach(pin_servo);

  ir_rx.enableIRIn(); // Start the receiver

}


void trace(String s) {
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
  digitalWrite(pin_left_forward, HIGH);
  digitalWrite(pin_left_reverse, LOW);
  digitalWrite(pin_right_forward, LOW);
  digitalWrite(pin_right_reverse, HIGH);
}

void turn_right() {
  trace("right");
  servo_right();
  digitalWrite(pin_left_forward, LOW);
  digitalWrite(pin_left_reverse, HIGH);
  digitalWrite(pin_right_forward, HIGH);
  digitalWrite(pin_right_reverse, LOW);
}


void forward() {
  trace("forward");
  servo_forward();
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
  digitalWrite(pin_mcc_in1, HIGH);
  digitalWrite(pin_mcc_in2, HIGH);
  digitalWrite(pin_mcc_in3, HIGH);
  digitalWrite(pin_mcc_in4, HIGH);
}


void loop() {
  if (ir_rx.decode(&ir_results)) {
   Serial.print("received IR: ");
   Serial.println(ir_results.value, HEX);
   ir_rx.resume(); // Receive the next value
  }
  turn_left();
  delay(1000);
  
  stop();
  delay(1000);
  
  turn_right();
  delay(1000);

  stop();
  delay(1000);

  forward();
  delay(1000);

  stop();
  delay(1000);
  
  reverse();
  delay(1000);

  stop();
  delay(1000);
}
