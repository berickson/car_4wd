/*
Board:      Arduino Uno
Programmer: USB Asp
*/

#include <IRremote.h>
#include <Servo.h>
#include <FlexiTimer2.h>

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

const int pin_right_encoder = A5;
const int pin_left_encoder = A0;


// globals
Servo servo;
IRrecv ir_rx(pin_ir_rx);
decode_results ir_results;
int speed = 255; // 0-255
int desired_heading = 0;
char heading_command = 'S'; // stop all
unsigned long last_loop_ms = 0;
unsigned long last_ping_start_ms = 0;

bool front_lights_on = false;
bool back_lights_on = false;
bool horn_on = false;
bool extra_on = false;

volatile unsigned long right_encoder_count;
volatile unsigned long left_encoder_count;
volatile int right_encoder_value;
volatile int left_encoder_value;


// infrared receiver
decode_results results;

enum mode_enum {
  mode_manual,
  mode_follow_closest,
  mode_go_to_wall,
  mode_back_and_forth
} mode;


void set_speed() {
  if(speed < 255) {
    analogWrite(pin_mcc_ena, speed);
    analogWrite(pin_mcc_enb, speed);
  } else {
    digitalWrite(pin_mcc_ena, 1);
    digitalWrite(pin_mcc_enb, 1);
  }
}

void timer_interrupt_handler() {
  int new_right_encoder_value = digitalRead(pin_right_encoder);
  if(right_encoder_value != new_right_encoder_value) {
    right_encoder_count++;
    right_encoder_value = new_right_encoder_value;
  }
  int new_left_encoder_value = digitalRead(pin_left_encoder);
  if(left_encoder_value != new_left_encoder_value) {
    left_encoder_count++;
    left_encoder_value = new_left_encoder_value;
  }
 
}


// these functions set digital high and low for MCCC
// this is special because of the MCC pullup resistors
// that wants high impedance to set a TRUE value
void mcc_high(int pin) {
  pinMode(pin, INPUT);
}

void mcc_low(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}


void setup() {
  mcc_high(pin_mcc_ena);
  mcc_high(pin_mcc_enb);
  mcc_high(pin_mcc_in1);
  mcc_high(pin_mcc_in2);
  mcc_high(pin_mcc_in3);
  mcc_high(pin_mcc_in4);

  Serial.begin(9600);

  
  pinMode(pin_ping_trig, OUTPUT);
  pinMode(pin_ping_echo, INPUT);

  set_speed();
  
  servo.attach(pin_servo);
  
  const int timer_us = 1000;
  FlexiTimer2::set(1, timer_interrupt_handler); // 1ms period
  FlexiTimer2::start();
  servo_forward();
  

  // ir_rx.enableIRIn(); // Start the receiver
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
   // wait for previous ping to settle
   const unsigned long min_time_between_pings_ms = 30;
   while(millis() - last_ping_start_ms < min_time_between_pings_ms) {
     delay(1);
   }
   last_ping_start_ms = millis();
   digitalWrite(pin_ping_trig, LOW);
   delayMicroseconds(2);
   digitalWrite(pin_ping_trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(pin_ping_trig, LOW);
   unsigned long timeout_us = 20000;
   double duration = pulseIn(pin_ping_echo, HIGH, timeout_us);
   double distance = (duration/2) / 29.1/2.54;
   trace((String)"ping distance: "+distance);
   
   // see post #16 at http://forum.arduino.cc/index.php?topic=55119.15
   if(distance == 0) {
      pinMode(pin_ping_echo, OUTPUT);
      digitalWrite(pin_ping_echo, LOW);
      delay(10);
      pinMode(pin_ping_echo, INPUT);
   }
   
   return distance;
}

void go_to_wall() {
  set_servo_angle(0);
  delay(20); 
  double distance = ping_distance();
  
  if (distance>18) { 
    forward();
  } else if (distance<16 && distance > 0) {  
    reverse();
  }
  else {
    coast();
  }
}

void turn_to_angle(double angle) {
  double ms_per_degree = 3.0;
  if(angle > 0)
    turn_left();
  if(angle < 0)
    turn_right();
  delay(abs(angle) * ms_per_degree);
  stop();
  delay(100);
  coast();
}


bool scan_for_closest(double * min_angle, double * min_distance) {
  trace("scanning for closest");
  bool found = false;
  set_servo_angle(90);
  delay(300);
  for(double angle = 90; angle >= -90; angle -=15) {
    set_servo_angle(angle);
    delay(300); // going much faster messes with the ping results
    double distance = ping_distance();
    if((!found || (distance < *min_distance)) && distance > 0) {
      found = true;
      *min_distance = distance;
      *min_angle = angle;
    }
  }
  if(found) {
    trace((String)"min_angle: " + *min_angle + "min_distance: " + *min_distance);
  }
  return found;
}

void go_inches(double inches) {
  go_ticks(inches * 5.52); 
}

void go_ticks(int ticks) {
  int original_ticks = right_encoder_count;
  if(ticks >= 0) {
    forward();
  } else {
    reverse();
  }
  while(abs(right_encoder_count - original_ticks) < abs(ticks)) {
    delay(1);
  }
  stop();
}

void follow_closest() {
  coast();
  double angle = 0;
  double distance = 0;
  double desired_distance = 5; // inches
  if(scan_for_closest(&angle, &distance)) {
    set_servo_angle(angle);
    turn_to_angle(angle);
    set_servo_angle(0);
    go_inches(constrain(distance-desired_distance,-10,10));
  }
}


void turn_left() {
  servo_left();
  mcc_low(pin_left_forward);
  mcc_high(pin_left_reverse);
  mcc_high(pin_right_forward);
  mcc_low(pin_right_reverse);
}

void turn_right() {
  servo_right();
  mcc_high(pin_left_forward);
  mcc_low(pin_left_reverse);
  mcc_low(pin_right_forward);
  mcc_high(pin_right_reverse);
}


void forward() {
  const double heading_tolerance = 3;
  servo_forward();
  double heading_error = 0;

  mcc_high(pin_left_forward);
  mcc_low(pin_left_reverse);

  mcc_high(pin_right_forward);
  mcc_low(pin_right_reverse);
}

void reverse() {
  mcc_low(pin_left_forward);
  mcc_high(pin_left_reverse);
  mcc_low(pin_right_forward);
  mcc_high(pin_right_reverse);
}


void stop() {
  mcc_high(pin_left_forward);
  mcc_high(pin_left_reverse);
  mcc_high(pin_right_forward);
  mcc_high(pin_right_reverse);
}

void coast() {
  mcc_low(pin_left_forward);
  mcc_low(pin_left_reverse);
  mcc_low(pin_right_forward);
  mcc_low(pin_right_reverse);
}


void read_remote_control() {
  if (Serial.available() > 0) {
    int key = Serial.read();
    switch(key) {
      case 'F':       // forward
        heading_command = key;
        mode = mode_manual;
        break;
      case 'B':       // back
        heading_command = key;
        mode = mode_manual;
        break;
      case 'L':       // left
      case 'G':       // forward left
      case 'H':       // back left
        heading_command = key;
        mode = mode_manual;
        break;
      case 'R':       // right
      case 'I':       // forward right
      case 'J':       // back right
        heading_command = key;
        mode = mode_manual;
        break;
      case 'S':       // stop
      case 'D':       // stop all
        heading_command = key;
        mode = mode_manual;
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
        mode = mode_follow_closest;
        break;
      case 'w':
        front_lights_on = false;
        mode = mode_manual;
        break;
      case 'U':
        back_lights_on = true;
        mode = mode_go_to_wall;
        break;
      case 'u':
        back_lights_on = false;
        mode = mode_manual;
        break;
      case 'V':
        horn_on = true;
        mode = mode_back_and_forth;
        break;
      case 'v':
        horn_on = false;
        mode = mode_manual;
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
}

void follow_remote_control_commands() {
  
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

void read_ir_remote_control() {
  if (ir_rx.decode(&results)) {
    if (results.decode_type == NEC) {
      Serial.print("NEC: ");
    } else if (results.decode_type == SONY) {
      Serial.print("SONY: ");
    } else if (results.decode_type == RC5) {
      Serial.print("RC5: ");
    } else if (results.decode_type == RC6) {
      Serial.print("RC6: ");
    } else if (results.decode_type == UNKNOWN) {
      Serial.print("UNKNOWN: ");
    }
    Serial.println(results.value, HEX);
    switch(results.value){
         // "special for mp3" remote control

        case 0xFFA25D: // power
          mode = mode_manual;
          heading_command = 'S';
          break;
        case 0xFF629D: // mode
          mode = mode_follow_closest;
          break;
        case 0xFFE21D: // mute
          mode = mode_go_to_wall;
          break;
        case 0xFF22DD: // play/stop button
          if(heading_command == 'F') {
            heading_command = 'S';
          } else {
            heading_command = 'F';
          }
          break;
        case 0xFF02FD: //  previous track
          if(heading_command == 'L') {
            heading_command = 'S';
          } else {
            heading_command = 'L';
          }
          break;
        case 0xFFC23D: // next track
          if(heading_command == 'R') {
            heading_command = 'S';
          } else {
            heading_command = 'R';
          }
          break;
        case 0xFFE01F: // eq
          if(heading_command == 'B') {
            heading_command = 'S';
          } else {
            heading_command = 'B';
          }
          break;
        case 0xFFA857: // -
          break;
        case 0xFF906F: // +
          break;
        case 0xFF6897: // 0
          break;
        case 0xFF9867: // shuffle?
          break;
        case 0xFFB04F: // U/SD
          break;
        case 0xFF30CF: // 1
          break;
        case 0xFF18E7: // 2
          break;
        case 0xFF7A85: // 3
          break;
        //case 0x: // 4 doesn't work
        //  break;
        case 0xFF38C7: // 5
          break;
        case 0xFF5AA5: // 6
          break;
        case 0xFF42BD: // 7
          break;
        case 0xFF4AB5: // 8
          break;
        case 0xFF52AD: // 9
          break;
        case 0xFFFFFFFF: // repeat
          break;
          
        default:
          break;
    }
    
    ir_rx.resume(); // Receive the next value
  }
  read_remote_control();

}


void go_back_and_forth() {
    delay(5000);
    go_inches(12);
    delay(5000);
    go_inches(-12);
}

// returns true if loop time passes through n ms boundary
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;

}

int last_reported_right_encoder_count = 0;
int last_reported_left_encoder_count = 0;
void loop() {
  unsigned long loop_ms = millis();
  bool every_second = every_n_ms(last_loop_ms, loop_ms, 1000);
  bool every_100_ms = every_n_ms(last_loop_ms, loop_ms, 100);
  bool every_10_ms = every_n_ms(last_loop_ms, loop_ms, 10);


  if(every_10_ms) {
    read_remote_control();
    read_ir_remote_control();
    switch(mode) {
      case mode_manual:
        follow_remote_control_commands();
        break;
      case mode_follow_closest:
        follow_closest();
        break;
      case mode_go_to_wall:
        go_to_wall();
        break;
      case mode_back_and_forth:
        go_back_and_forth();
        break;
      default:
        Serial.print("error: invalid mode");
        break;
    }
  }
  if(every_second) {
    Serial.print("ping: ");
    Serial.print(ping_distance());
    Serial.print("right: ");
    Serial.print(right_encoder_count);
    Serial.print(", ");
    Serial.print(right_encoder_count - last_reported_right_encoder_count);
    last_reported_right_encoder_count = right_encoder_count;
    Serial.print("  left: ");
    Serial.print(left_encoder_count);
    Serial.print(", ");
    Serial.println(left_encoder_count - last_reported_left_encoder_count);
    last_reported_left_encoder_count = left_encoder_count;
  }


  last_loop_ms = loop_ms;
}
