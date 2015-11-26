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

bool front_lights_on = false;
bool back_lights_on = false;
bool horn_on = false;
bool extra_on = false;


// infrared receiver
decode_results results;

enum mode_enum {
  mode_manual,
  mode_follow_closest,
  mode_go_to_wall
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
  coast();
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
  delay(60); // go forward a little and also wait for ping to settle
  coast();   // have to be coasting when we do the next ping because of noise
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
  for(double angle = 90; angle >= -90; angle -=15) {
    set_servo_angle(angle);
    delay(300);
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
  double ms_per_inch = 50;
  if(inches > 0)
    forward();
  else
    reverse();
  delay(abs(inches) * ms_per_inch);
  coast();
}

void follow_closest() {
  delay(100);
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


void read_remote_control() {
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
        front_lights_on = true;
        mode = mode_follow_closest;
        break;
      case 'w':
        front_lights_on = false;
        mode = mode_manual;
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

// returns true if loop time passes through n ms boundary
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;

}

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
      default:
        Serial.print("error: invalid mode");
        break;
    }
  }
  last_loop_ms = loop_ms;
}
