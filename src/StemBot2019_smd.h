//****************** HCSR04 setting ******************//
#include "hcsr04.h"
#define TRIG_PIN 4
#define ECHO_PIN 2
//HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, min_distance(mm), max_distance(mm));
HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);
//****************** HCSR04 setting ******************//

//****************** OLED setting ******************//
#ifdef OLED
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
#endif
//****************** OLED setting ******************//

//****************** PID Setting ******************//
//pid settings and gains
int Speed = 255, out_min = -Speed, out_max = Speed;
double IR_position = 0 , setPoint = 0, outputVal = 0, KP = 0, KI = 0, KD = 0;
//****************** PID Setting ******************//

//pins input
const int8_t IR1 = A7, IR2 = A6, IR3 = A3, IR4 = A2, IR5 = A1;
const int8_t battMon_pin = A0;

//pins output
const int8_t pwmL1 = 3, pwmL2 = 5, pwmR1 = 9, pwmR2 = 10, led_r = 13, led_y = 1, led_g = 0, battMon_en = 6;

void LED(bool r, bool y, bool g) {
  digitalWrite(led_r, r);
  digitalWrite(led_y, y);
  digitalWrite(led_g, g);
}

int offsetL = 0, offsetR = 0;
bool invert_L = 0, invert_R = 0;
void forward(int left, int right) {
  if (invert_L == 0) {
    digitalWrite(pwmL2, 0);
    analogWrite(pwmL1, constrain( (left + offsetL), 0, 255 ));
  }
  else {
    digitalWrite(pwmL1, 0);
    analogWrite(pwmL2, constrain( (left + offsetL), 0, 255 ));
  }
  if (invert_R == 0) {
    digitalWrite(pwmR2, 0);
    analogWrite(pwmR1, constrain( (right - offsetR), 0, 255 ));
  }
  else {
    digitalWrite(pwmR1, 0);
    analogWrite(pwmR2, constrain( (right - offsetR), 0, 255 ));
  }
}
void backward(int left, int right) {
  if (invert_L == 0) {
    digitalWrite(pwmL1, 0);
    analogWrite(pwmL2, constrain( (left + offsetL), 0, 255 ));
  }
  else {
    digitalWrite(pwmL2, 0);
    analogWrite(pwmL1, constrain( (left + offsetL), 0, 255 ));
  }
  if (invert_R == 0) {
    digitalWrite(pwmR1, 0);
    analogWrite(pwmR2, constrain( (right - offsetR), 0, 255 ));
  }
  else {
    digitalWrite(pwmR2, 0);
    analogWrite(pwmR1, constrain( (right - offsetR), 0, 255 ));
  }
}
void turnLeft(int left, int right) {
  if (invert_L == 0) {
    digitalWrite(pwmL1, 0);
    analogWrite(pwmL2, constrain( (left + offsetL), 0, 255 ));
  }
  else {
    digitalWrite(pwmL2, 0);
    analogWrite(pwmL1, constrain( (left + offsetL), 0, 255 ));
  }
  if (invert_R == 0) {
    digitalWrite(pwmR2, 0);
    analogWrite(pwmR1, constrain( (right - offsetR), 0, 255 ));
  }
  else {
    digitalWrite(pwmR1, 0);
    analogWrite(pwmR2, constrain( (right - offsetR), 0, 255 ));
  }
}
void turnRight(int left, int right) {
  if (invert_L == 0) {
    digitalWrite(pwmL2, 0);
    analogWrite(pwmL1, constrain( (left + offsetL), 0, 255 ));
  }
  else {
    digitalWrite(pwmL1, 0);
    analogWrite(pwmL2, constrain( (left + offsetL), 0, 255 ));
  }
  if (invert_R == 0) {
    digitalWrite(pwmR1, 0);
    analogWrite(pwmR2, constrain( (right - offsetR), 0, 255 ));
  }
  else {
    digitalWrite(pwmR2, 0);
    analogWrite(pwmR1, constrain( (right - offsetR), 0, 255 ));
  }
}
void Stop() {
  digitalWrite(pwmL1, 1); digitalWrite(pwmL2, 1);
  digitalWrite(pwmR1, 1); digitalWrite(pwmR2, 1);
}

void alignment(int align, bool _invert_L, bool _invert_R, bool align_test) {
  invert_L = _invert_L; invert_R = _invert_R;
  offsetL = constrain(align, -200, 0);
  offsetR = constrain(align, 0, 200);
  if (align_test == 1) {
    forward(255, 255);
    delay(2000);
    Stop();
    while (1);
  }
}

int IR1_avg = 0;
void calibrateIR(int i, int man_value) {
  if (i == 0) IR1_avg = man_value;
  else {
    int IR1_min = 1023, IR1_max = 0;
#ifdef OLED
    u8x8.drawString(0, 0, " TUNING ");
    u8x8.drawString(0, 2, " SENSOR ");
    u8x8.drawString(0, 4, "        ");
    u8x8.drawString(0, 6, " -auto- ");
#endif
    int j = i * 10, k = 0, kk = 0, state = 0;
    while (k != j) {
      k += 1; kk += 1;
      if (kk == 10) {
        kk = 0;
        state = !state;
        LED(state, state, state);
      }
      //auto calibrate
      int IR_cal = analogRead(IR1); delay(2);
      if (IR_cal < IR1_min) {
        IR1_min = IR_cal;
      }
      else if (IR_cal > IR1_max) {
        IR1_max = IR_cal;
      }
      IR_cal = analogRead(IR2); delay(2);
      if (IR_cal < IR1_min) {
        IR1_min = IR_cal;
      }
      else if (IR_cal > IR1_max) {
        IR1_max = IR_cal;
      }
      IR_cal = analogRead(IR3); delay(2);
      if (IR_cal < IR1_min) {
        IR1_min = IR_cal;
      }
      else if (IR_cal > IR1_max) {
        IR1_max = IR_cal;
      }
      IR_cal = analogRead(IR4); delay(2);
      if (IR_cal < IR1_min) {
        IR1_min = IR_cal;
      }
      else if (IR_cal > IR1_max) {
        IR1_max = IR_cal;
      }
      IR_cal = analogRead(IR5); delay(2);
      if (IR_cal < IR1_min) {
        IR1_min = IR_cal;
      }
      else if (IR_cal > IR1_max) {
        IR1_max = IR_cal;
      }
    }
    //auto
    IR1_avg = (IR1_min + IR1_max) / 2;
    LED(0, 0, 0);
    /* #ifdef OLED
        char c_IR[9];
        u8x8.clear();
        u8x8.drawString(0, 0, " TUNING ");
        ( "min" + String(IR1_min) ).toCharArray(c_IR, 9);
        u8x8.drawString(0, 2, c_IR);
        ( "avg" + String(IR1_avg) ).toCharArray(c_IR, 9);
        u8x8.drawString(0, 4, c_IR);
        ( "max" + String(IR1_max) ).toCharArray(c_IR, 9);
        u8x8.drawString(0, 6, c_IR);
        delay(1000);
      #endif */
  }
}

byte IRval = 0b00000000;
const int8_t blackLine = 1, whiteLine = 0;
void update_sensor(int8_t line, int offsetIR) {
  if (analogRead(IR1) < IR1_avg + offsetIR) IRval = IRval | 0b00010000;
  else IRval = IRval & 0b11101111;
  if (analogRead(IR2) < IR1_avg + offsetIR) IRval = IRval | 0b00001000;
  else IRval = IRval & 0b11110111;
  if (analogRead(IR3) < IR1_avg + offsetIR) IRval = IRval | 0b00000100;
  else IRval = IRval & 0b11111011;
  if (analogRead(IR4) < IR1_avg + offsetIR) IRval = IRval | 0b00000010;
  else IRval = IRval & 0b11111101;
  if (analogRead(IR5) < IR1_avg + offsetIR) IRval = IRval | 0b00000001;
  else IRval = IRval & 0b11111110;

  if (line == 1) IRval = ~IRval & 0b00011111; //black line
  else if (line == 0); //white line
}

unsigned long previous_time = 0, current_time = 0, elapsed_time = 0;
double last_error = 0, _error = 0, int_error = 0, rate_error = 0;
double pid(double _setpoint, double _input, int min_out, int max_out) {
  current_time = millis();
  elapsed_time = current_time - previous_time;
  previous_time = current_time;

  _error = _setpoint - _input;
  //int_error += _error * (elapsed_time);
  //int_error = constrain(int_error, -1000, 1000);
  rate_error = (_error - last_error) / (elapsed_time);
  last_error = _error;

  //double _output = KP * _error + KI * int_error + KD * rate_error;
  double _output = KP * _error - KD * rate_error;
  return ( constrain(_output, out_min, out_max) );
}

void followLine(int speedL, int speedR) {
  setPoint = 1000;
  outputVal = pid(setPoint, IR_position, out_min, out_max);

  if (outputVal > 0) { // line is on the LEFT of robot
    forward(speedL - outputVal, speedR);
  }
  else if (outputVal == 0) { // line is on the MIDDLE of robot
    forward(speedL, speedR);
  }
  else if (outputVal < 0) { // line is on the RIGHT of robot
    forward(speedL, speedR + outputVal);
  }
  //LED(0, 1, 0);
}

void junction(int speed_M, int trace_back, int trace_delay, int TYPE, int action, int delay_b4_turn, int turn_speed, int turn_duration, int line, int offsetIR) {
  out_min = -speed_M;
  out_max = speed_M;
  int countIgnore = 0;
  do {
    update_sensor(line, offsetIR);
    if ( (IRval & 0b00011111) != 0b00000000 ) { // robot is ON line
	trackLine:
      IR_position = ( bitRead(IRval, 4) * 0 + bitRead(IRval, 3) * 500 + bitRead(IRval, 2) * 1000 + bitRead(IRval, 1) * 1500 + bitRead(IRval, 0) * 2000 ) / ( bitRead(IRval, 4) + bitRead(IRval, 3) + bitRead(IRval, 2) + bitRead(IRval, 1) + bitRead(IRval, 0) );
      followLine(speed_M, speed_M);
	  
	  // Left junction 
      if ( (IRval & 0b00010111) == 0b00010100 )  { // Left junction 	  
        if (TYPE == 1 && action == 11 ) { // Left junction, turn left
		LED(0, 0, 1);
		  forward(speed_M, speed_M);
          delay(delay_b4_turn);
          turnLeft(turn_speed, turn_speed);
          delay(turn_duration);
		  IR_position = setPoint - 100;
		  LED(0, 0, 0);
		  break;
        }
        else if (TYPE == 1 && action == 22 ) { // left junction, turn right
		LED(1, 0, 0);
		  forward(speed_M, speed_M);
          delay(delay_b4_turn);
          turnRight(turn_speed, turn_speed);
          delay(turn_duration);
		  IR_position = setPoint + 100;
		  LED(0, 0, 0);
		  break;
        }
        else if (TYPE == 1 && action == 33 ) { // left junction, ignore
		LED(1, 1, 1);
		followLine(speed_M, speed_M);
          while ( (IRval & 0b00010111) == 0b00010100 ) {
            update_sensor(line, offsetIR);            
          }
          break;
        }
      }
      
	  // Middle junction 
	//else if ( (IRval & 0b00010010) == 0b00010010 || (IRval & 0b00010001) == 0b00010001 || (IRval & 0b00001001) == 0b00001001 ) { // Middle junction
	else if ( (IRval & 0b00010101) == 0b00010101 ) {
        if (TYPE == 3 && action == 11 ) { // Middle junction, turn left
		LED(0, 0, 1);
		  forward(speed_M, speed_M);
          delay(delay_b4_turn);
          turnLeft(turn_speed, turn_speed);
          delay(turn_duration);
		  IR_position = setPoint - 100;
		  LED(0, 0, 0);
		  break;
        }
        else if (TYPE == 3 && action == 22 ) { // Middle junction, turn right
		LED(1, 0, 0);
		  forward(speed_M, speed_M);
          delay(delay_b4_turn);
          turnRight(turn_speed, turn_speed);
          delay(turn_duration);
		  IR_position = setPoint + 100;
		  LED(0, 0, 0);
		  break;
        }
        else if (TYPE == 3 && action == 33 ) { // Middle junction, ignore
		LED(1, 1, 1);
		followLine(speed_M, speed_M);
          //while ( (IRval & 0b00010010) == 0b00010010 || (IRval & 0b00010001) == 0b00010001 || (IRval & 0b00001001) == 0b00001001 ) {
          while ( (IRval & 0b00010101) == 0b00010101 ){
			update_sensor(line, offsetIR);
          }
          break;
        }
      }

      // Right junction
      else if ( (IRval & 0b00011101) == 0b00000101 )  { // Right junction
        if (TYPE == 2 && action == 11) { // Right junction, turn left
		LED(0, 0, 1);
		  forward(speed_M, speed_M);
          delay(delay_b4_turn);
          turnLeft(turn_speed, turn_speed);
          delay(turn_duration);
		  IR_position = setPoint - 100;
		  LED(0, 0, 0);
		  break;
        }
        else if (TYPE == 2 && action == 22 ) { // Right junction, turn right
		LED(1, 0, 0);
		  forward(speed_M, speed_M);
          delay(delay_b4_turn);
          turnRight(turn_speed, turn_speed);
          delay(turn_duration);
		  IR_position = setPoint + 100;
		  LED(0, 0, 0);
		  break;
        }
        else if (TYPE == 2 && action == 33 ) { // Right junction, ignore
		LED(1, 1, 1);
		followLine(speed_M, speed_M);
          while ( (IRval & 0b00011101) == 0b00000101 ) {
            update_sensor(line, offsetIR);
          }
          break;
        }
      }

      else if (TYPE == 0 && action == 11) { // dont care, turn left
        if (countIgnore == delay_b4_turn) {
		LED(0, 0, 1);
          turnLeft(turn_speed, turn_speed);
          delay(turn_duration);
          IR_position = setPoint - 100;
		  LED(0, 0, 0);
          break;
        }
        countIgnore += 1;
      }
      else if (TYPE == 0 && action == 22) { // dont care, turn right
        if (countIgnore == delay_b4_turn) {
		LED(1, 0, 0);
          turnRight(turn_speed, turn_speed);
          delay(turn_duration);
          IR_position = setPoint + 100;
		  LED(0, 0, 0);
          break;
        }
        countIgnore += 1;
      }
      else if (TYPE == 0 && action == 33 && delay_b4_turn != -1) { // dont care, ignore
        countIgnore += 1;
		if (countIgnore == delay_b4_turn) {
          break;
        }
      }
      else if (TYPE == 4 && action == 33 && delay_b4_turn == -1) { // dont care, loop forever
        // loop forever
      }
    }
    else { // robot is OFF line (trace back)
      int countTrace = 0;
      while (countTrace < trace_delay) {
        countTrace++;
        update_sensor(line, offsetIR);
        if ( (IRval & 0b00011111) != 0b00000000 )
          goto trackLine;
      }
      if (countTrace >= trace_delay) {
        if (IR_position < setPoint) { // last position is on the LEFT before lost
          //LED(0, 0, 1);
          turnLeft(turn_speed, turn_speed);
        }
        else if (IR_position == setPoint) { // last position is on the LEFT before lost
		  if (trace_back == 1){
			  //LED(0, 0, 1);
			  turnLeft(turn_speed, turn_speed);
		  }
		  else if (trace_back == 2) {
			  //LED(1, 0, 0);
		      turnRight(turn_speed, turn_speed);
		  }
        }
        else { // last position is on the RIGHT before lost
          //LED(1, 0, 0);
          turnRight(turn_speed, turn_speed);
        }
      }
    }
  }
  while (1);
}

void obstacle(int speed_M, int trace_back, unsigned int distance, int action, int delay_b4_turn, int turn_speed, int turn_duration, int line, int offsetIR) {
  out_min = -speed_M;
  out_max = speed_M;
  do {
    update_sensor(line, offsetIR);
    if (IRval != 0b00000000) { // robot is ON line
      IR_position = (bitRead(IRval, 4) * 0 + bitRead(IRval, 3) * 500 + bitRead(IRval, 2) * 1000 + bitRead(IRval, 1) * 1500 + bitRead(IRval, 0) * 2000) / (bitRead(IRval, 4) + bitRead(IRval, 3) + bitRead(IRval, 2) + bitRead(IRval, 1) + bitRead(IRval, 0));
      followLine(speed_M, speed_M);

      if (hcsr04.distanceInMillimeters() <= distance && action == 11) {
        Stop(); delay(delay_b4_turn);
        turnLeft(turn_speed, turn_speed);
        delay(turn_duration);
        Stop();
      }
      else if (hcsr04.distanceInMillimeters() <= distance && action == 22) {
        Stop(); delay(delay_b4_turn);
        turnRight(turn_speed, turn_speed);
        delay(turn_duration);
        Stop();
      }
    }
    else { // robot is OFF line (trace back)
      if (IR_position < setPoint) { // last position is on the LEFT before lost
        LED(0, 0, 1);
        if (trace_back != 0)
          turnLeft(trace_back, trace_back);
        else
          forward(0, speed_M);

        while ( (IRval & 0b00011000) == 0b00000000 )
          update_sensor(line, offsetIR);
      }
      else { // last position is on the RIGHT before lost
        LED(1, 0, 0);
        if (trace_back != 0)
          turnRight(trace_back, trace_back);
        else
          forward(speed_M, 0);

        while ( (IRval & 0b00000011) == 0b00000000 )
          update_sensor(line, offsetIR);
      }
    }
  }
  while (1);
}

void back_to_line (int speedL, int speedR, int line, int offsetIR) { // while going back to line
  while (1) {
    update_sensor(line, offsetIR);
    if (IRval != 0b00000000) { // back to line
      Stop(); delay(50);
      break;
    }
    else { // approaching line
      forward(speedL, speedR);
    }
  }
}

unsigned long start_ms;
void bot_setup(int calibrate_time, int manual_value) {
  //****************** OLED setup ******************//
#ifdef OLED
  u8x8.begin();
  u8x8.setPowerSave(0);
  //  u8x8.setFont(u8x8_font_lucasarts_scumm_subtitle_r_2x2_r);
  u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
#endif
  //****************** OLED setup ******************//

  //****************** robot setup ******************//
  pinMode(pwmL1, OUTPUT);  digitalWrite(pwmL1, LOW);
  pinMode(pwmL2, OUTPUT);  digitalWrite(pwmL2, LOW);
  pinMode(pwmR1, OUTPUT);  digitalWrite(pwmR1, LOW);
  pinMode(pwmR2, OUTPUT);  digitalWrite(pwmR2, LOW);
  pinMode(led_r, OUTPUT); digitalWrite(led_r, LOW);
  pinMode(led_y, OUTPUT); digitalWrite(led_y, LOW);
  pinMode(led_g, OUTPUT); digitalWrite(led_g, LOW);
  pinMode(battMon_en, OUTPUT); digitalWrite(battMon_en, HIGH);
  //****************** robot setup ******************//

  //****************** check battery ******************//
#ifdef OLED
  u8x8.drawString(0, 0, "BATTERY ");
  u8x8.drawString(0, 2, "        ");
  u8x8.drawString(0, 4, "CHECKING");
  u8x8.drawString(0, 6, "        ");
#endif
  delay(100);
  float battVolt = 0;
  for (int i = 0; i < 100; i++) {
    battVolt += analogRead(battMon_pin) * 5.00 / 1023.00;
	delay(10);
  }
  int battPercent = (battVolt-282) / (296-282) * 100;
  battPercent = constrain(battPercent, 0, 99);
  char c_battPercent[9];
  ("  " + String(battPercent) + "%   ").toCharArray(c_battPercent, 9);
  digitalWrite(battMon_en, LOW);
	
  if (battPercent > 70) {
#ifdef OLED
    u8x8.drawString(0, 4, c_battPercent);
    u8x8.drawString(0, 6, "  Full  ");
#endif
    LED(0, 0, 1);
  }
  else if (battPercent <= 70 && battPercent > 50) {
#ifdef OLED
    u8x8.drawString(0, 4, c_battPercent);
    u8x8.drawString(0, 6, "  Good  ");
#endif
    LED(0, 1, 0);
  }
  else if (battPercent <= 50 && battPercent > 30) {
#ifdef OLED
    u8x8.drawString(0, 4, c_battPercent);
    u8x8.drawString(0, 6, "  LOW!  ");
#endif
    LED(1, 0, 0);
  }
  else {
#ifdef OLED
    u8x8.drawString(0, 4, " PLEASE ");
    u8x8.drawString(0, 6, " CHANGE ");
#endif
    while (1) {
      LED(1, 0, 0);
      delay(100);
      LED(0, 0, 0);
      delay(150);
    }
  }
  delay(1000);
  //****************** check battery ******************//

  calibrateIR(calibrate_time, manual_value);

  //****************** tell user ready to go ******************//
#ifdef OLED
  u8x8.clear();
  u8x8.drawString(3, 0, "READY");
  u8x8.setFont(u8x8_font_courB24_3x4_n);
  u8x8.drawString(1, 3, "  1");
  delay(1000);
  u8x8.drawString(1, 3, "  2");
  delay(1000);
  u8x8.clear();
  u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
  u8x8.drawString(3, 0, "TIMER");
  u8x8.setFont(u8x8_font_courB24_3x4_n);
  u8x8.drawString(1, 3, "00.00");
#endif
  //****************** tell user ready to go ******************//

  start_ms = millis();
}

void display_finishTime(bool dispFlip) {
  //****************** display finish time ******************//
  //long stop_ms = millis();
  long ms = millis() - start_ms;
  int second = 0;// minute = 0;
  while (ms >= 1000) {
    second += 1;
    ms -= 1000;
  }
  //  while (second >= 60) {
  //    minute += 1;
  //    second -= 60;
  //  }
#ifdef OLED
  String Str = "";
  //  if (minute < 10)
  //    Str += "0" + String(minute) + ":";
  //  else
  //    Str += String(minute) + ":";
  if (second < 10)
    Str += "0" + String(second) + ".";
  else
    Str += String(second) + ".";
  if (ms < 10)
    Str += "0" + String(ms);
  else
    Str += String(ms);
  char c_Time[6];
  Str.toCharArray(c_Time, 6);

  if (dispFlip == 1) {
    u8x8.clear();
    u8x8.setFlipMode(1);
  }
  u8x8.clear();
  u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
  u8x8.drawString(0, 0, "!FINISH!");
  u8x8.setFont(u8x8_font_courB24_3x4_n);
  u8x8.drawString(1, 3, c_Time);
#endif
  //****************** display finish time ******************//
  while (1);
}
