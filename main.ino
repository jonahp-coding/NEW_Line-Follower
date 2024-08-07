
// make a class for motor and functions
// Use #define for variables that do not change...

uint8_t input_1_pin = 13;
uint8_t input_2_pin = 12;
uint8_t input_3_pin = 11;
uint8_t input_4_pin = 10;

uint8_t motor_1_pwm_pin = 6;
uint8_t motor_2_pwm_pin = 5;

uint8_t ir_1_pin = 8;
uint8_t ir_2_pin = 7;
uint8_t ir_3_pin = 9;

uint8_t motor_speed_pot = A0;
uint8_t steering_speed_pot = A1;

bool motor_1_dir = true;
bool motor_2_dir = true;

bool ir_1_state = true;
bool ir_2_state = true;
bool ir_3_state = true;

uint8_t default_speed = 80;
uint8_t motor_speed = default_speed;
uint8_t steering_speed = default_speed * 0.5;
int8_t direction = 0;

uint8_t R_target_speed = motor_speed; // 'motor_speed'
uint16_t R_rate = 50; // how long motor 'R' should take to reach 'R_target_speed'
uint8_t R_speed; // instantaneous speed of motor 'R'
uint16_t R_acceleration_timestamp; // instantaneous timestamp of motor 'R' acceleration
uint16_t R_initial_timestamp; // timestamp at beginning of motor 'R' acceleration
uint8_t R_initial_speed = default_speed; // the intial speed of motor 'R' at beginning of acceleration

uint8_t L_target_speed = motor_speed; // 'motor_speed'
uint16_t L_rate = 50; // how long motor 'L' should take to reach 'L_target_speed'
uint8_t L_speed; // instantaneous speed of motor 'L'
uint16_t L_acceleration_timestamp; // instantaneous timestamp of motor 'L' acceleration
uint16_t L_initial_timestamp; // timestamp at beginning of motor 'L' acceleration
uint8_t L_initial_speed = default_speed; // the intial speed of motor 'L' at beginning of acceleration

uint16_t current_millis = millis();

void setup() {

  Serial.begin(115200);

  // ...
  pinMode(input_1_pin, OUTPUT);
  pinMode(input_2_pin, OUTPUT);
  pinMode(input_3_pin, OUTPUT);
  pinMode(input_4_pin, OUTPUT);

  pinMode(motor_1_pwm_pin, OUTPUT);
  pinMode(motor_2_pwm_pin, OUTPUT);

  pinMode(ir_1_pin, INPUT);
  pinMode(ir_2_pin, INPUT);
  pinMode(ir_3_pin, INPUT);

  // Intial settings
  motor_1_set_pwm(0);
  motor_2_set_pwm(0);
  
  motor_1_reverse_dir(motor_1_dir);
  motor_2_reverse_dir(motor_2_dir);
}

void loop() {
  
  motor_speed = map(analogRead(motor_speed_pot), 0, 1023, 0, 255);
  steering_speed = map(analogRead(steering_speed_pot), 0, 1023, 0, 255);

  ir_1_state = digitalRead(ir_1_pin);
  ir_2_state = digitalRead(ir_2_pin);
  ir_3_state = digitalRead(ir_3_pin);

  if (ir_1_state == false && ir_2_state == true) {
    if (direction != -1) {
      direction = -1;
      
      R_target_speed = motor_speed;
      R_initial_timestamp = current_millis;

      L_target_speed = steering_speed;
      L_initial_timestamp = current_millis;
    }
  }
  
  if (ir_2_state == false && ir_1_state == true) {
    if (direction != 1) {
      direction = 1;
      
      R_target_speed = steering_speed;
      R_initial_timestamp = current_millis;

      L_target_speed = motor_speed;
      L_initial_timestamp = current_millis;
    }
  }

  if ((ir_3_state == false && (ir_1_state == true) && ir_2_state == true) || (ir_1_state == true && ir_2_state == true)) {
    if (direction != 0) {
      direction = 0;

      R_target_speed = motor_speed;
      R_initial_timestamp = current_millis;

      L_target_speed = motor_speed;
      L_initial_timestamp = current_millis;
    }
  } /* else if (ir_3_state == false && ir_1_state == true && ir_2_state == true) {
    // Keep the motor equal speed, travels relatively straight

    R_target_speed = 0;
    R_initial_timestamp = current_millis;

    L_target_speed = 0;
    L_initial_timestamp = current_millis;
  } */

  // Motor Control Logic 
  if (R_target_speed == 0) {
    motor_1_set_pwm(0);
    Serial.println("R_Motor: Disabled");
  } else if (current_millis - R_initial_timestamp < R_rate) {
    motor_1_set_pwm(R_initial_speed);
    R_speed = map(current_millis, R_initial_timestamp, (R_initial_timestamp + R_rate), R_initial_speed, R_target_speed);
    R_acceleration_timestamp = current_millis - R_initial_timestamp;

    motor_1_set_pwm(R_speed);

    Serial.print("R_speed: ");
    Serial.println(R_speed);
    Serial.print("R_acceleration_timestamp(ms): ");
    Serial.println(R_acceleration_timestamp);
  }

  if (L_target_speed == 0) {
    motor_2_set_pwm(0);
    Serial.println("L_Motor: Disabled");
  } else if (current_millis - L_initial_timestamp < L_rate) {
    motor_2_set_pwm(L_initial_speed);
    L_speed = map(current_millis, L_initial_timestamp, (L_initial_timestamp + L_rate), L_initial_speed, L_target_speed);
    L_acceleration_timestamp = current_millis - L_initial_timestamp;

    motor_2_set_pwm(L_speed);

    Serial.print("L_speed: ");
    Serial.println(L_speed);
    Serial.print("L_acceleration_timestamp(ms): ");
    Serial.println(L_acceleration_timestamp);
  }

  Serial.println();

  // end of Motor Control Logic

  Serial.print("ir_1_state: ");
  Serial.println(ir_1_state);
  Serial.print("ir_2_state: ");
  Serial.println(ir_2_state);
  Serial.print("ir_3_state: ");
  Serial.println(ir_3_state);

  Serial.print("\nmotor_speed: ");
  Serial.println(motor_speed);
  Serial.print("steering_speed: ");
  Serial.println(steering_speed);

  Serial.println("-----------------------------");

  current_millis = millis();
  delay(10);

}

void motor_1_reverse_dir(bool reverse) {
  if (reverse) {
    digitalWrite(input_1_pin, LOW);
    digitalWrite(input_2_pin, HIGH);
  } else {
    digitalWrite(input_1_pin, HIGH);
    digitalWrite(input_2_pin, LOW);
  }
}

void motor_2_reverse_dir(bool reverse) {
  if (reverse) {
    digitalWrite(input_3_pin, LOW);
    digitalWrite(input_4_pin, HIGH);
  } else {
    digitalWrite(input_3_pin, HIGH);
    digitalWrite(input_4_pin, LOW);
  }
}

void motor_1_set_pwm(uint8_t val) {
  analogWrite(motor_1_pwm_pin, val);
}

void motor_2_set_pwm(uint8_t val) {
  analogWrite(motor_2_pwm_pin, val);
}
