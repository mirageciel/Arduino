#include <Servo.h>

// Configurable parameters

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

//FrameWork setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.5 // EMA filter is abled

// Servo range
#define _DUTY_MIN 1150
#define _DUTY_NEU 1300
#define _DUTY_MAX 1380

// Servo speed control
#define _SERVO_ANGLE 30 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 30 // servo speed limit (deg/sec)

// Event periods
#define _INTERVAL_DIST 10 // distance sensor interval (ms)
#define _INTERVAL_SERVO 10 // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)

// PID parameters
#define _KP 0.3 // proportional gain *****
#define _KI 0
#define _KD 0

//global variables
float alpha = _DIST_ALPHA;
float dist_raw_list[30];
int dist_index = 0;

int a, b; // unit: mm

// Servo instance
Servo myservo;

// Distance sensor
float dist_target = _DIST_TARGET; // location to send the ball
float dist_raw, dist_mid;
// float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
              last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void sort(float array[], int n) {
  int i, j;
  float save;
  for (i=0; i<n-1; i++) {
    for (j=i+1; j<n; j++) {
      if (array[i] > array[j]) {
        save = array[i];
        array[i] = array[j];
        array[j] = save;
      }
    }
  }
}

float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  float dist_cali = 100 + 300.0 / (b-a) * (val - a);
  return dist_cali;
}

float ir_distance_filtered(void) { // return value unit: mm
  // for now, just use ir_distance() without noise filter
  dist_raw = ir_distance();
  if (dist_index < 30) {
    dist_raw_list[dist_index] = dist_raw;
    dist_index += 1;
  }
  else {
    dist_raw_list[0] = dist_raw;
    dist_index = 1;
  }
  sort(dist_raw_list, 30);
  float dist_mid = (dist_raw_list[15] + dist_raw_list[16]) / 2;
  return dist_mid;
}

void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO);

  // initialize global variables
  a = 68;
  b = 310;
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(57600);

  // convert angle speed into duty change per interval
  duty_chg_per_interval = 5;
  //duty_chg_per_interval = (float)(_DUTY_MAX-_DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
}

void loop() {
  // Event generator
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }

  // Event handlers
  if (event_dist) {
    event_dist = false;
    // get a distance reading from the distance sensor
    dist_raw = ir_distance();
    dist_mid = ir_distance_filtered();

    // PID control logic
    error_curr = dist_target - dist_mid;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    control = pterm;

    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; // 255mm에서의 각도

    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    // update error_prev
    error_prev = error_curr;

    // keep duty_target value within the range of
    // [_DUTY_MIN, _DUTY_MAX]
  }

  if (event_servo) {
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    event_servo = false;
    float angle_curr = myservo.read();
    duty_curr = 1000 + 5.55 * angle_curr;
    if (duty_curr != duty_target) duty_curr += (duty_chg_per_interval * control);
    else duty_curr = duty_target;
    if (duty_curr > _DUTY_MAX) duty_curr = _DUTY_MAX;
    else if (duty_curr < _DUTY_MIN) duty_curr = _DUTY_MIN;

    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_mid);
    Serial.print(", pterm:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(", duty_target:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(", duty_curr:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(", Min:100, Low:200, dist_target:255, High:310, Max:410");
  }
}
