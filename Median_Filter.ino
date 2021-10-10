// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.1 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float dist_raw_list3[3];
float dist_raw_list10[10];
float dist_raw_list30[30];
int dist_index_3 = 0;
int dist_index_10 = 0;
int dist_index_30 = 0;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

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

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  if (dist_index_3 < 3) {
    dist_raw_list3[dist_index_3] = dist_raw;
    dist_index_3 += 1;
  }
  else {
    dist_raw_list3[0] = dist_raw;
    dist_index_3 = 1;
  }
  if (dist_index_10 < 10) {
    dist_raw_list10[dist_index_10] = dist_raw;
    dist_index_10 += 1;
  }
  else {
    dist_raw_list10[0] = dist_raw;
    dist_index_10 = 1;
  }
  if (dist_index_30 < 30) {
    dist_raw_list30[dist_index_30] = dist_raw;
    dist_index_30 += 1;
  }
  else {
    dist_raw_list30[0] = dist_raw;
    dist_index_30 = 1;
  }

  sort(dist_raw_list3, 3);
  sort(dist_raw_list10, 10);
  sort(dist_raw_list30, 30);

  float dist_mid_3 = dist_raw_list3[2];
  float dist_mid_10 = (dist_raw_list10[5] + dist_raw_list10[6]) / 2;
  float dist_mid_30 = (dist_raw_list30[15] + dist_raw_list30[16]) / 2;

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median3: ");
  Serial.print(map(dist_mid_3,0,400,100,500));
  Serial.print(",");
  Serial.print("median10: ");
  Serial.print(map(dist_mid_10,0,400,100,500));
  Serial.print(",");
  Serial.print("median30: ");
  Serial.print(map(dist_mid_30,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}
