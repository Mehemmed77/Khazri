#include <secrets.h>
#include <WiFi.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>

#define INIT_WIND_SPEED 5
#define INIT_WIND_DIRECTION 180
#define SPEED_STEP 0.2f   // m/s per tick
#define DIR_STEP   3.0f   // degrees per tick
#define NODE_COUNT 16

typedef struct {
  float wind_speed; //m/s
  float wind_direction; // degree
  uint64_t timestamp; // milliseconds
  uint8_t node_id; // count
} WindRecord;

hw_timer_t *timer = NULL;
volatile uint8_t sample_count = 0;
volatile bool tick = false;

float speed[NODE_COUNT];
float dir[NODE_COUNT];

float sum_speed[NODE_COUNT];
float sum_sin[NODE_COUNT];
float sum_cos[NODE_COUNT];

WindRecord records[NODE_COUNT];

void ARDUINO_ISR_ATTR onTimer() {
  // Simulating Sampling
  tick = true;
}

float rand_unit() {// returns a value in [-1.0, +1.0]
  return (esp_random() / (float)UINT32_MAX) * 2.0f - 1.0f;
}

uint64_t now_ms() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000;
}

void sample_wind_values() {
  for(int i = 0; i < NODE_COUNT; i++) {
    speed[i] += SPEED_STEP * rand_unit();
    if (speed[i] < 0) speed[i] = 0;

    dir[i] += DIR_STEP * rand_unit();
    if (dir[i] >= 360) dir[i] -= 360;
    if (dir[i] < 0)    dir[i] += 360;

    sum_speed[i] += speed[i];

    float rad = dir[i] * PI / 180.0f;   // degrees → radians
    sum_sin[i] += sinf(rad);
    sum_cos[i] += cosf(rad);
  }
}

void print_records() {
  for(int i = 0; i < NODE_COUNT; i++) {
    Serial.printf("node=%u ts=%llu speed=%.2f dir=%.1f\n",
                  records[i].node_id, records[i].timestamp,
                  records[i].wind_speed, records[i].wind_direction);
  }
}

void send_and_reset() {
  // simulating sending
  for(int i = 0; i < NODE_COUNT; i++) {
     records[i].wind_speed = sum_speed[i] / 4;

     float d = atan2f(sum_sin[i], sum_cos[i]) * 180.0f / PI;  // angle of the summed arrow
     records[i].wind_direction = (d < 0) ? d + 360 : d;       // atan2 gives -180..180 → 0..360

     records[i].timestamp = now_ms();
     records[i].node_id = i + 1;
  }

  print_records();

  // RESETING
  sample_count = 0;
  memset(sum_speed, 0, sizeof(sum_speed));
  memset(sum_sin,   0, sizeof(sum_sin));
  memset(sum_cos,   0, sizeof(sum_cos));
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }

  configTime(0, 0, "pool.ntp.org");               // UTC, no DST offset
  while (time(NULL) < 1700000000) { delay(200); } // wait until synced

  Serial.println("\nTime synced");

  for (int i = 0; i < NODE_COUNT; i++) {
    speed[i] = INIT_WIND_SPEED;
    dir[i]   = INIT_WIND_DIRECTION;
  }

  timer = timerBegin(1000);

  timerAttachInterrupt(timer, &onTimer);

  timerAlarm(timer, 250, true, 0);
}

void loop() {
  if(!tick) return;
  tick = false;
  sample_count++;

  sample_wind_values();

  if(sample_count >= 4) send_and_reset();
}
