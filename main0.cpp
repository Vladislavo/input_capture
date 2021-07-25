#include <Arduino.h>

#define INPUT_PIN           3

#define MEASURE_PERIOD_MS   100

uint8_t pin_state      = 1;
uint8_t pin_state_prev = 1;

uint32_t t         = 0;
uint32_t t_acum    = 0;

uint32_t prev_edge = 0;
uint32_t new_edge  = 0;
float freq         = .0;

uint32_t pulse_ctr = 0;
uint32_t prev_ms   = 0;
uint32_t cur_ms    = 0;

void capture_edge();

void setup() {
  Serial.begin(115200);

  pinMode(INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), capture_edge, RISING);
}

void loop() {
  // pin_state = digitalRead(INPUT_PIN);
  // if (pin_state_prev != pin_state) {
  //   pin_state_prev = pin_state;
  //   Serial.print("change ");
  //   Serial.println(pin_state);
  //   t = new_edge - prev_edge;
  //   freq = 1000/float(t); // ms -> s
  //   Serial.print(freq); Serial.println(" Hz");
  // }

  cur_ms = millis();
  if (cur_ms - prev_ms >= MEASURE_PERIOD_MS) {
    prev_ms = cur_ms;

    freq = 1e6/float(t_acum); // us -> s
    freq *= pulse_ctr;
    
    pulse_ctr = 0;
    t_acum = 0;

    if (!isnan(freq)) {
      Serial.print(freq); Serial.println(" Hz");
    }
  }
}

void capture_edge() {
  prev_edge = new_edge;
  new_edge = micros();
  t_acum += (new_edge - prev_edge);
  pulse_ctr++;
}