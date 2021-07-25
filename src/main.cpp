#include "Arduino.h"
#include <avr/interrupt.h>

#define INPUT_CAPTURE_PIN   8
#define MEASURE_PERIOD_MS   103

#define CLK_PRESCALER       IC_CLK_PRESCALER_64

uint32_t t_acum    = 0;     
uint32_t prev_edge = 0;
uint32_t new_edge  = 0;
float us_acum      = .0;
float freq         = .0;

uint32_t pulse_ctr = 0;
uint32_t prev_ms   = 0;
uint32_t cur_ms    = 0;

uint8_t overflows  = 0;

/* Timer prescale configuration
  CS12 CS11 CS10
    0    0    1  : /1    No prescaler, CPU clock,  62.5 ns per tick
    0    1    0  : /8    prescaler,                0.5  us per tick
    0    1    1  : /64   prescaler                 4    us per tick
    1    0    0  : /256  prescaler                 16   us per tick
    1    0    1  : /1024 prescaler                 64   us per tick
*/
typedef enum {
    IC_CLK_PRESCALER_NONE = 1,
    IC_CLK_PRESCALER_8,
    IC_CLK_PRESCALER_64,
    IC_CLK_PRESCALER_256,
    IC_CLK_PRESCALER_1024
} ic_clk_prescaler_t;

typedef struct {
    ic_clk_prescaler_t prescaler;
    float us_per_tick;
} prescaler_lookup_t;

static const prescaler_lookup_t prescaler_lookup[] = {
    { .prescaler = IC_CLK_PRESCALER_NONE, .us_per_tick = 0.0625 },
    { .prescaler = IC_CLK_PRESCALER_8,    .us_per_tick = 0.5    },
    { .prescaler = IC_CLK_PRESCALER_64,   .us_per_tick = 4      },
    { .prescaler = IC_CLK_PRESCALER_256,  .us_per_tick = 16     },
    { .prescaler = IC_CLK_PRESCALER_1024, .us_per_tick = 64     }
};

float us_per_tick(ic_clk_prescaler_t prescaler) {
    return prescaler_lookup[prescaler - 1].us_per_tick;
}

void init_capture(void) {
    // Input Capture setup
    // ICNC1: Enable Input Capture Noise Canceler
    // ICES1: = 1 for trigger on rising edge
    TCCR1A = 0;
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | CLK_PRESCALER;
    TCCR1C = 0;

    // Interrupt setup
    // ICIE1: Input capture
    // TOIE1: Timer1 overflow
    TIFR1  = (1 << ICF1) | (1 << TOV1);  // clear pending interrupts
    TIMSK1 = (1 << ICIE1) | (1 << TOIE1); // enable interupts

    // Set up the Input Capture pin, ICP1, Arduino Uno pin 8
    pinMode(INPUT_CAPTURE_PIN, INPUT);
    digitalWrite(INPUT_CAPTURE_PIN, LOW); // floating may have 50 Hz noise on it.
    //digitalWrite(INPUT_CAPTURE_PIN, HIGH); // or enable the pullup
}

// Interrupt capture handler
//
ISR(TIMER1_CAPT_vect) {
    prev_edge = new_edge;
    new_edge = ICR1;
    
    if (!(TIFR1 & _BV(TOV1))) {
        t_acum += (new_edge - prev_edge);
        pulse_ctr++;
    }
}

ISR(TIMER1_OVF_vect) {
    overflows++;
}

void setup() {
    Serial.begin(115200);
    init_capture();
}

void loop() {
    cur_ms = millis();

    if (cur_ms - prev_ms >= MEASURE_PERIOD_MS) {
        prev_ms = cur_ms;

        us_acum = float(t_acum) * us_per_tick(CLK_PRESCALER);
        freq = 1e6/us_acum; // us -> s
        freq *= pulse_ctr;

        Serial.print(pulse_ctr); Serial.println(" pulses");
        Serial.print(overflows); Serial.println(" overflows");
        
        pulse_ctr = 0;
        t_acum = 0;
        overflows = 0;

        if (!isnan(freq)) {
            Serial.print(freq); Serial.println(" Hz");
        }
    }
}