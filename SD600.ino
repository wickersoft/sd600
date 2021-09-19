// CONSTANTS
#define MIN_VOLTS 75L
#define MAX_VOLTS 360L
#define EMERGENCY_STOP_VOLTS 400L
#define CHARGE_PER_CYCLE 3L
#define MIN_CHARGE_MS 0L
#define MAX_CHARGE_MS 5000L
#define READY_BEEP_MS 100L
#define DUMP_FLASH_DELAY_MS 2000L
#define FLASH_IDLE_DELAY_MS 100L
#define MIN_DIM_MICROS 4000L
#define MAX_DIM_MICROS 9400L
#define TRIAC_FIRE_MICROS 200L
#define OVERCHARGE_THRESHOLD_EXP(t, x) (x > t + 4L * CHARGE_PER_CYCLE && x > (t * 4L) / 3L)
#define UNDERCHARGE_THRESHOLD_EXP(t, x) (x < t - 2L * CHARGE_PER_CYCLE && x < (t * 6L) / 7L)
#define REFRESH_CHARGE_THRESHOLD_EXP(t, x) (x < t - CHARGE_PER_CYCLE)

// INPUTS
#define DIM 16
#define GMGKG 15
#define SPK 14
#define LAMP_MO1 18
#define LAMP_MO2 17
#define INTENS A6
#define GMG 2 // INT0
#define TRG 3 // INT1
#define HV_SENS A7
#define AC_DET0 6 // AIN0
#define AC_DET1 7 // AIN1
#define DEBUG 19

// OUTPUTS
#define CHARGE 10
#define FLASH 8
#define DIM_TRIAC 9
#define LED 20
#define BEEP 21

// I/O MACROS
#define DIM_TRIAC_ON PORTB |= (1<<PB1)
#define DIM_TRIAC_OFF PORTB &= ~(1<<PB1)
#define FLASH_ON PORTB |= (1<<PB0)
#define FLASH_OFF PORTB &= ~(1<<PB0)
#define CHARGE_ON PORTB |= (1<<PB2)
#define CHARGE_OFF PORTB &= ~(1<<PB2)
#define LED_ON PORTB |= (1<<PB6)
#define LED_OFF PORTB &= ~(1<<PB6)
#define BEEP_ON PORTB |= (1<<PB7)
#define BEEP_OFF PORTB &= ~(1<<PB7)
#define DEBUG_ON PORTC |= (1<<PC5)
#define DEBUG_OFF PORTC &= ~(1<<PC5)

#define READ_DIM ((PINC & (1<<PC2)) != 0)
#define READ_SPK ((PINC & (1<<PC0)) != 0)
#define READ_LAMP_MO1 ((PINC & (1<<PC4)) != 0)
#define READ_LAMP_MO2 ((PINC & (1<<PC3)) != 0)

volatile enum charge_state {IDLE, CHARGING, READY, FLASHING, FAULT} state;
volatile int current_intens = 0;
volatile int current_volts = 0;
volatile long idle_timeout = 0;
volatile long charge_start = 0;
volatile long charge_end = 0;
volatile long overcharge_millis = 0;

void setup() {
  cli();
  DDRB = 0b11000111; // Beep, LED, Charge, Dim_Triac, Flash
  DDRC = 0b00100000; // Debug
  setup_tint();
  setup_acint();
  setup_flash_int();
  idle_timeout = millis() + 1000; // Wait one second before charging
  state = IDLE; // Do not charge until firmware has finished booting
  CHARGE_OFF;
  FLASH_OFF;
  BEEP_OFF;
  LED_OFF;
  sei();
}

void loop() {
  current_volts = analogRead(HV_SENS) >> 1;
  if (current_volts >= EMERGENCY_STOP_VOLTS) {
    state = FAULT;
  }

  current_intens = analogRead(INTENS);
  int target_volts = MIN_VOLTS + (MAX_VOLTS - MIN_VOLTS) * current_intens / 1023L;
  if (target_volts < MIN_VOLTS) {
    target_volts = MIN_VOLTS;
  }

  if (state != READY) {
    disarm_flash_int();
  }

  switch (state) {
    case IDLE:
      // Steady state: Bank idle, LED off, no beep
      CHARGE_OFF;
      FLASH_OFF;
      BEEP_OFF;
      LED_OFF;

      // Transition check: Idle period expired, ready to charge
      if (millis() >= idle_timeout) {
        // Bank is now safe to charge
        charge_start = millis() + MIN_CHARGE_MS;
        charge_end = millis() + MAX_CHARGE_MS;
        state = CHARGING;
        break;
      }
      break;
    case CHARGING:
      // Steady state: Bank charging, LED off, no beep
      CHARGE_ON;
      FLASH_OFF;
      BEEP_OFF;

      // Transition check: Charging curve valid or invalid
      if (current_volts >= target_volts) {
        CHARGE_OFF;
        if (millis() >= charge_start) {
          // Healthy amount of time has passed --> Ready to flash
          if (!READ_SPK) {
            BEEP_ON;
            delay(READY_BEEP_MS);
          }
          overcharge_millis = 0;
          state = READY;
        } else {
          // Voltage reached too fast --> Bank fault
          //state = FAULT;
        }
      } else if (millis() >= charge_end) {
        // Took too long to charge --> Bank charging circuit fault
        CHARGE_OFF;
        //state = FAULT;
      }
      break;
    case READY:
      // Steady state: Bank idle, LED on, no beep
      CHARGE_OFF;
      FLASH_OFF;
      BEEP_OFF;
      LED_ON;
      // Maintain voltage or transition if target_volts changes
      if (OVERCHARGE_THRESHOLD_EXP(target_volts, current_volts)) {
        if (overcharge_millis == 0) {
          overcharge_millis = millis();
        } else if (millis() - overcharge_millis > DUMP_FLASH_DELAY_MS) {
          // Flash
          DEBUG_ON;
          DEBUG_OFF;
          cli();
          flash();
          sei();
        }
      } else {
        overcharge_millis = 0;
      }
      if (UNDERCHARGE_THRESHOLD_EXP(target_volts, current_volts)) {
        // Restart charging cycle
        state = IDLE;
      } else {
        arm_flash_int();
        if (REFRESH_CHARGE_THRESHOLD_EXP(target_volts, current_volts)) {
          CHARGE_ON;
        }
      }
      break;
    case FLASHING:
      idle_timeout = millis() + FLASH_IDLE_DELAY_MS; // Stay idle for 250ms to allow arc to extinguish
      state = IDLE; // Delay by two analogReads, in practice about 1ms
      break;
    case FAULT:
      // Steady state: Bank idle, LED off, Lamp off, permanent beep
      TIMSK1 = 0; // Force Lamp Dim off
      ACSR = 0; // Force ACDET interrupt off
      CHARGE_OFF;
      FLASH_OFF;
      BEEP_ON;// Be annoying on purpose to make user turn off device immediately
      DIM_TRIAC_OFF;
      break;
  }
}

void setup_acint() {
  ACSR = 0;  // clear Analog Comparator interrupt enable
  ACSR = (1 << ACI);    // clear Analog Comparator interrupt
  ACSR  |=
    (0 << ACD)   |       // Logic 0: Comparator ON
    (0 << ACBG)  |       // Disconnect 1.23V reference from AIN0 (use AIN0 and AIN1 pins)
    (0 << ACIC)  |       // input capture disabled
    (0 << ACIS1) |       // set interrupt bit on BOTH edges
    (0 << ACIS0);        //
  // Enable the interrupt
  ACSR |= (1 << ACIE);
}

// Two Timer1 Copare Match Interrupts are used to generate a short, delayed pulse on DIM_TRIAC
void setup_tint() {
  TIMSK1 = 0;
  TCCR1A = 0;     // We hate TCCR1A
  TCCR1B = 0b00000010;    // 8 prescaler
  OCR1A = MAX_DIM_MICROS - TRIAC_FIRE_MICROS;  // We subtract fixed values from TCNT to configure frequency. We could use the overflow interrupt instead, but this is just how I did it
  OCR1B = MAX_DIM_MICROS;  // We subtract fixed values from TCNT to configure frequency. We could use the overflow interrupt instead, but this is just how I did it
  TCNT1 = 0;      // First interval is assumed to be full 99ms
  TIMSK1 = (1 << OCIE1A | 1 << OCIE1B); // enable timer overflow interrupts A and B
}

void setup_flash_int() {
  EICRA = 0b00001010; // 10: Falling edge on INTn triggers interrupt
}

inline void arm_flash_int() {
  EIFR = 3;  // Clear INT status (Thanks Utsuho)
  EIMSK = 3; // Set INT1 and INT0 bits
  LED_ON;
}

inline void disarm_flash_int() {
  EIMSK = 0; // Clear INT1 and INT0 bits
  LED_OFF;
}

// Perform the flash by pulling the flash triac high.
// Perform all housekeeping after FLASH_ON to ensure fast response
inline void flash() {
  FLASH_ON;
  CHARGE_OFF;
  disarm_flash_int();
  state = FLASHING;
}

// Both external interrupts fire the flash. This needs to respond as quickly
// as possible to the interrupt. The current implementation has a latency of
// about 2 microseconds from input edge to triac edge.
ISR(INT0_vect) {
  flash();
}

ISR(INT1_vect) {
  flash();
}

// This interrupt fires a determined amount of time after an ACDET interrupt
// and turns on the incandescent lamp.
ISR(TIMER1_COMPA_vect) {
  DIM_TRIAC_ON;
}

// This interrupt fires after TIMER1_COMPA_vect and turns lamp off.
// We use this to conveniently keep DIM_TRIAC's gate on for a few microseconds
ISR(TIMER1_COMPB_vect) {
  DIM_TRIAC_OFF;
}

// ACDET Interrupt fires every zero crossing of mains and configures Timer1 to
// fire after a delay such that the DIM_TRIAC acts as a forward phase cut dimmer.
// dim_micros effectively calculates the time the triac wil be on each half wave.
// There is some error because the ACDET interrupts do not happen symmetrically.
// In theory, MAX_DIM_MICROS is exactly 10000. This value would not work because
// at max intensity, we would be firing the triac at zero crossing, which is impossible.
// Additionally, we account for the ACDET asymmetry by requiring a minimum and
// maximum duty cycle. At near full intensity, we stop switching and energize
// the triac continuously. We don't need an exact phase cut anyway.
ISR(ANALOG_COMP_vect) {
  long dim_micros = MIN_DIM_MICROS + ((MAX_DIM_MICROS - MIN_DIM_MICROS) * current_intens) / 1023L;
  if (!READ_DIM && dim_micros < MAX_DIM_MICROS - 3 * TRIAC_FIRE_MICROS) {
    DIM_TRIAC_OFF;
    TCNT1 = dim_micros;
  } else {
    DIM_TRIAC_ON;
  }
}
