#define DEBUG true

#define WM_ACCESSPOINT_PASSWORD "cynu4c9r"

// ESP-01
// GPIO_0
// TX = GPIO1 = LED
// GPIO_2
// RX = GPIO3

#define STEP_PIN 0
#define DIR_PIN 2
#define ENABLE_PIN 3

#define STEPPER_A4988
// #define STEPPER_TMC2209

// A4988 Stepper
#ifdef STEPPER_A4988
#define STEPPER_ACCELERATION 800
#define STEPPER_MAXSPEED 400
#endif

// TMC2209
#ifdef STEPPER_TMC2209
#define STEPPER_ACCELERATION 800
#define STEPPER_MAXSPEED 400
#endif

#define EEPROM_ADDRESS 0
#define EEPROM_SIZE 10