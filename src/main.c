/* Playing with getting the small stepper motors driven. */
#include <avr/io.h>
#include <util/delay.h>

/* microseconds between steps */
#define DELAY 500
#define STEP_PIN PD3
#define DIR_PIN  PD4
#define DIR_BUTTON PD5
#define STOP_START PD6

volatile unsigned int direction = 0;

enum {
    STOP = 0,
    CV,
    CCV
};

unsigned int spinStatus = STOP;

unsigned int debounce(volatile uint8_t *port, uint8_t pin);

int main(void) {

    DDRD = 0b10011111;
    PORTD = 0x00;

    while(1) {

        if (debounce(&PIND, DIR_BUTTON)) {
            PORTD ^= (1 << DIR_PIN);
        }
        if (debounce(&PIND, STOP_START)) {
            spinStatus++;
            if (spinStatus) {
                spinStatus = STOP;
            }
        }

        if (spinStatus != STOP) {
            PORTD ^= (1 << STEP_PIN);
            _delay_us(DELAY);
        }

    }
}

unsigned int debounce(volatile uint8_t *port, uint8_t pin) {
    if (!(*port & (1 << pin))) {
        _delay_ms(50);
        if ( *port & (1 << pin) ) {
            _delay_ms(50);
            return 1;
        }
    }
    return 0;
}
