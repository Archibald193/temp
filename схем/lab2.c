#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#define F_CPU 8000000UL

int main(void) {
    DDRB = 0xFF;
    DDRD = 0x00;
    PORTD = 0xFF;
    PORTB = 0x00;

    uint8_t a = 0;
    uint8_t b = 0;

    while (1) {
        if (!(PIND & (1 << 0))) {
            a++;
            PORTB = a;
            _delay_ms(200);
        }

        if (!(PIND & (1 << 1))) {
            b++;
            PORTB = b;
            _delay_ms(200);
        }

        if (!(PIND & (1 << 2))) {
            PORTB = a + b;
        }

        if (!(PIND & (1 << 3))) {
            PORTB = a - b;
        }

        if (!(PIND & (1 << 4))) {
            PORTB = a * b;
        }

        if (!(PIND & (1 << 5))) {
            if (b != 0) {
                PORTB = a / b;
            } else {
                PORTB = 0xFF;
                _delay_ms(500);
            }
        }

        if (!(PIND & (1 << 6))) {
            a = 0;
            b = 0;
            PORTB = 0;
        }
    }
}