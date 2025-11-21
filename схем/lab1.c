#include <avr/io.h>
#include <util/delay.h>
#define F_CPU 8000000UL

int main(void) {
    DDRB = 0xFF;
    DDRD = 0x00;

    PORTD = 0xFF;

    uint8_t mode = 1; 
    static int snake_pos = 0;
    static int direction = 1;

    while (1) {
        if (!(PIND & (1 << 1))) {
            mode = 1;
        }
        else if (!(PIND & (1 << 2))) {
            mode = 2;
        }
        else if (!(PIND & (1 << 3))) {
            mode = 3;
        }

        switch (mode) {
            case 1: {
                int i;
                for (i = 0; i < 8; i++) {
                    PORTB = (1 << i);
                    _delay_ms(100);
                }
                for (i = 6; i > 0; i--) {
                    PORTB = (1 << i);
                    _delay_ms(100);
                }
                break;
            }
            case 2: {
                if (!(PIND & (1 << 0))) {
                    direction = -1;
                } else {
                    direction = 1;
                }
                PORTB = (1 << snake_pos);
                _delay_ms(150);
                snake_pos += direction;

                if (snake_pos > 7) {
                    snake_pos = 0;
                }
                if (snake_pos < 0) {
                    snake_pos = 7;
                }
                break;
            }

            case 3: {
                PORTB = ~PIND;
                break;
            }
        }
    }
}