#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#define F_CPU 8000000UL

volatile bool is_timer_running = true;

ISR(TIMER1_OVF_vect) {
    PORTB ^= (1 << 0); 
}

int main(void) {
    DDRB = 0xFF;
    DDRD = 0x00;
    PORTD = 0xFF;

    TIMSK = (1 << TOIE1);
    TCCR1B = (1 << CS12) | (1 << CS10);
    sei();
    while (1) {
        if (!(PIND & (1 << 4))) {
            _delay_ms(200);
            is_timer_running = !is_timer_running; 
            if (is_timer_running) {
                TCCR1B = (1 << CS12) | (1 << CS10);
            } else {
                TCCR1B = 0x00;
            }
        }
    }
}