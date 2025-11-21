#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000UL
#define TIMER_START_VALUE 49911 

ISR(TIMER1_OVF_vect) {
    TCNT1 = TIMER_START_VALUE;
    PORTB ^= (1 << 0);
}

int main(void) {
    DDRB = (1 << 0);
    TCNT1 = TIMER_START_VALUE;
    TIMSK = (1 << TOIE1);
    TCCR1B = (1 << CS12) | (1 << CS10);
    sei();
    while (1) {}
}