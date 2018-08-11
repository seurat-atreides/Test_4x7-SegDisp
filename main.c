
#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define _CA // used to determine if the 7-Seg display is common anode
#define _DELAY 64 // TCCR0B overflows: aprox. 2ms

/*
                 Gd 12V
                 Gd Rt V+ Ck MI MO (tinyusb programmer)
                  +  +  +  +  +  +
   +=====================================================+
   |  .  .  .  .  .  I  .  .  .  .  .  .  .  .  .  .  .  | [I]n (78L05)
   |  .  .  .  .  C  .  .  .  .  .  .  .  .  .  .  .  .  | [C]ommon
   |  .  .  .  .  .  .  O  .  .  .  .  .  .  .  .  .  .  | [O]ut
   | (v)(d)(b)(c) +--------o (v) .  .  .  .  . (d)(c) .  |
   |  +--+--+--+  |  o  +--+-(1)-A--F-(2)(3)-B--+--+  .  |
   | |+         | 1k | |+ b7..6..5..4..3..2..1..0 d6|    |
   | |         -| |  1k|R d0..1 a1..0 d2..3..4..5  -|    |
   |  +--+--+--+  o  |  +--+--+--+--+--+--+--+--+--+  .  |
   |  .  .  .  .  .  o  -  -  E  D (.) C  G (4) -  -  .  |
   |  o--------------o  .  .  .  .  .  .  .  .  .  .  .  |
   |  . (a) .  E  B  C  . (a) .  .  .  .  .  . (b) .  .  | EBC of 2n2222
   |  .  .  .  .  .  .  .  .  .  .  .  .  .  o--B--o  .  |
   +=====================================================+
                    (p)(p)                                 join (p)(p) during programming

*/

//Port B functional assignments
#define SEG_A_PB	_BV(5)
#define SEG_B_PB	_BV(1)
#define SEG_C_PB	0x00
#define SEG_D_PB	0x00
#define SEG_E_PB	0x00
#define SEG_F_PB	_BV(4)
#define SEG_G_PB	0x00
#define SEG_d_PB	0x00
#define DIGIT_0_PB	_BV(6)
#define DIGIT_1_PB	_BV(3)
#define DIGIT_2_PB	_BV(2)
#define DIGIT_3_PB	0x00

//Port B functional assignments
#define SEG_A_PD	0x00
#define SEG_B_PD	0x00
#define SEG_C_PD	_BV(2)
#define SEG_D_PD	_BV(7)		// d.7 maps to a.1
#define SEG_E_PD	_BV(1)
#define SEG_F_PD	0x00
#define SEG_G_PD	_BV(3)
#define SEG_d_PD	_BV(6)		// d.6 maps to a.0
#define DIGIT_0_PD	0x00
#define DIGIT_1_PD	0x00
#define DIGIT_2_PD	0x00
#define DIGIT_3_PD	_BV(4)

// calcuate the number of segments on individual characters
// will be used to decide how long a character stays "on"
#define SEGS_STAY(v) \
   (((v & _BV(6)) ? 1 : 0) +\
	((v & _BV(5)) ? 1 : 0) +\
	((v & _BV(4)) ? 1 : 0) +\
	((v & _BV(3)) ? 1 : 0) +\
	((v & _BV(2)) ? 1 : 0) +\
	((v & _BV(1)) ? 1 : 0) +\
	((v & _BV(0)) ? 1 : 0)) // | 0x40

// Calculate the IO port pins for individual characters on a particular port, according to the "abcdefg" segment/port distribution
// This is done at compile time so that no runtime cycles are needed
// to map segment to port pins
#define SEGS_PORT_DET(p, v) \
   (((v & _BV(6)) ? SEG_A_P##p : 0) |	\
	((v & _BV(5)) ? SEG_B_P##p : 0) |	\
	((v & _BV(4)) ? SEG_C_P##p : 0) |	\
	((v & _BV(3)) ? SEG_D_P##p : 0) |	\
	((v & _BV(2)) ? SEG_E_P##p : 0) |	\
	((v & _BV(1)) ? SEG_F_P##p : 0) |	\
	((v & _BV(0)) ? SEG_G_P##p : 0))

//What stays port bit combinations
#define SEGS_PORT(v) {SEGS_STAY(v),SEGS_PORT_DET(B, v),SEGS_PORT_DET(D, v)}

//What IO pins are used to light the character segments
#define SEGS_B (SEG_A_PB|SEG_B_PB|SEG_C_PB|SEG_D_PB|SEG_E_PB|SEG_F_PB|SEG_G_PB|SEG_d_PB)
#define SEGS_D (SEG_A_PD|SEG_B_PD|SEG_C_PD|SEG_D_PD|SEG_E_PD|SEG_F_PD|SEG_G_PD|SEG_d_PD)

//What IO pins are used to activate the digit in each port
#define DIGITS_B (DIGIT_0_PB|DIGIT_1_PB|DIGIT_2_PB|DIGIT_3_PB)
#define DIGITS_D (DIGIT_0_PD|DIGIT_1_PD|DIGIT_2_PD|DIGIT_3_PD)

// What IO pins are used in each port
#define USED_B (SEGS_B|DIGITS_B)
#define USED_D (SEGS_D|DIGITS_D)

/*
       ___a__
      |      |
     f|      | b
       ___g__
     e|      | c
      |      |
       ___d__
*/
// Segment to character mapping
//_______________abcdefg
#define LTR_0 0b01111110
#define LTR_1 0b00110000
#define LTR_2 0b01101101
#define LTR_3 0b01111001
#define LTR_4 0b00110011
#define LTR_5 0b01011011
#define LTR_6 0b01011111
#define LTR_7 0b01110000
#define LTR_8 0b01111111
#define LTR_9 0b01111011
#define LTR__ 0b00001000
#define LTRdg 0b01100011
#define LTR_C 0b01001110
#define LTR_c 0b00001101
#define LTR_A 0b01110111
#define LTR_b 0b00011111
#define LTR_J 0b00111100
#define LTR_L 0b00001110
#define LTR_E 0b01001111
#define LTR_t 0b00001111
#define LTR_n 0b00010101
#define LTR_d 0b00111101
#define LTR_i 0b00010000
#define LTR_H 0b00110111
#define LTR_r 0b00000101
#define LTR_o 0b00011101
#define LTR_F 0b01000111
#define LTRml 0b01100110
#define LTRmr 0b01110010
#define BAR_1 0b01000000
#define BAR_2 0b01000001
#define BAR_3 0b01001001
#define BLANK 0b00000000

uint8_t EEMEM digit2ports[][3] = {
	SEGS_PORT(LTR_0), SEGS_PORT(LTR_1), SEGS_PORT(LTR_2), SEGS_PORT(LTR_3),
	SEGS_PORT(LTR_4), SEGS_PORT(LTR_5), SEGS_PORT(LTR_6), SEGS_PORT(LTR_7),
	SEGS_PORT(LTR_8), SEGS_PORT(LTR_9), SEGS_PORT(LTR__), SEGS_PORT(LTRdg),
	SEGS_PORT(LTR_o), SEGS_PORT(LTR_b), SEGS_PORT(LTR_F), SEGS_PORT(LTR_C),
	SEGS_PORT(LTR_t), SEGS_PORT(LTR_H), SEGS_PORT(LTR_n), SEGS_PORT(LTR_r),
	SEGS_PORT(LTR_J), SEGS_PORT(LTR_d), SEGS_PORT(LTR_L), SEGS_PORT(LTR_i),
	SEGS_PORT(LTR_E), SEGS_PORT(LTR_A), SEGS_PORT(BAR_1), SEGS_PORT(BAR_2),
	SEGS_PORT(BAR_3), SEGS_PORT(LTRml), SEGS_PORT(LTRmr), SEGS_PORT(BLANK),
};

enum {
	POS_0, POS_1, POS_2, POS_3,
	POS_4, POS_5, POS_6, POS_7,
	POS_8, POS_9, POS__, POS_o,
	POS_dg, POS_b, POS_F, POS_C,
	POS_t, POS_H, POS_n, POS_r,
	POS_J, POS_d, POS_L, POS_i,
	POS_E, POS_A, POS_B1, POS_B2,
	POS_B3, POS_ml, POS_mr, POS_BLK,
};

volatile bool busy = false;

uint8_t  pos = 0 , pulse, mode = 0;

// ticks per second and derived values
#define TPS   (F_CPU/256)
#define TPS_2 (TPS/2)
#define TPS_4 (TPS/4)

#define NUM_DIGITS	4

static const uint8_t digit_mapb[] PROGMEM = { 0x40,0x08,0x04,0x00 }; // corresponds to the port B digit pins
static const uint8_t digit_mapd[] PROGMEM = { 0x00,0x00,0x00,0x10 }; // corresponds to the port D digit pins

// list of porta..c,ddra..c,stay * number_of_digits
uint8_t output[3 * NUM_DIGITS];
uint8_t *ioptr = output, pause = _DELAY;


ISR(TIMER0_OVF_vect) {

	if (pulse) { pulse--; return; } // return if stays not zero (anode pulse width)

	// clear used bits in the Data Direction Registers and the Port Registers
	DDRA  &= ~(USED_D >> 6);
	DDRB  &= ~USED_B;
	DDRD  &= ~(USED_D & 0x3f);
	PORTA &= ~(USED_D >> 6);
	PORTB &= ~USED_B;
	PORTD &= ~(USED_D & 0x3f);

	if (pause) { pause--; return; } // return if 64 interrupt cycle (0x40) not over
	if (busy) return; //if busy loading from EEPROM then return

	// load next digit
	uint8_t portd = *ioptr++; // get portd value from output
	uint8_t portb = *ioptr++; // get portb value from output
	uint8_t porta = portd >> 6; // map d.7-6 of a.1-0 because  d.7-6 don't exist
	uint8_t mapb = pgm_read_byte(&digit_mapb[pos]); // get the next digit mask for port B
	uint8_t mapd = pgm_read_byte(&digit_mapd[pos]); // get the next digit mask for port D
	DDRA  |= porta;
	DDRB  |= portb | mapb;
	DDRD  |= portd | mapd;

#ifdef _CA
	PORTA |= ~porta;
	PORTB |= ~portb & mapb;
	PORTD |= ~portd & mapd;
#else
	PORTA |= porta;
	PORTB |= portb & ~mapb;
	PORTD |= portd & ~mapd;
#endif

	pulse  = *ioptr++; // get stays value from output
	pause = _DELAY;

	// if all digits have been lighted, pos == 4, then reset pos and ioptr
	if (++pos > 3) {
		pos = 0;
		ioptr = output;
	}
}


//__________________________________________________
void seg2port(uint8_t bcd, uint8_t which) {
	uint8_t *pp = output + which * 3;
	uint8_t offset = 3;
	busy=true;
	do {
		*pp++ = eeprom_read_byte(&digit2ports[bcd][--offset]);
	} while (offset);
	busy=false;
}

//__________________________________________________
int main(void) {

	TCCR0B |= _BV(CS00); // no CLKio prescaling
	TIMSK  |= _BV(TOIE0);
	TCNT0   = 0x00;

	_delay_ms(50);

/*	seg2port(POS_8, 0);
	seg2port(POS_8, 1);
	seg2port(POS_8, 2);
	seg2port(POS_8, 3);*/

	sei();

	while (1) {
		uint8_t bcd, digit;

		for (bcd=0;bcd<29;bcd++) {
			for (digit=0;digit<4;digit++) seg2port(bcd + digit, digit);
			_delay_ms(500);
		}
	}//while
}
