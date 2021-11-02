#include "MKL25Z4.h"

// Pin names
#define SWITCH_PIN  (4)     // A4
#define SV1_PIN     (5      // E5
#define H1_PIN      (3)     // D3
#define H2_PIN      (16)    // C16
#define H3_PIN      (2)     // D2

static int = 0;

int main(void) {
    init();

    while (!IsKeyPressed(SWITCH_PIN))
        ;

    PTE->PSOR = (1 << SV1_PIN);

    while ( (PTC->PDIR & (1 << H2_PIN)) == 0)
        ;

    PTE->PCOR = (1 << SV1_PIN);

    for (;;) {
        i++;
    }

    return 0;
}

void init(void) {
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK) | (SIM_SCGC5_PORTE_MASK);
    // Init pin
    PORTA->PCR[SWITCH_PIN] = PORT_PCR_MUX(1);
	PORTE->PCR[SV1_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[H1_PIN] = PORT_PCR_MUX(1);
	PORTC->PCR[H2_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[H3_PIN] = PORT_PCR_MUX(1);

    // Init input
    PTE->PDDR |= (1 << SV1_PIN);

    PTD->PDDR &= ~(1 << H1_PIN);
    PTC->PDDR &= ~(1 << H2_PIN);
    PTD->PDDR &= ~(1 << H3_PIN);

    // Set to log0
    PTE->PCOR |= (1 << SV1_PIN);

}

static inline int IsKeyPressed(int pin) {
    if ((PTA->PDIR & (1 << pin)) == 0)
        return 1;
    else
        return 0;
}