#include <avr/io.h>
#include "wiring_private.h"

#define PORT PORTB
#define DDR DDRB
#define NBIT 0
#define NPIX 32
#define RSPACE (NPIX * 3)
#define ADC_READY_FLAG PORTB0
#define TIMER_FLAG PORTB1
#define BUTTON_FLAG PORTC0
#define BUTTON_PRESSED_FLAG PORTC1
#define LONG_PRESS_FLAG PORTC2

volatile uint8_t vpix[RSPACE];
volatile uint16_t aReadX = 512;  // Valore letto dall'ADC per X
volatile uint16_t aReadY = 512;  // Valore letto dall'ADC per Y
volatile uint16_t button_press_start_time = 0; // Tempo di inizio pressione del pulsante
volatile uint16_t last_click_time = 0;  // Tempo dell'ultimo clic
volatile uint16_t last_move_time = 0; // Tempo dell'ultimo movimento del LED
uint8_t led_x = 3;    // Coordinata X del LED
uint8_t led_y = 3;    // Coordinata Y del LED
uint8_t led_intensity = 100;  // Luminosità del LED
uint8_t color_index = 0;  // Indice del colore attuale

const uint8_t colors[7][3] = {
    {255, 0, 0},    // Rosso
    {255, 127, 0},  // Arancione
    {255, 255, 0},  // Giallo
    {0, 255, 0},    // Verde
    {0, 0, 255},    // Blu
    {75, 0, 130},   // Indigo
    {143, 0, 255}   // Violet
};

volatile uint16_t system_time = 0; // Variabile per tenere traccia del tempo trascorso
volatile uint16_t current_time = 0; // Variabile per memorizzare il tempo corrente
volatile uint16_t led_move_counter = 0; // Contatore per il movimento del LED

void setup() {
    // Configurazione ADC per il joystick
    ADMUX = (1 << REFS0); // Riferimento AVcc
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Configurazione Timer1 in modalità CTC
    TCCR1A = 0; 
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC mode, prescaler 1024
    TIMSK1 = (1 << OCIE1A); // Abilita l'interrupt per OCR1A
OCR1A = 15; 
    // Configurazione pin pulsante come input
    DDRD &= ~(1 << DDD2); // Imposta PD2 come ingresso
    PORTD |= (1 << PORTD2); // Abilita pull-up su PD2
    EICRA |= (1 << ISC01); // Configura l'interrupt esterno INT0 per attivarsi quando il segnale passa da alto a basso
    EIMSK |= (1 << INT0); // Abilita l'interruzione INT0
    

    sei(); // Abilita gli interrupt globali
        system_time = 0;
    // Inizializza il LED
   setColor(255, 255, 255);
    

    // Avvia la conversione ADC
    ADCSRA |= (1 << ADSC);
}

void loop() {
       uint16_t current_time = system_time;

    if (PORTB & (1 << TIMER_FLAG) && (current_time - last_move_time >= 500)) {
        PORTB &= ~(1 << TIMER_FLAG); // Resetta il flag
            led_move_counter = 0; // Resetta il contatore
        // Controlla i valori del joystick per determinare la direzione del movimento
        if (aReadX < 512) {
            if (led_x > 1) {
                led_x--;
            } else {
                led_x = 8; // Torna alla fine della riga
            }
        } else if (aReadX > 512) {
            if (led_x < 8) {
                led_x++;
            } else {
                led_x = 1; // Torna all'inizio della riga
            }
        }

        if (aReadY < 512) {
            if (led_y > 1) {
                led_y--;
            } else {
                led_y = 4; // Torna in fondo alla colonna
            }
        } else if (aReadY > 512) {
            if (led_y < 4) {
                led_y++;
            } else {
                led_y = 1; // Torna in cima alla colonna
            }
        }
        if (!(aReadY == 512 && aReadX == 512)){
        // Aggiorna il LED con la nuova posizione
        updateLED();
        }
            last_move_time = current_time; // Aggiorna il tempo dell'ultimo movimento

    }

    if (PORTC & (1 << BUTTON_FLAG)) {
        PORTC &= ~(1 << BUTTON_FLAG);  // Resetta il flag

        if (current_time - last_click_time < 100) {
            changeColor();
        }

        if (current_time - last_click_time < 500) {
            // Doppio clic rilevato
            increaseIntensity();
        }

        last_click_time = current_time;
    }

    if (PORTC & (1 << BUTTON_PRESSED_FLAG) && (current_time - button_press_start_time >= 1000)) {
        PORTC |= (1 << LONG_PRESS_FLAG);
    }

    if (PORTC & (1 << LONG_PRESS_FLAG) && (PORTC & (1 << BUTTON_PRESSED_FLAG))) {
        static uint16_t last_decrease_time = 0;

        if (current_time - last_decrease_time >= 1000) {
            decreaseIntensity();
            last_decrease_time = current_time;
        }
    }

  
}

void pix_show(uint8_t nb, uint8_t *vp) {
    sbi(DDR, NBIT);  // Imposta il pin come uscita
    cbi(PORT, NBIT);  // Imposta il pin basso
    cli();

    asm volatile (
        "lwhile0:\n\t"
        "ld r24, %a[vp]+\n\t"  // Carica il byte corrente
        "ldi r25, 8 \n\t"  // Contatore bit
        "lwhile:\n\t"
        "nop \n\t"  // Fine ciclo: tau = 62,5*20 = 1,25 us
        "sbi %[port], %[nbit] \n\t"  // Inizia il segnale H
        "rjmp .+0 \n\t"  // delay 2 
        "rjmp .+0 \n\t"  // delay 2 
        "sbrs r24, 7 \n\t"  // Test bit 7, skip se 1
        "cbi %[port], %[nbit] \n\t"  // Se bit 0 impulso corto
        "add r24, r24 \n\t"  // r24 <<=1
        "subi r25, 1 \n\t"  // Decremento contatore bit 
        "breq vnextb \n\t"  // Ciclo lungo, son finiti i bit
        "rjmp .+0 \n\t"  // delay 2 
        "cbi %[port], %[nbit] \n\t"  // Fine segnale H
        "rjmp .+0 \n\t"  // delay 2 
        "nop \n\t"  // delay 1 
        "rjmp lwhile \n\t"  // Loop
        "vnextb:\n\t"
        "nop \n\t"  // delay 1 
        "cbi %[port], %[nbit] \n\t"  // Fine segnale H
        "rjmp .+0 \n\t"  // delay 2 
        "rjmp .+0 \n\t"  // delay 2 
        "subi %[nb], 1 \n\t"  // Decremento contatore byte
        "brne lwhile0 \n\t"  // Se non e' finito continua
        : [vp] "+e" (vp), [nb] "+r" (nb)
        : [port] "I" (_SFR_IO_ADDR(PORT)), [nbit] "I" (NBIT)
        : "r24", "r25");

    sei();
}

// Interrupt per il timer
ISR(TIMER1_COMPA_vect) {
       system_time++; // Aggiorna il tempo di sistema ogni millisecondo
    led_move_counter++; // Incrementa il contatore del movimento del LED
    PORTB |= (1 << TIMER_FLAG); // Imposta il flag del timer
}

ISR(ADC_vect) {
  if (ADMUX & (1 << MUX0)) { // Se il canale ADC1 (Y) è attivo
        aReadY = ADC; // Legge il valore Y
        ADMUX &= ~(1 << MUX0); // Seleziona il canale ADC0 (X)
    } else { // Se il canale ADC0 (X) è attivo
        aReadX = ADC; // Legge il valore X
        ADMUX |= (1 << MUX0); // Seleziona il canale ADC1 (Y)
    }
    ADCSRA |= (1 << ADSC); // Avvia una nuova conversione ADC
}

void updateLED() {
    // Spegne tutti i led
    for (uint16_t i = 0; i < RSPACE; i++) {
        vpix[i] = 0;
    }

    // Calcola l'indice corretto per la posizione attuale del LED
     uint16_t led_index = ((led_y - 1) << 3) + (led_x - 1);
      led_index *= 3;
   vpix[led_index] = (colors[color_index][0] * led_intensity) >> 8;
    vpix[led_index + 1] = (colors[color_index][1] * led_intensity) >> 8;
    vpix[led_index + 2] = (colors[color_index][2] * led_intensity) >> 8;
    // Mostra l'array di LED aggiornato
    pix_show(RSPACE, vpix);
}

// Interrupt del pulsante
ISR(INT0_vect) {
   uint16_t current_time = system_time;

    if (!(PIND & (1 << PIND2))) {
        // Pulsante premuto
        PORTC |= (1 << BUTTON_PRESSED_FLAG);
        button_press_start_time = current_time;
    } else {
        // Pulsante rilasciato
        if (PORTC & (1 << BUTTON_PRESSED_FLAG)) {
            PORTC |= (1 << BUTTON_FLAG);
            PORTC &= ~(1 << BUTTON_PRESSED_FLAG);
            PORTC &= ~(1 << LONG_PRESS_FLAG);
        }
    }
}

void changeColor() {
    color_index = (color_index + 1) % 7; // Cambia colore
    updateLED();
}

void increaseIntensity() {
    if (led_intensity <= 247) {
        led_intensity += 8;
    } else {
        led_intensity = 255; // Imposta la luminosità massima
    }
    updateLED();
}

void decreaseIntensity() {
    if (led_intensity >= 4) {
        led_intensity -= 4;
    } else {
        led_intensity = 0; // Imposta la luminosità minima
    }
    updateLED();
}
void setColor(uint8_t red, uint8_t green, uint8_t blue) {
    uint16_t led_index = ((led_y - 1) << 3) + (led_x - 1);
    vpix[led_index] = red;
    vpix[led_index + 1] = green;
    vpix[led_index + 2] = blue;
        pix_show(RSPACE, vpix);

}