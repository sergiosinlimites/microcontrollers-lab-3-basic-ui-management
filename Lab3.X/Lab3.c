#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000UL

// ==================== CONFIG (MCLRE OFF) ====================
#pragma config PLLDIV=1, CPUDIV=OSC1_PLL2, USBDIV=1
#pragma config FOSC=INTOSCIO_EC, FCMEN=OFF, IESO=OFF
#pragma config PWRT=OFF, BOR=OFF, VREGEN=OFF
#pragma config WDT=OFF // Perro guardin
#pragma config PBADEN=OFF // Funciones análogas
#pragma config MCLRE=OFF, LPT1OSC=OFF, CCP2MX=ON
#pragma config STVREN=ON, ICPRT=OFF, XINST=OFF, DEBUG=OFF
#pragma config LVP=OFF // Liberar pines

// ===== Parámetros =====
#define BEEP_PERIOD_US 250u
#define ON  1u
#define OFF 0u

// Buzzer (RA2)
#define BUZZ_TRIS  TRISAbits.TRISA2
#define BUZZ_LAT   LATAbits.LATA2
// LED aqua (RA1)
#define LEDA_TRIS  TRISAbits.TRISA1
#define LEDA_LAT   LATAbits.LATA1
// Timer0 para 1 Hz en RA1 (toggle cada 0.5 s): preload 0xF0BE con 1:256
#define TMR0_PRELOAD_H  0xF0
#define TMR0_PRELOAD_L  0xBE

// Entradas
// Botón para bloquear conteo al poner 5V
#define BTN_EMG_TRIS  TRISBbits.TRISB0
#define BTN_EMG_PORT  PORTBbits.RB0
// Botón para resetear conteo al poner 5V
#define BTN_RST_TRIS  TRISBbits.TRISB1
#define BTN_RST_PORT  PORTBbits.RB1   // flanco 0->1 = reset
// Botón para contar cuando al poner 0V
#define BTN_RC1_TRIS  TRISCbits.TRISC1
#define BTN_RC1_PORT  PORTCbits.RC1   // flanco 1->0 = paso (pull-up externo)

// RGB (RE0=B, RE1=G, RE2=R) - Cátodo
#define RGB_R_TRIS  TRISEbits.TRISE2
#define RGB_G_TRIS  TRISEbits.TRISE1
#define RGB_B_TRIS  TRISEbits.TRISE0
#define RGB_R_LAT   LATEbits.LATE2
#define RGB_G_LAT   LATEbits.LATE1
#define RGB_B_LAT   LATEbits.LATE0

// Display 7 segmentos

// ===== Estado =====
volatile uint8_t unidades=0, decenas=0;
volatile uint8_t started=0, finished=0;

// ===== Prototipos =====
void init_hw(void);
void tmr0_start(void);
void sevenseg_bcd(uint8_t d);      // *** SOLO RD0..RD3 ***
void rgb_set(uint8_t r, uint8_t g, uint8_t b);
void set_rgb_by_decena(uint8_t d);
void beep_ms(uint16_t ms);
void advance_one_press(void);
void reset_to_zero(uint8_t keep_started);

// ===== ISR =====
void __interrupt(high_priority) high_isr(void){
    if (INTCONbits.TMR0IF){
        INTCONbits.TMR0IF = 0;
        TMR0H = TMR0_PRELOAD_H;
        TMR0L = TMR0_PRELOAD_L;
        LEDA_LAT ^= 1; // RA1 1 Hz
    }
}

// ===== MAIN =====
void main(void){
    init_hw();
    tmr0_start();

    sevenseg_bcd(0);        // inicio: 0 (solo RD0..3)
    rgb_set(OFF,OFF,OFF);   // inicio: NEGRO
    started = 0; finished = 0;

    uint8_t last_rc1 = 1;   // RC1 reposo=1 (pull-up)
    uint8_t last_rb1 = 0;

    while(1){
        // Emergencia por nivel
        if (BTN_EMG_PORT){ rgb_set(ON,OFF,OFF); __delay_ms(10); continue; }

        // RC1: flanco 1->0 = avanzar un paso (o reiniciar si estaba en 59)
        uint8_t rc1 = BTN_RC1_PORT;
        if (last_rc1==1 && rc1==0){
            if (!started && !finished){ started = 1; set_rgb_by_decena(0); } // magenta al iniciar
            advance_one_press();
            __delay_ms(60);             // antirrebote
        }
        last_rc1 = rc1;

        // RB1: reset (flanco 0->1), mantiene si ya habías ?empezado?
        uint8_t rb1 = BTN_RST_PORT;
        if (last_rb1==0 && rb1==1){
            reset_to_zero(started);     // 00 y magenta si started=1; negro si no
            __delay_ms(60);
        }
        last_rb1 = rb1;
    }
}

// ===== Initialize Hardware =====
void init_hw(void){
    // Reloj interno 8 MHz
    OSCCONbits.IRCF = 0b111;
    OSCCONbits.SCS  = 0b10;

    // Digital total y comparadores OFF
    ADCON1 = 0x0F;
    CMCON  = 0x07;

    // Display: SOLO RD0..RD3 como salidas; RD4..RD7 quedan INPUT
    TRISD = 0b11110000; // Entradas Salidas
    LATD  = (LATD & 0xF0);     // limpia nibble bajo sin tocar D7..D4

    // RGB (RE2=R, RE1=G, RE0=B)
    RGB_R_TRIS = 0;
    RGB_G_TRIS = 0;
    RGB_B_TRIS = 0;
    LATE = 0b00000000;

    // Buzzer (RA2) y LED aqua (RA1)
    BUZZ_TRIS = 0;
    BUZZ_LAT = 0;
    LEDA_TRIS = 0;
    LEDA_LAT = 0;

    // Entradas
    BTN_EMG_TRIS = 1;   // RB0
    BTN_RST_TRIS = 1;   // RB1
    BTN_RC1_TRIS = 1;   // RC1
    INTCON2bits.RBPU = 1; // pull-ups internos RB deshabilitados

    // Habilita prioridades (solo TMR0)
    RCONbits.IPEN   = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
}

void tmr0_start(void){
    // T0CON: [TMR0ON T08BIT T0CS T0SE PSA T0PS2 T0PS1 T0PS0] = [7:0]
    T0CON = 0b00000001
    // TMR0ON=0: Stops timer 0;
    // T08BIT=0: Timer0 is configured as a 16-bit timer/counter ;
    // T0CS  =0: Internal instruction cycle clock (CLKO);
    // T0SE  =0: Increment on low-to-high transition on T0CKI pin;
    // PSA   =0: Timer0 prescaler is assigned;
    // T0PS2 =0; // 1:4 el minimo para que quepa 64k / 250k con 1MHz
    // T0PS1 =0;
    // T0PS0 =1;
    
    
    INTCON2bits.TMR0IP=;
    INTCONbits.TMR0IF =0;
    INTCONbits.TMR0IE =1;

    TMR0H=TMR0_PRELOAD_H; TMR0L=TMR0_PRELOAD_L;
    T0CONbits.TMR0ON=1;
}

// === Display BCD en RD0..RD3 (NO toca RD4..RD7) ===
void sevenseg_bcd(uint8_t d){
    uint8_t v = (d & 0x0F);        // BCD 0..9
    LATD = (LATD & 0xF0) | v;      // solo nibble bajo
}

// RE2=R, RE1=G, RE0=B (común cátodo: 1=encendido)
void rgb_set(uint8_t r, uint8_t g, uint8_t b){
    RGB_R_LAT = r ? 1 : 0;
    RGB_G_LAT = g ? 1 : 0;
    RGB_B_LAT = b ? 1 : 0;
}

void set_rgb_by_decena(uint8_t d){
    switch (d % 6){
        case 0: rgb_set(ON,OFF,ON); break;   // magenta (R+B)
        case 1: rgb_set(OFF,OFF,ON); break;  // azul (B)
        case 2: rgb_set(OFF,ON,ON); break;   // cian (G+B)
        case 3: rgb_set(OFF,ON,OFF); break;  // verde (G)
        case 4: rgb_set(ON,ON,OFF); break;   // amarillo (R+G)
        default: rgb_set(ON,ON,ON); break;   // blanco (R+G+B)
    }
}

void beep_ms(uint16_t ms){
    uint32_t n = ((uint32_t)ms*1000u)/(2u*BEEP_PERIOD_US);
    while(n--){
        BUZZ_LAT=1; __delay_us(BEEP_PERIOD_US);
        BUZZ_LAT=0; __delay_us(BEEP_PERIOD_US);
    }
}

// Avanza una unidad por cada toque. Si estaba en 59, el siguiente toque reinicia.
void advance_one_press(void){
    if (finished){                // en 59 ? reset por toque
        reset_to_zero(1);         // started se mantiene
        finished = 0;
        return;
    }

    if (unidades < 9){
        unidades++;
    } else {
        unidades = 0;
        if (decenas < 5){
            decenas++;
            set_rgb_by_decena(decenas);
            beep_ms(80);          // beep al cambiar de decena (color)
        } else {
            decenas = 5; unidades = 9; // 59
            finished = 1;
        }
    }
    sevenseg_bcd(unidades);
}

void reset_to_zero(uint8_t keep_started){
    unidades = 0;
    decenas  = 0;
    finished = 0;
    sevenseg_bcd(0);
    if (keep_started){ set_rgb_by_decena(0); }  // magenta si ya estaba iniciado
    else              { rgb_set(OFF,OFF,OFF); } // negro si aún no había iniciado
    started = keep_started;
}