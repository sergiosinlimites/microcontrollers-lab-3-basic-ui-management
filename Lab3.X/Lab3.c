#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 4700000UL //Frecuencia de trabajo ¿? 

// ==================== CONFIG (MCLRE ON) ====================
#pragma config WDT=OFF                                       //Desactiva el Watchdog
#pragma config LVP=OFF                                        //Desactiva el Low-Voltage Programming
#pragma config PBADEN = OFF                               //PORTB digital al inicio (RB0?RB4 no analógicos)
#pragma config MCLRE=ON                                    //Masster clear habilitado
#pragma config FOSC = HS  // Oscilador interno, RA6 y RA7 como I/O o CLKO

//#pragma config FOSC = EC_EC                                // Reloj externo del Cristal (CLKIN en RA6)

// ===== Parámetros =====
#define BEEP_PERIOD_US 250u
#define ON  1u
#define OFF 0u

// Buzzer (RA2)
#define BUZZ_TRIS TRISA2
#define BUZZ_LAT LATA2
// LED clock (RA1)
#define CLK_TRIS TRISA1
#define CLK_LAT LATA1
// Timer0 para 1 Hz en RA1 (toggle cada 0.5s) con prescaler 1:16, son 36718, faltan(28817)_10 = (7091)_16
#define TMR0_PRELOAD_H  0x70
#define TMR0_PRELOAD_L  0x91
// Timer0 para 4Hz en RA1 (toggle cada 0.125s) con prescaler 1:16 son 9180, faltan (56356)_10 = (DC24)_16
#define TMR0_PRELOAD_250MS_H  0xDC
#define TMR0_PRELOAD_250MS_L  0x24


// Entradas
// Botón para bloquear conteo al poner 5V
#define BTN_EMG_TRIS TRISB0
#define BTN_EMG_PORT RB0
// Botón para resetear conteo al poner 5V
#define BTN_RST_TRIS TRISB1
#define BTN_RST_PORT RB1   // flanco 0->1 = reset
// Botón para contar cuando al poner 0V
#define COUNTER_TRIS TRISC1
#define BTN_RC1_PORT RC1   // flanco 1->0 = paso (pull-up externo)

// RGB (RE0=B, RE1=G, RE2=R) - C?todo
#define RGB_R_LAT LATE1
#define RGB_G_LAT LATE0
#define RGB_B_LAT LATE2

// Display 7 segmentos

// ===== Estado =====
unsigned char vdd_low = 0;
volatile uint8_t unidades=0, decenas=0;
volatile uint8_t started=0, finished=0;
volatile uint8_t emg_latched = 0;   // 0 = normal; 1 = paro latcheado (solo MCLR lo limpia)
// ===== Prototipos =====
void init_hw(void);
void hlvd_init_4v(void);
void tmr0_start(void);
void sevenseg_bcd(uint8_t d);      // *** SOLO RD0..RD3 ***
void rgb_set(uint8_t r, uint8_t g, uint8_t b);
void set_rgb_by_decena(uint8_t d);
void beep_ms(uint16_t ms);
void advance_one_press(void);
void reset_to_zero(uint8_t keep_started);

// ===== ISR =====
void __interrupt(high_priority) high_isr(void){
    if (INT0IF){
        emg_latched = 1;
        rgb_set(ON,OFF,OFF); // rojo
        INT0IF = 0;// limpia bandera
    }
}

void __interrupt(low_priority) low_isr(void){
    if (TMR0IF){
        if (vdd_low){
            TMR0H = TMR0_PRELOAD_250MS_H;
            TMR0L = TMR0_PRELOAD_250MS_L;
        } else {
            TMR0H = TMR0_PRELOAD_H;
            TMR0L = TMR0_PRELOAD_L;
        }
        TMR0IF = 0;
        CLK_LAT ^= 1; // RA1 1 Hz / 4 Hz
        vdd_low = 0;
    }
    if(HLVDIF){
        vdd_low = 1;
        HLVDIF = 0;
    }
    
}

// ===== MAIN =====
void main(void){
    // --- I/O base: todo digital ---
    ADCON1 = 0x0F;              // ANx -> digitales
    TRISE  = 0b1000;            // RE3/MCLR input
    LATE   = 0b1000;            // estado seguro en RE
    // Buzzer y LED
    BUZZ_TRIS = 0; BUZZ_LAT = 0;
    CLK_TRIS  = 0; CLK_LAT  = 0;
    // Entradas
    BTN_EMG_TRIS = 1;           // RB0 (EMG)
    BTN_RST_TRIS = 1;           // RB1
    COUNTER_TRIS = 1;           // RC1
    RBPU = 1;                   // pull-ups RB deshabilitados (usa externos)

    // --- Prioridades e INT0 (EMG) ---
    IPEN = 1;                   // usar prioridades
    GIEH = 1;                   // habilita alta prioridad
    GIEL = 1;                   // habilita baja prioridad
    INTEDG0 = 1;                // INT0 flanco 0->1
    INT0IF  = 0;                // limpia bandera
    INT0IE  = 1;                // habilita INT0

    // --- Latch EMG al arranque (si RB0 ya está activo) ---
    __delay_ms(2);              // pequeño settle
    if (BTN_EMG_PORT) {         // EMG activa al encender
        emg_latched = 1;
        rgb_set(ON,OFF,OFF);    // ROJO inmediato
        // Nota: NO tocamos TRISD/LATD ni seteamos 00 en display
    }

    // --- Timer0 para el blink ---
    tmr0_start();
    
    // --- HLVD (~4V) ---
    hlvd_init_4v();
    
    // --- UI solo si NO hay EMG activa ---
    if (!emg_latched) {
        TRISD = 0b11110000;     // RD0..RD3 salidas BCD
        LATD  = (LATD & 0xF0);  // 0 en nibble bajo
        sevenseg_bcd(0);        // mostrar 00
        rgb_set(OFF,OFF,OFF);   // negro
        started = 0;
        finished = 0;
    }

    // --- Loop principal ---
    uint8_t last_rc1 = 1;       // RC1 reposo=1 (pull-up)
    uint8_t last_rb1 = 0;

    while (1) {
        // Paro latcheado: mantener rojo y no atender entradas
        if (emg_latched) {
            rgb_set(ON,OFF,OFF);   // anclar ROJO durante EMG
            __delay_ms(50);
            continue;
        }

        // RC1: flanco 1->0 = avanzar un paso (o reiniciar si estaba en 59)
        uint8_t rc1 = BTN_RC1_PORT;
        if (last_rc1 == 1 && rc1 == 0) {
            if (!started && !finished) { started = 1; set_rgb_by_decena(0); }
            advance_one_press();
            __delay_ms(50);        // antirrebote
        }
        last_rc1 = rc1;

        // RB1: reset (flanco 0->1)
        uint8_t rb1 = BTN_RST_PORT;
        if (last_rb1 == 0 && rb1 == 1) {
            reset_to_zero(started); // 00 y magenta si started=1; negro si no
            __delay_ms(50);
        }
        last_rb1 = rb1;
    }
}

// ===== Initialize Hardware =====
void init_hw(void){
    
}

void hlvd_init_4v(void){
    // Apagar para configurar
    // HLVDCON = [VDIRMAG - IRVST HLVDEN HLVDL3 HLVDL2 HLVDL1 HLVDL0]
    HLVDCON = 0b00000000;
    // Para min=3.7V typ=3.9V max=4.1V
    //HLVDCONbits.HLVDL3 = 1;              // 0b1011
    //HLVDCONbits.HLVDL2 = 1;
    //HLVDCONbits.HLVDL1 = 0;
    //HLVDCONbits.HLVDL0 = 1;
    
    HLVDCONbits.HLVDL = 0b1100;
    HLVDCONbits.VDIRMAG = 0; // Detectar caida
    HLVDCONbits.HLVDEN  = 1; // Habilitar detector
    while (!HLVDCONbits.IRVST); // Mantener quieto hasta que sea estable para generar la interrupción

    HLVDIF = 0; // Bajar bandera
    HLVDIP = 0; // Dejar de alta prioridad con 1
    HLVDIE = 1; // Habilitar
}

void tmr0_start(void){
    // T0CON: [TMR0ON T08BIT T0CS T0SE PSA T0PS2 T0PS1 T0PS0] = [7:0]
    T0CON = 0b00000011; // Prescaler 1:16
    // TMR0ON=0: Stops timer 0;
    // T08BIT=0: Timer0 is configured as a 16-bit timer/counter ;
    // T0CS  =0: Internal instruction cycle clock (CLKO);
    // T0SE  =0: Increment on low-to-high transition on T0CKI pin;
    // PSA   =0: Timer0 prescaler is assigned;
    // T0PS2 =1;
    // T0PS1 =0;
    // T0PS0 =0;
//    if (vdd_low == 1){
//        TMR0H = TMR0_PRELOAD_250MS_H;   // o tus _125ms_ si te quedas con 4 Hz
//        TMR0L = TMR0_PRELOAD_250MS_L;
//    } else {
//        TMR0H = TMR0_PRELOAD_H;         // 0.5 s (para 1 Hz de parpadeo)
//        TMR0L = TMR0_PRELOAD_L;
//    }
    
    TMR0IP = 0; // TMR0 Overflow Interrupt Priority bit como HIGH PRIORITY
    TMR0IF = 0; // Bandera de interrupción
    TMR0IE = 1; // TMR0 Overflow Interrupt Enable bit HABILITADO
    TMR0ON = 1; // Timer0 habilitado
}

// === Display BCD en RD0..RD3 (NO toca RD4..RD7) ===
void sevenseg_bcd(uint8_t d){
    uint8_t v = (d & 0x0F);        // BCD 0..9
    LATD = (LATD & 0xF0) | v;      // solo nibble bajo
}

// RE2=R, RE1=G, RE0=B (com?n c?todo: 1=encendido)
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
        beep_ms(1000);
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
    else              { rgb_set(OFF,OFF,OFF); } // negro si a?n no hab?a iniciado
    started = keep_started;
}