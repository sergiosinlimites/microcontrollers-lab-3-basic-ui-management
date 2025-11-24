#include <xc.h>                       //Libreria de PIC's, que pic se selecciono, llamar bits de con
#define _XTAL_FREQ 4000000  // Definir la constante para el cálculo de retardos
#pragma config FOSC=HS       // Configurar el reloj externo
//#pragma config FOSC=INTOSC_EC // Configurar el reloj externo
#pragma config WDT=OFF      // Desactivar el perro guardian libera pin RB0 a RB4 para que sean digitales 
#pragma config LVP=OFF       // Programa el PIC NO PERMITER PROGRAMACION EN BAJO VOLTAJE

//Las funciones se escriben despues del MAIN en C


//Prototipos
unsigned char segmentos;                         // Variable char-> 8 bits 7 SEGMENTOS
unsigned char DECENAS_RGB;                  //Led RGB se usa para que se lea como decenas pero abajo por ejemplo para cada
                                                                     //decena se le asigna el color correcto correspondiente
void __interrupt()ISR(void);                        //Definir interrupciones LIBRERIA XC8 Propia VOID no recibe ni entrega nada
//Que_dato_retornan... Nombre_de_la_función...Que_tipo_de_dato_reciben
unsigned char sensor;                               //Puede ser 0 o 1 u otra cosa por convenencia 0 o 1
unsigned char Retorno;                            //Puede ser 0 o 1 u otra cosa por convenencia 0 o 1


void main (void){
    //CONFIGURACIÓN DE LAS VARIABLES 
    DECENAS_RGB=0;            //Inicia en 0 peroooo que es 0 pues magenta
    segmentos=0;                 //Variable que se usa para visualizar el numero en el 7segmentos
                                            //PIC no le importa si se introduce el pin en binario, decimal, hexadecimal
                                            //Decodificador BCD se encarga de pasarlo a BCD
    
    sensor = 1;                      //Logica negativa por el sensor lo usamos para asegurarnos que el sensor no cuente doble
    Retorno=0;                     //Variable que usaremos para detectar HLVD, cuando cae el voltaje a menos de 4.11
    
    // CONFIGURACIÓN DE LOS PUERTOS
    ADCON1=0b00001111; //Quital las funciones analogas de los pines RA1-RA4, RB0-RB4 , RE0-RE2   
    
    // Pines para el RGB
    TRISE=0; // Todos los pines del puerto E son salidas digitales  
    LATE=0b00000111; // Todos los pines de salida del puerto E en 1 como la logica esta inversa pues aparece inicialmente en NEGRO
                                     //todo apagado
    
    // Pines para el Siete segmentos
    TRISD=0;                         // Todos los pines del puerto D son salidas digitales
    LATD=segmentos;           // El puerto es igual a el valor de la variable segmentos 
    
    // Pin del led de operación
    TRISA1=0; // Pin A1 es configurado como salida digital
    LATA1=0; // La salida del Pin A1 es 0
    
    
    TRISA2=0; // Pin A1 es configurado como SALIDA DIGITAL
    LATA2=0; // La salida del Pin A2 es 0
    
    //TRISB2 = 0; // Configura RB2 como salida
    //LATB2 = 0;   // Inicializa en nivel bajo
    
    //Pin del pulsador de conteo
    TRISC1=1; //Pin C1 es configurado como ENTRADA DIGITAL
 
    // CONFIGURACIÓN DE LAS INTERRUPCIONES 
    
    // Configuración de la interrupción del TIMER0
    
    T0CON=0b00000011; //Configuración del timer0 modo 16 bits - prescale 16 
    TMR0=34286; // Valor de precarga del TIMER0
    TMR0IF=0; // Bandera inicializada en 0 para que todavia no cuente
    TMR0IE=1; // Habilitación local de la interrupción 
    TMR0ON=1; // Encender el Timer0
    
    // Configuración de la interrución RB0 PARADA DE EMERGENCIA
    //Usamos registros INTCON y INTCON2
    INTEDG0=1;                          // Se active con una subida  (Registro INTCON2)
    INT0IF=0;                              // Bandera inicializada en 0  BANDERA CARACTERISTICA (Registro INTCON)
    INT0IE=1;                              // Habilitación local de la interrupción ENABLE (Registro INTCON)
   
    
    // Configuración de la interrución RB1 REINICIO DE CONTEO
    //INT1 exclusiva de RB1
    INTEDG1=1; // Se active con una subida 
    INT1IP=1; // Prioridad de la interrupción (Alta)
    INT1IF=0; // Bandera inicializada en 0 BANDERA CARACTERISTICA
    INT1IE=1; // Habilitación local de la interrupción ENABLE pa' que se pueda hacer la interrupcion
    
    
    
     //Habilitacion HLVDCON BONO Si o si toca poner el registro si no no funciona ¡¿?!
    HLVDCONbits.HLVDL   = 0b1101;   // Selección del umbral TABLE 18-6 Y Pag 287 SE ACTIVA CON MENOS DE 4.11V
    HLVDCONbits.VDIRMAG = 0;        // Detectar caída
    HLVDCONbits.HLVDEN  = 1;           // Encender HLVD

    while(!HLVDCONbits.IRVST);     //Esperar referencia estable osea espera un 1, si no es estable vale un 0 (!Negacion lo contrario)

    HLVDIF = 0;                  // Limpiar bandera
    HLVDIE = 1;                  // Habilitar interrupción HLVD
    
    PEIE=1;                                      // Habilitar interrupciones de perifericos 
    GIE=1;                                       //Habilitación global de las interrupciones para que funcionen, se ponen al final del codigo
    
    while (1){                                //While infinito de todo el codigo
        
        if(RC1==0){ // Verifica si el interruptor está pulsado  o el sensor tiene algo en frente
            sensor =0; //Para que no cuente infinitamente
        }
        
        if(sensor==0){                                                             //Verifica si el interruptor está pulsado o el sensor tiene algo en frente
            if(RC1==1){                                                              //Dejo de detectar 
                __delay_ms(50);                                                  // delay pa' el pulsador pa' que no cuente doble
                segmentos++;                                                    //Aumenta la variable segmentos +1 
                if (segmentos==10 && DECENAS_RGB !=6){ //Si la cuenta llega a 10 y las decenas son diferentes de 6
                    segmentos=0;                                               //Se reinicia la cuenta y en el siete segmentos aparece un 0

                    for(int i = 0; i < 600; i++){  // 600 ciclos * 0.5 ms = 300 ms ESTO NO ES NECESARIO SI EL BUZZER FUERA ACTIVO
                        //NECESITA UN TREN DE PULSOS SI ES PASIVO NO ES PWM
                             LATA2 = 1;
                              __delay_us(250);
                            LATA2 = 0;
                            __delay_us(250);
                   }
                    DECENAS_RGB++;  //Cambio de color pasa de 0 a 1 a 2 a 3 a 4 ...
                }
                
                if (DECENAS_RGB==6){ // !!!! Ojito llega a 6 decenas aqui debe resetear
                       for(int i = 0; i < 2000; i++){
                            LATA2 = 1;
                            __delay_us(250);
                            LATA2 = 0;
                            __delay_us(250);
                    }
                       DECENAS_RGB = 0;  //vuelve a 0 osea magenta

                    }
                LATD=segmentos; //El pic le da igual si lee binario o decimal en este caso lo esta leyendo en decimal, 
                //luego mando binario y el conversor en fisico lo pasa a BCD
             

                //RGB - decenas 
                //RE2=Rojo
                //RE1=Azul
                //RE0=Verde
                
                if(DECENAS_RGB==0){
                    LATE=0b00000001; // Magenta  (Rojo+Azul)
                }else if(DECENAS_RGB==1){
                    LATE=0b00000101; // Azul 
                }else if(DECENAS_RGB==2){
                    LATE=0b00000100; // Cyan 
                }else if(DECENAS_RGB==3){
                    LATE=0b00000110; // Verde 
                }else if(DECENAS_RGB==4){
                    LATE=0b00000010; // Amarillo 
                }else if(DECENAS_RGB==5){
                    LATE=0b00000000; // Blanco
                }

                sensor=1; //Vuelve a su valor original
            }
        }    
    }
}

// INTERRUPCIONES 
/*
Queremos que oscile a 1Hz tiene que cumplir un ciclo cada segundo ósea prende y apaga eso es un ciclo para que cumpla cada 0,5 segundos debe cambiar de estado 

T=0,5s (Tiempo)
C=65536 ? TMR0 (Precarga)
N (Preescaler) = 16
Tiempo de instrucción (Variable) = 4/Fosc (Fosc reloj) = 1micro segundo
T= (65536-TMR0)N(Tiempo de instrucción) -> TMR0=34286 PRECARGA cada 0,5 seg
*/



void __interrupt()ISR(void){
    if(TMR0IF==1){                                  // Led de operacion RA1
        if(Retorno==0){                                //Si no se detecta cambio en el voltaje HLVD conmuta normalito
            TMR0=34286;                              // Valor de precarga de 0,5 Seg
            TMR0IF=0;                                   //Vuelve a contar 
            LATA1=LATA1^1; // Prende y apaga el led  CONMUTA XOR
        }
        else if(Retorno == 1){                     //Si detecta el cambio de HLVDL 
                                                                 //Cambio conmuta 4 veces mas rapido
            
            TMR0=49911  ;                         // Valor de precarga -> T= C*N*Tinstr -> C= 65536-precarga(TMR0); 
                                                              //N=16 (Configuración/Preescaler); Tinstr=4/Fosc=1us -> Despejando 
                                                              //C=15625 -> TMR0=65536 - C = (TMR0=49911)
            TMR0IF=0;                               // Bandera en 0
            LATA1=LATA1^1;                     // Prende y apaga el led XOR
            Retorno=0;                              //Para que vuelva a preguntar Variable HLVD
        }

    }
    
    
    if (HLVDIF)                                      //Si se activó la interrupción HLVD
    {
        Retorno=1;                                 //Aqui usamos la variable para saber si el voltaje BAJO recuerda que inicio en 0
        HLVDIF = 0;                               // Limpiar bandera            
    }
    
    if (INT0IF==1){                              // Para de emergencia - RB0
        INT0IF=0;                                 // Bandera en 0
        LATE=0b00000011;                 // El led del rgb en rojo      
        while(1){                                   //Infinito sin parada  
            
            
            //CONMUTACIÓN LED RA1 INCLUSO EN PARADA DE EMERGENCIA
                if(TMR0IF==1){                                  // Led de operacion RA1
                    if(Retorno==0){
                        TMR0=34286;                              // Valor de precarga de 0,5 Seg
                        TMR0IF=0;                                   //Vuelve a contar 
                        LATA1=LATA1^1;                          // Prende y apaga el led  CONMUTA XOR
                    }
                    
                            else if(Retorno == 1){                 //Cambio conmuta 4 veces mas rapido
                                TMR0=49911  ;                          // Valor de precarga -> T= C*N*Tinstr -> C= 65536-precarga(TMR0); 
                                                                                //N=16 (Configuración/Preescaler); Tinstr=4/Fosc=1us -> Despejando 
                                                                                //C=15625 -> TMR0=65536 - C = (TMR0=49911)
                                TMR0IF=0;                                // Bandera en 0
                                LATA1=LATA1^1;                       // Prende y apaga el led XOR
                                Retorno=0;                            //Para que vuelva a preguntar 
                            }

                 }
                

                /*FIN de CONMUTACIÓN LED RA1 INCLUSO EN PARADA DE EMERGENCIA
                Lo unico que hice fue copiar el IF de LED DE OPERACION
                 * 
                Curiosidad: el PIC al ser alimentado con menos de 4,11 Voltios con este Trocito de codigo 
                sigue conmutando a la  veces mas rapido por la interrupcion HLVD PEROOOOO
                si se activa la parada de emergencia vuelve a conmutar a la velocidad normal
                esto porque tiene PRIORIDAD ALTA la interrupcion del LED ROJO y en el while infinito
                solo definimos que el led RA1 siga conmutando a 1 Seg, si se quiere que cambie solo
                 se debe añadir el IF de HLVD
                */
                
                //CAMBIO DE VELOCIDAD SI SE DESEO SE VA LA CURIOSIDAD :C
                
                /*
                if (HLVDIF){                                      //Si se activó la interrupción HLVD
                    Retorno=1;
                    HLVDIF = 0;                               // Limpiar bandera
                }                
                */
                
                
                  }
    }
    if (INT1IF==1){                              // Reseteo del conteo - RB1
        INT1IF=0;                                 // Bandera en 0
        segmentos=0;                         //El siete segmentos vuelve a 0
        DECENAS_RGB=0;                    
        LATD=segmentos; 
        LATE=0b00000001;                   // Led rgb a Magenta 
    }   
}