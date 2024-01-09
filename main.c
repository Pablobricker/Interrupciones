
// Uso del ADC empleando interrupcion de COnversi�n terminada.
// uso del TImer para fijar la frecuencia de muestreo
// en la ISR se lee el resultado de la  conversi�n.

#include "tm4c1294ncpdt.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

void DisableInterrupts(void);
void EnableInterrupts(void);
#define PORTbit0 0x04
#define PORTFbits (GPIO_PORTF_DATA_BITS_R|PORTbit0)
#define PF0 (*(volatile unsigned int *)GPIO_PORTF_DATA_BITS_R)
uint32_t result;

void GPIO_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x00001310; // (a) activa el reloj para el puerto J,E,N,K
//  Flancosdebajada = 0;                // (b) inicializa el contador
//******* Puerto J *******
  GPIO_PORTJ_DIR_R &= ~0x01;       // (c) PJ0 dirección entrada - boton SW1
  GPIO_PORTJ_DEN_R |= 0x01;        //     PJ0 se habilita
  GPIO_PORTJ_PUR_R |= 0x01;        //     habilita weak pull-up on PJ1
  GPIO_PORTJ_IS_R &= ~0x01;        // (d) PJ1 es sensible por flanco
  GPIO_PORTJ_IBE_R &= ~0x01;       //     PJ1 no es sensible a dos flancos
  GPIO_PORTJ_IEV_R &= ~0x01;       //     PJ1 detecta eventos de flanco de bajada
  GPIO_PORTJ_ICR_R = 0x01;         // (e) limpia la bandera 0
  GPIO_PORTJ_IM_R |= 0x01;         // (f) Se desenmascara la interrupcion PJ0 y se envia al controlador de interrupciones
  NVIC_PRI12_R = (NVIC_PRI12_R&0x00FFFFFF)|0x00000000; // (g) prioridad 0  (pag 159)
  NVIC_EN1_R= 0x180000;          //(h) habilita la interrupción 51 y 52 en NVIC (Pag. 154) DATO DIRECTO EN HEXA
//******* Puerto N *******
  GPIO_PORTN_DIR_R |= 0x0F;    // puerto N de salida
  GPIO_PORTN_DEN_R |= 0x0F;    // puerto N habilitado

  GPIO_PORTN_DATA_R = 0x02;
}


//**********FUNCION Inizializa el SysTick *********************
  void SysTick_Init(void){
    NVIC_ST_CTRL_R = 0;                   // Desahabilita el SysTick durante la configuración
    NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // Se establece el valor de cuenta deseado en RELOAD_R
    NVIC_ST_CURRENT_R = 0;                // Se escribe al registro current para limpiarlo
    NVIC_ST_CTRL_R = 0x00000001;         // Se Habilita el SysTick y se selecciona la fuente de reloj
  }

  //*************FUNCION Tiempo de retardo utilizando wait.***************
  // El parametro de retardo esta en unidades del reloj interno/4 = 4 MHz (250 ns)
  void SysTick_Wait(uint32_t retardo){
      NVIC_ST_RELOAD_R= retardo-1;   //número de cuentas por esperar
      NVIC_ST_CURRENT_R = 0;
      while((NVIC_ST_CTRL_R&0x00010000)==0){//espera hasta que la bandera COUNT sea valida
      }
     } //


  void PWM_init(){

      SYSCTL_RCGCGPIO_R |= 0x00000F19; //Activa el reloj de los puertos A, D, E, J, K, L y M (p. 382)
         while((SYSCTL_PRGPIO_R & 0x00001619) == 0){};//Espera a que el reloj se estabilice
      //Configuración del puerto D
        GPIO_PORTD_DIR_R = 0x0F; // Puerto D como salida. (p.760)
        GPIO_PORTD_DEN_R = 0x0F; // 4 primeros bits como salidas digitales.
        GPIO_PORTD_DATA_R =0x00;
      //Configuración del puerto D para PWM
      GPIO_PORTD_DEN_R |= 0x10; //BIT 4 DIGITAL
      GPIO_PORTD_DIR_R |= 0x10; //bit 4 SALIDA
      GPIO_PORTD_DATA_R |= 0x00; // SALIDA A 0
      GPIO_PORTD_AFSEL_R |= 0x10; //FUNCION ALTERNA EN BIT 4
      GPIO_PORTD_PCTL_R |= 0x00030000; //DIRIGIDO A T3CCP0

      //Configuración del Timer 3
      SYSCTL_RCGCTIMER_R |= 0X08; //HABILITA TIMER 4
      while ((SYSCTL_PRGPIO_R & 0X0008) == 0){};  // reloj listo?

      TIMER3_CTL_R=0X00000000; //DESHABILITA TIMER EN LA CONFIGURACION
      TIMER3_CFG_R= 0X0000004; //CONFIGURAR PARA 16 BITS
      TIMER3_TAMR_R= 0X0000000A; //CONFIGURAR PARA MODO PWM, MODO PERIODICO CUENTA HACIA ABAJO

      TIMER3_TAILR_R= 45000; // VALOR DE RECARGA 1 KHz
      TIMER3_TAMATCHR_R =40000; // 50 %
      TIMER3_TAPR_R= 0X00; // RELOJ 16 MHZ / 16 = 1 MHz
      TIMER3_CTL_R |= 0X00000001; //HABILITA TIMER A
  }
void main(void){

  DisableInterrupts();
  SYSCTL_RCGCGPIO_R = 0x210;  // 1) Habilita reloj para Puerto E, K
  while( (SYSCTL_PRGPIO_R & 0x210) ==0);

  GPIO_PORTE_DIR_R = 0x00;    // 2) PE4 entrada (anal�gica)
  GPIO_PORTE_AFSEL_R |= 0x10; // 3) Habilita Funci�n Alterna de PE4
  GPIO_PORTE_DEN_R = 0x00;    // 4) Deshabilita Funci�n Digital de PE4
  GPIO_PORTE_AMSEL_R |= 0x10; // 5) Habilita Funci�n Anal�gica de PE4

  GPIO_PORTK_DIR_R  = 0xFF; // puerto K todos los bits salidas
  GPIO_PORTK_DEN_R  = 0xFF;   // todos digitales.
  GPIO_PORTK_DATA_R = 0x00;

  // Habilita reloj en Puerto F
    SYSCTL_RCGCGPIO_R |= 0x20;      // (a)

    // Espera a que esté listo el puerto
    while((SYSCTL_PRGPIO_R & 0x20)==0);

    // Bits 4 y 0 del PTO F como salidas, los demas bits entradas
    GPIO_PORTF_DIR_R |= 0x01;       // (b)

    // Habilita el modo Digital de los pines 4 y 0
    GPIO_PORTF_DEN_R |= 0x01;           // (c)

  SYSCTL_RCGCADC_R  = 0x01;   // 6) Habilita reloj para l�gica de ADC0
  while((SYSCTL_PRADC_R & 0x01)==0);

  ADC0_PC_R = 0x01;       //  7) Configura para 129Ksamp/s --> Fconv/8
  ADC0_SSPRI_R = 0x3210;  //  8) SS3 con la m�s baja prioridad
  ADC0_ACTSS_R = 0x0000;  //  9) Deshabilita SS3 antes de cambiar configuraci�n de registros
  ADC0_EMUX_R = 0x5000;   // 10) iniciar muestreo sel SS3 por Timer
  ADC0_SSEMUX3_R = 0x00;    // 11) posibles Entradas AIN(15:0)
  ADC0_SSMUX3_R = 0x09;    //     en especifico canal AIN9-> PE4
  ADC0_SSCTL3_R = 0x0006; // 12) no TS0, D0 , yes IE0 END0
  ADC0_IM_R = 0x0008;     // 13) Habilita interrupciones de SS3
  ADC0_ACTSS_R |= 0x0008; // 14) Habilita SS3

  NVIC_EN0_R = 1 << 17;  // Habilita Interrupci�n del ADC0 Secuenciador 3

  SYSCTL_PLLFREQ0_R |= SYSCTL_PLLFREQ0_PLLPWR;  // encender PLL
  while((SYSCTL_PLLSTAT_R&0x01)==0);    // espera a que el PLL fije su frecuencia
  SYSCTL_PLLFREQ0_R &= ~SYSCTL_PLLFREQ0_PLLPWR; // apagar PLL

  ADC0_ISC_R = 0x0008;      // Se recomienda Limpia la bandera RIS del ADC0

  /*Configuracion del Timer*/

  SYSCTL_RCGCTIMER_R = 0x01;                // _Activa TIMER0
  while((SYSCTL_PRTIMER_R & 0x01)==0);

  TIMER0_CTL_R = 0X00000000; //  _disable timer0A during setup
  TIMER0_CFG_R = TIMER_CFG_16_BIT;          // _configure TIMER0 for 16-bit timer mode

  TIMER0_CTL_R |= TIMER_CTL_TAOTE;     // _enable timer0A trigger to ADC
  TIMER0_ADCEV_R = 0x1;                // el trigger de ADC es con Time Out de Timer

  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;   // _configure for periodic mode
  TIMER0_TAPR_R = 0;                        // prescale value for trigger
  TIMER0_TAILR_R = 4000;                    // start value for trigger
  TIMER0_IMR_R &= ~TIMER_IMR_TATOIM;        // _disable timeout (rollover) interrupt
  TIMER0_CTL_R |= TIMER_CTL_TAEN;           // _enable timer0A 16-b, periodic, no interrupts


  SysTick_Init(); // Inicializa SysTick
  PWM_init();
  GPIO_Init();  // Inicializa puertos e interrupciones en J y E
  EnableInterrupts();
  GPIO_PORTF_DATA_R |= 0x01;
  while(1){
      unsigned int i=0;
      //50 Hz
      //Period=20 ms= 320 000 ticks a 16MHz
      //CT 0-100%

      //Result=0-4000
      //unsigned int Delay = (result*5)+1000;//Tope del CT=15000
      unsigned int Delay = (result)+100;//Tope del CT=15000
      GPIO_PORTF_DATA_R &= 0x00;
      for(i=0; i<32000-Delay; i++){
      }
      GPIO_PORTF_DATA_R |= 0x01;
      for(i=0; i<Delay; i++){
      }
      TIMER3_CTL_R=0X00000000; //DESHABILITA TIMER EN LA CONFIGURACION
      TIMER3_TAMATCHR_R =(10*result); // 50 %
      TIMER3_CTL_R |= 0X00000001; //HABILITA TIMER A
      //if (result>1000){
      //TIMER3_TAILR_R= 45000; // VALOR DE RECARGA 1 KHz
      //TIMER3_TAMATCHR_R =40000; // 50 %
      //TIMER3_CTL_R=0X00000000; //DESHABILITA TIMER EN LA CONFIGURACION
      //TIMER3_TAMATCHR_R =(10*result); // 50 %
      //TIMER3_CTL_R |= 0X00000001; //HABILITA TIMER A
      //}
  }

}


void ADC0_HandleSS3(void)
{
  result = (ADC0_SSFIFO3_R & 0xFFF);  // Resultado en FIFO3 se asigna a variable "result"
  ADC0_ISC_R = 0x0008;                // Limpia la bandera RIS del ADC0
  GPIO_PORTK_DATA_R = (result>>4);    // despliega en 8 bits resultado en PTO K.

}

//******* Rutina de Servicio de Interrupción Puerto J*******
 void GPIOPortJ_Handler(void)
 {
  GPIO_PORTJ_ICR_R = 0x01;      // bandera0 de confirmación
  // Tiempo de retardo utilizando wait igual a 3 s
      int i=0;
      int j=0;
      for(i=0; i<5; i++){
      //SysTick_Wait(800000);  // Espera 200 ms (asume reloj de  4 MHz)
      for(j=0;j<800000;j++);
      GPIO_PORTN_DATA_R = GPIO_PORTN_DATA_R^0x03; // conmuta encendido de leds
}
    }

void DisableInterrupts(void){
    __asm(  " CPSID I ");
}

void EnableInterrupts(void){
    __asm( "  CPSIE  I  ");
}
















