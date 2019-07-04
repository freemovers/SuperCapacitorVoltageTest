#define F_CPU 1000000L
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#include <stdio.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

void RTC_init(void);
void USART0_init(void);
void CLKCTRL_init(void);
void ADC0_init(void);

static void USART0_sendChar(char c)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
    {
        ;                                   /* Wait for USART ready for receiving next char */
    }
    USART0.TXDATAL = c;
}

static int USART0_printChar(char c, FILE *stream)
{
    USART0_sendChar(c);
    return 0;
}

static FILE USART_stream = FDEV_SETUP_STREAM(USART0_printChar, NULL, _FDEV_SETUP_WRITE);

void USART0_init(void)
{

    PORTB.DIR &= ~PIN3_bm;                  /* Configure RX pin as an input */
    PORTB.DIR |= PIN2_bm;                   /* Configure TX pin as an output */

    USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600);

    USART0.CTRLB |= USART_TXEN_bm;          /* Transmitter Enable bit mask. */

    stdout = &USART_stream;                 /* Bind UART to stdio output stream */
}

void CLKCTRL_init(void)
{
#if (F_CPU == 1000000L)                     /* 1MHz, 16/16, set fuse for 16MHz */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_16X_gc | CLKCTRL_PEN_bm);
#elif (F_CPU == 20000000L)                  /* 20MHz, no division */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0 << CLKCTRL_PEN_bp);
#else                                       /* default 3.33MHz (20MHz/6) set fuse for 20MHz */
#endif
}

void RTC_init(void)
{
    /* Initialize RTC: */
    while (RTC.STATUS > 0)
    {
        ;                                   /* Wait for all register to be synchronized */
    }
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;      /* 32.768kHz Internal Crystal Oscillator (XOSC32K) */

    RTC.PITINTCTRL = RTC_PI_bm;             /* Periodic Interrupt: enabled */

    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc   /* RTC Clock Cycles 32768, resulting in 32.768kHz/32768 = 1Hz */
    | RTC_PITEN_bm;                         /* Enable: enabled */
}

void ADC0_init(void)
{
    /* The App Note AN2447 uses Atmel Start to configure Vref but we'll do it explicitly in our code*/
    VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;    /* Set the Vref to 1.1V*/

    /* The following section is directly taken from Microchip App Note AN2447 page 13*/

    ADC0.INTCTRL = 1 << ADC_RESRDY_bp       /* Result Ready Interrupt Enable: enabled */
    | 0 << ADC_WCMP_bp; /* Window Comparator Interrupt Enable: disabled */

    ADC0.CTRLC = ADC_PRESC_DIV4_gc          /* CLK_PER divided by 4 */
    | ADC_REFSEL_VDDREF_gc                  /* Vdd (Vcc) be ADC reference */
    | 0 << ADC_SAMPCAP_bp;                  /* Sample Capacitance Selection: disabled */

    ADC0.CTRLA = 1 << ADC_ENABLE_bp         /* ADC Enable: enabled */
    | 0 << ADC_FREERUN_bp                   /* ADC Free run mode: enabled */
    | ADC_RESSEL_10BIT_gc                   /* 10-bit mode */
    | 1 << ADC_RUNSTBY_bp;                  /* Run standby mode: enabled */

    ADC0.COMMAND |= 1;                      /* start running ADC */
}

ISR(RTC_PIT_vect)
{
    ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;     /* ADC internal reference, the Vbg */
    ADC0.COMMAND = ADC_STCONV_bm;
    RTC.PITINTFLAGS = RTC_PI_bm;            /* Clear flag by writing '1': */
}

ISR(ADC0_RESRDY_vect)
{
    int Vcc_value = 0;                      /* measured Vcc value */
    /* ADC result ready interrupt handling: start USART transmission */
    Vcc_value = ( 0x400 * 1100L ) / ADC0.RES; /* calculate the Vcc value */
    printf("Counter value is, %u \r\n", Vcc_value);

    while (!(USART0.STATUS & USART_TXCIF_bm))
    ;                                       /* wait for USART TX complete */
    USART0.STATUS = USART_TXCIF_bm;         /* Clear TXCIF flag */

    ADC0.INTFLAGS = ADC_RESRDY_bm;          /* The interrupt flag has to be cleared manually */
}

int main(void)
{
    CLKCTRL_init();
    USART0_init();
    RTC_init();
    ADC0_init();

    sei();                                  /* Enable Global Interrupts */

    set_sleep_mode(SLEEP_MODE_STANDBY);     /* Set sleep mode to STANDBY mode */
    sleep_enable();

    while (1)
    {
        sleep_cpu();                        /* Nothing to do here */
    }
}
