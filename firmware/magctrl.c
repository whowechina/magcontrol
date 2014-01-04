//ICC-AVR application builder : 2013/9/5 21:58:14
// Target : T13
// Crystal: 9.6000Mhz

#include <iot13v.h>
#include <macros.h>

#define GREEN PB3

#define POWER_ZERO 255

#define POWER_FULL 15
#define POWER_HOLD 170


#define DEADLOOP -1

#define BLINK 1          /* LED blink */
#define NO_BLINK 0       /* LED not blink */

#define ADC_ADJ 1        /* Adjust by ADC */
#define NO_ADC_ADJ 0     /* Not adjust by ADC */

typedef unsigned char byte;

void led_on(byte led);
void led_off(byte led);
void output(int pow);

void led_on(byte led)
{
    PORTB |= (1 << led);
}

void led_off(byte led)
{
    PORTB &= ~(1 << led);
}

void output(int pow)
{
    OCR0A = pow;
}

void delay(int ms)
{
     int i, j;
     for (i = 0; i < ms; i ++)
  	    for (j = 0; j < 573; j ++)
	  	  /* NOP */;
}

void port_init(void)
{
    PORTB = 0x00;
    DDRB = 0x00;
    DDRB |= (1 << PB0);
    DDRB |= (1 << GREEN);
}

void init_devices(void)
{
    CLI();
    MCUCR = 0x00;
    TIMSK0 = 0x00;
    GIMSK = 0x00;

    output(POWER_ZERO);

    TCCR0B = 0;
    TCCR0A = (0 << WGM01) | (1 << WGM00) ;  /* Phase Correct PWM */
    TCCR0B |= (0 << WGM02) | (0 << CS01) | (1 << CS00);  /* No Prescaling */
    TCCR0A |= ((1 << COM0A1) | (0 << COM0A0));
 
    SEI();
}

void init_adc(void)
{
    ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADPS1) | (1 << ADPS2);
    ADCSRB = 0x00;  /* Free running */
    ADMUX = (0 << REFS0) | (1 << ADLAR) | (1 << MUX1) | (0 << MUX0); /* PB4 */
    DIDR0 = (1 << ADC2D);
    ADCSRA |= (1 << ADSC);  
}


/* Fading test */
void fading(void)
{
    int i;
    while (1)
    {
        for (i = 255; i > 5; i --)
        {
            led_on(GREEN);
            output(i);
            delay(30);
        }
        for (i = 5; i < 255; i ++)
        {
            led_off(GREEN);
            output(i);
            delay(30);
        }
    }
}

/* ADC test */
void adctest(void)
{
    byte adc, old;
  
    while (1)
    {
        adc = ADCH;
        output(adc);
        if (old != adc)
        {
            led_on(GREEN);
        }
        old = adc;
        delay(25);
        led_off(GREEN);
    }
}

/* Get ADC adjustments [-47..47] */
int adc_adjustment(void)
{
    int adc;
    
    adc = ADCH >> 2;
    adc += (adc >> 1);
    
    return adc - 47;
}

/* Output constant power level */
void const_power(byte power, int duration, byte blink, byte adc_adj)
{
    int t;

	for (t = 0; (t < duration) || (duration == DEADLOOP); t += 100)
	{
        if (adc_adj == ADC_ADJ)
            output(power + adc_adjustment());
        else
            output(power);

        led_on(GREEN);
        
	    delay(50);
        
        if (blink == BLINK)
            led_off(GREEN);
        
        delay(50);
    }
}

/* Transit from begin level to end level */
void transit_power(byte begin, byte end, int wait)
{
    int i;
    char step;
    
    if (end > begin)
        step = 1;
    else
        step = -1;
        
    for (i = begin; i != end; i += step)
    {
        output(i);
        delay(wait);
    }
}

/* Main routine */
void main(void)
{
    /* Initialize everything */
    
    port_init();   
    init_devices(); 
    init_adc();

    /* fading(); */
    /* adctest(); */
    
    /* Phase 1: use full power level */
    const_power(POWER_FULL, 1500, NO_BLINK, NO_ADC_ADJ);
    
    /* Phase 2: transit from full level to holding level*/
    transit_power(POWER_FULL, POWER_HOLD + adc_adjustment(), 1);
    
    /* Phase 3: Hold on holding level */
    const_power(POWER_HOLD, DEADLOOP, BLINK, ADC_ADJ);
}
