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

#define LED_BLINK 1      /* LED blink */
#define LED_ON 2         /* LED not blink */
#define LED_OFF 3        /* LED not blink */

#define DETECT_LV 1      /* Low-voltage detection */
#define NO_DETECT_LV 0   /* No Low-voltage detection */

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

void init_port(void)
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
    ADMUX = (0 << REFS0) | (1 << ADLAR) | (0 << MUX1) | (1 << MUX0); /* PB2 */
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

/* Output constant power level
   \retval 0 OK
   \retval -1 Voltage Drop
*/
int const_power(byte power, int duration, byte blink, byte lv_detect)
{
    int t, i;
    byte c, lv;
    
    output(power);
    
    if (blink == LED_ON)
        led_on(GREEN);
    else if (blink == LED_OFF)
        led_off(GREEN);
        
    c = 0;
	for (t = 0; (t < duration) || (duration == DEADLOOP); t += 10)
	{
        /* BLINK control */
        if (blink == LED_BLINK)
        {
            if ((c & 0x07) == 0x00)
               led_on(GREEN);
            else if ((c & 0x07) == 0x04)
               led_off(GREEN);
            
            c ++;
        }
        
        /* Voltage Drop detection */
        if (lv_detect == DETECT_LV)
        {
            lv = 1;
            for (i = 0; i < 10; i ++)
  	        {
                if (ADCH > 0x08)  /* Voltage gate: 0xff: 5v,  0x10: 0.31v */
                    lv = 0;
                
                delay(1);
            }
            if (lv == 1)
                return -1;
        }
        else
        {
            delay(10);
        }
    }
    return 0;
}

/* Transit from begin level to end level */
void transit_power(byte begin, byte end, int wait)
{
    int i, step;
    
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

    init_port();   
    init_devices(); 
    init_adc();

    /* fading(); */
    /* adctest(); */
    
    /* Phase 1: use full power level */
    const_power(POWER_FULL, 1500, LED_ON, NO_DETECT_LV);
    
    /* Phase 2: transit from full level to holding level*/
    transit_power(POWER_FULL, POWER_HOLD, 1);
    
    /* Phase 3: Hold on holding level */
    if (const_power(POWER_HOLD, DEADLOOP, LED_BLINK, DETECT_LV) < 0)
    {
        /* Recommended steps: 
           const_power(POWER_ZERO); 
           delay(sometime, e.g.50ms);
           const_power(max); */
        const_power(POWER_HOLD, DEADLOOP, LED_OFF, NO_DETECT_LV);
    }
}
