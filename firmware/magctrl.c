/*
 * magctrl.c
 *
 * Created: 2014/9/21 21:17:06
 *  Author: whowe
 */ 

#define F_CPU 4800000

#include <avr/signature.h>
const char fusedata[] __attribute__ ((section (".fuse"))) =
{0x75, 0xFF};
const char lockbits[] __attribute__ ((section (".lockbits"))) =
{0xFC};
const char eeprom[] __attribute__ ((section (".eeprom"))) =
{0x00, 88};
	
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define GREEN PORTB3

#define RC_CAL_ADDR 0
#define RC_MAGIC_ADDR 1

#define RC_MAGIC 88

#define POWER_FULL 10
#define POWER_HOLD 50
#define POWER_MIN 255

#define INFINITE -1

#define LED_BLINK 1      /* LED blink */
#define LED_ON 2         /* LED not blink */
#define LED_OFF 3        /* LED not blink */

#define DETECT_LV 1      /* Low-voltage detection */
#define NO_DETECT_LV 0   /* No Low-voltage detection */

typedef unsigned char byte;

void init_rc();
void init_port();
void init_adc();
void init_pwm();

void led_on(byte led);
void led_off(byte led);
void output_pwm(int pow);

/* Output constant power level
    \param power power (PWM) value 
    \param duration duration in ms, or INFINITE
    \param blink LED_BLINK, LED_ON or LED_OFF
    \param lv_detect DETECT_LV or NO_DETECT_LV
    \retval 0 OK, -1 Voltage Drop
*/
int const_power(byte power, int duration, byte blink, byte lv_detect);

/* Transit power level from begin to end
    \param begin begin value
    \param end end value
    \param wait delay time for each transit step
*/
void transit_power(byte begin, byte end, int wait);

void init_rc(void)
{
    unsigned char magic;
    unsigned char cal;
    
    magic = eeprom_read_byte((unsigned char *)RC_MAGIC_ADDR);
    cal = eeprom_read_byte((unsigned char *)RC_CAL_ADDR);
    
    if (magic == RC_MAGIC)
        OSCCAL = cal;
}

void init_port(void)
{
    PORTB = 0x00;    /* Clear Output */
    DDRB = 1 << PORTB0 | 1 << GREEN; /* PWM and LED */
}

void init_pwm(void)
{
    OCR0A = POWER_MIN; /* Minimal power when init */
    TCCR0A = 2 << COM0A0 | 1 << WGM00 ;  /* Phase Correct PWM */
    TCCR0B = 1 << CS00;  /* No Prescaler */
}

void init_adc(void)
{
    ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADPS1) | (1 << ADPS2);
    ADCSRB = 0x00;  /* Free running */
    ADMUX = (0 << REFS0) | (1 << ADLAR) | (1 << MUX0); /* PB2 */
    DIDR0 = (1 << ADC2D);
    ADCSRA |= (1 << ADSC);  
}

void led_on(byte led)
{
    PORTB |= (1 << led);
}

void led_off(byte led)
{
    PORTB &= ~(1 << led);
}

void output_pwm(int pow)
{
    OCR0A = pow;
}

/* Output constant power level
   \retval 0 OK
   \retval -1 Voltage Drop
*/
int const_power(byte power, int duration, byte blink, byte lv_detect)
{
    int t, i;
    byte c, lv;
    
    output_pwm(power);
    
    if (blink == LED_ON)
        led_on(GREEN);
    else if (blink == LED_OFF)
        led_off(GREEN);
        
    c = 0;
	for (t = 0; (t < duration) || (duration == INFINITE); t += 10)
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
                
                _delay_ms(1);
            }
            if (lv == 1)
                return -1;
        }
        else
        {
            _delay_ms(10);
        }
    }
    return 0;
}

/* Transit from begin level to end level */
void transit_power(byte begin, byte end, int wait)
{
    int i, j, step;
    
    if (end > begin)
        step = 1;
    else
        step = -1;
        
    for (i = begin; i != end; i += step)
    {
        output_pwm(i);
        
        for (j = 0; j < wait; j ++)
            _delay_ms(1);
    }
}

/* Main routine */
int main(void)
{
    /* Initialize everything */
    
    init_rc();
    init_port();   
    init_pwm(); 
    
    //init_adc();

    // fading();
    // adctest();
    
    /* Phase 1: use full power level */
    const_power(POWER_FULL, 2000, LED_ON, NO_DETECT_LV);
    
    /* Phase 2: transit from full level to holding level*/
    transit_power(POWER_FULL, POWER_HOLD, 1);
    
    /* Phase 3: Hold on holding level */
    const_power(POWER_HOLD, INFINITE, LED_BLINK, NO_DETECT_LV);
    
    return 0;
}



/* Fading test */
void fading(void)
{
    int i;
    
    while (1)
    {
        for (i = 255; i > 0; i --)
        {
            led_on(GREEN);
            output_pwm(i);
            _delay_ms(30);
        }
        for (i = 0; i < 255; i ++)
        {
            led_off(GREEN);
            output_pwm(i);
            _delay_ms(30);
        }
    }
}

/* ADC test */
void adctest(void)
{
    byte adc, old = 0;
    
    while (1)
    {
        adc = ADCH;
        output_pwm(adc);
        if (old != adc)
        {
            led_on(GREEN);
        }
        old = adc;
        _delay_ms(25);
        led_off(GREEN);
    }
}
