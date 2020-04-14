//

#include <ADC.h>

//ADMUX:
    //REFS1:0 - Reference Selection Bits
        //00 AREF, Internal Vref turned off
        //01 AVCC with external capacitor at AREF pin
        //10 Reserved
        //11 - Internal 2.56V Voltage Reference with external capacitor at AREF pin
    //ADLAR - ADC Left Adjust Result
    //MUX3:0 - Analog Channel Selection Bits
        //0000 - ADC0

//ADCSRA:
    //ADEN - ADC Enable
    //ADSC - ADC Start Conversion
    //ADFR - ADC Free Running Select
    //ADIF - ADC Interrupt Flag
    //ADIE - ADC Interrupt Enable
    //ADPS2:0 - ADC Prescaler Select Bits
        //110 - Division Factor 64

//ADCH, ADCL Conversion result (ADCW)

//--------------------------------------------------------------------
interrupt [ADC_INT] void ADCCmpl(void)
{
    getADC();
//PORTC.1 = 0;
}

//--------------------------------------------------------------------
void ADCinit (void)
{
    // ADC initialization
    // ADC Clock frequency: 125,000 kHz
    // ADC Voltage Reference: AREF pin
    // ADC Input 0
    //ADMUX = (0<<REFS1)|(0<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
    ADMUX = 0x00;
    //ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADFR)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
    ADCSRA = 0b10011110;
}

//--------------------------------------------------------------------
void getADC(void)
{
    static int FirADC[4];
    static byte i = 0;
    
    //FIR filter for ADC
    ADCVal = ADCVal - FirADC[i] + ADCW;
    FirADC[i] = ADCW;
    if (++i == 4) i = 0;
}