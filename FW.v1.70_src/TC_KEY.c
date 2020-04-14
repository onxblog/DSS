//

#include <TC.h>

void TCinit(void)
{
// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 31,250 kHz

    //TCCR0 – Timer/Counter Control Register
        //Bit 2:0 – CS02:0: Clock Select
            //0 0 0 No clock source (Timer/Counter stopped).
            //0 0 1 clk I/O /(No prescaling)
            //0 1 0 clk I/O /8 (From prescaler)
            //0 1 1 clk I/O /64 (From prescaler)
            //1 0 0 clk I/O /256 (From prescaler)
            //1 0 1 clk I/O /1024 (From prescaler)
            //1 1 0 External clock source on T0 pin. Clock on falling edge.
            //1 1 1 External clock source on T0 pin. Clock on rising edge.

    TCCR0=(1<<CS02) | (0<<CS01) | (0<<CS00);

    //TCNT0 – Timer/Counter Register
    TCNT0=0x06;

    //TCCR1A – Timer/Counter 1 Control Register A
        //Bit 7:6 – COM1A1:0: Compare Output Mode for channel A
        //Bit 5:4 – COM1B1:0: Compare Output Mode for channel B
        //Bit 3 – FOC1A: Force Output Compare for channel A
        //Bit 2 – FOC1B: Force Output Compare for channel B
        //Bit 1:0 – WGM11:0: Waveform Generation Mode
    //TCCR1B – Timer/Counter 1 Control Register B
        //Bit 7 – ICNC1: Input Capture Noise Canceler
        //Bit 6 – ICES1: Input Capture Edge Select
        //Bit 5 – Reserved Bit
        //Bit 4:3 – WGM13:2: Waveform Generation Mode
        //Bit 2:0 – CS12:0: Clock Select
    //TCNT1H and TCNT1L – Timer/Counter 1
    //OCR1AH and OCR1AL – Output Compare Register 1 A
    //OCR1BH and OCR1BL – Output Compare Register 1 B
    //ICR1H and ICR1L – Input Capture Register 1
    
// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 125,000 kHz
// Mode: Ph. correct PWM top=ICR1
// OC1A output: Non-Inverted PWM
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 10 ms
// Output Pulse(s):
// OC1A Period: 10 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: On
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
    if (eHSM[3])
    //Для Clock value: 125,000 kHz і відповідно частоти ШІМ 100Гц
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
    else
    //Для Clock value: 7,812 kHz і відповідно частоти ШІМ 6,25Гц
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x02;
ICR1L=0x71;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x02;
OCR1BL=0x71;    

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 125,000 kHz
// Mode: Normal top=0xFF
// OC2 output: Disconnected
// Timer Period: 1,024 ms

    //TCCR2 – Timer/Counter Control Register
        //Bit 7 – FOC2: Force Output Compare
        //Bit 6,3 – WGM21:0: Waveform Generation Mode
        //Bit 5:4 – COM21:0: Compare Match Output Mode
        //Bit 2:0 – CS22:0: Clock Select
    //TCNT2 – Timer/Counter Register
    //OCR2 – Output Compare Register
    //ASSR – Asynchronous Status Register
        //Bit 3 – AS2: Asynchronous Timer/Counter2
        //Bit 2 – TCN2UB: Timer/Counter2 Update Busy
        //Bit 1 – OCR2UB: Output Compare Register2 Update Busy
        //Bit 0 – TCR2UB: Timer/Counter Control Register2 Update Busy

ASSR=0x00;
TCCR2=0x04;
TCNT2=0x80;
OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
    //TIMSK – Timer/Counter Interrupt Mask Register
        //Bit 0 – TOIE0: Timer/Counter0 Overflow Interrupt Enable
        //Bit 2 – TOIE1: Timer/Counter1, Overflow Interrupt Enable
        //Bit 3 – OCIE1B: Timer/Counter1, Output Compare B Match Interrupt Enable
        //Bit 4 – OCIE1A: Timer/Counter1, Output Compare A Match Interrupt Enable
        //Bit 5 – TICIE1: Timer/Counter1, Input Capture Interrupt Enable
        //Bit 6 – TOIE2: Timer/Counter2 Overflow Interrupt Enable
        //Bit 7 – OCIE2: Timer/Counter2 Output Compare Match Interrupt Enable

TIMSK=(0<<OCIE2) | (1<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (1<<OCIE1B) | (0<<TOIE1) | (1<<TOIE0);
}


//--------------------------------------------------------------------
// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
    TCNT0=0x06;                                         //переініціалізація таймера T=8ms
    
    static byte /*t_ss = 0,*/ t_blk = 0, t_pid = 0, nd = 0;
    static byte KeyCount = 0, OldKeyPress = 0, KeyDelay = 0;
    static unsigned int t_time = 0;
    static int ADCFirDispVal = 0;
    static unsigned int FirDisp[8], FirDisp_i;
    
    //PID calculate enabled
    if (++t_pid == T_PID)                               //Запуск ПІД розрахунку з періодом 200ms
    {
        ADCValPID = ((long)ADCVal*Kg/10L + (long)Ks*40L)/4;
        FirDisp_i = ADCValPID/10;
        
        //FIR filter for display
        ADCFirDispVal = ADCFirDispVal - FirDisp[nd] + FirDisp_i;
        FirDisp[nd] = FirDisp_i;
        if (++nd == 8) nd = 0;
        ADCDispVal = (ADCFirDispVal + 4)/8;
        
        enPID = 1;
        enErrCheck = 1;                                 //дозволяєм запуск функції детектора помилки
        t_pid = 0;
    }
    
    //Timer, SS
//    if (!isSS)                                          //якщо не СофтСтарт
//    {
        if (tRst)
        {
            DotDisp = 0;
            if (!--tRst)
            {
                t_time = 0;
                if (TimerEn) DotDisp = 3;
            }
        }

        if (isStab && tval && (++t_time == CPS0_M))     //відраховуєм 1 хвилину
        {
            tval--;
            t_time = 0;
        }
//    }
//    else                                                //якщо СофтСтарт
//    {
//        if (++t_ss == CPS0_S*Tss/1000)                  //затримка між запусками функції СофтСтарту
//        {
//            enSS = 1;
//            t_ss = 0;
//        }    
//    }

    //Button presed
/*    if (!KeyPress)                                      //якщо все оброблено або кнопка не натискалась
    {
        if (BT_OK)
        {
            KeyPress = 1;                               //якщо нажата кнопка енкодера
            if (OldKeyPress) KeyCount++;                //якщо попереднє значення кнопки дорівнює поточному то KeyCount++
            else KeyCount = 0;
            OldKeyPress = KeyPress;                     //попередньому значенню кнопки встановити поточне
            KeyPress = 0; 

            if (KeyCount == CPS0_S*T_COUNT/1000)        //якщо довге утримування кнопки
            {
                KeyPress = OK_LNG;                      //Довге натиснення на кнопку
                KeyCount = 0;
            }
        }
        else
        {
            if (OldKeyPress) KeyPress = OK_SHT;         //якщо перед цим нажмалась "ОК"
            KeyCount = 0;                       
            OldKeyPress = 0;
        }
    }
*/
    if (!KeyPress)                                          //якщо все оброблено (немає значення функції клавіші)
    {
        if (!KeyDelay)                                      //Якщо KeyDelay == 0, можна натискати знову
        {
            if (BT_SEL) KeyPress = KP_SEL_SHT;              //якщо кнопка "select/menu"
            if (BT_MNS)                                     //якщо кнопка "-"
            {
                if (BT_PLS && !KeyMode) KeyPress = KP_PLMN_SHT;             //кнопка "-" і "+"
                else KeyPress = KP_MINUS;                   //кнопка "-" коротке натискання
            }
            if (BT_PLS)                                     //якщо кнопка "+"
            {
                if (BT_MNS && !KeyMode) KeyPress = KP_PLMN_SHT;             //кнопка "-" і "+"
                else KeyPress = KP_PLUS;                    //кнопка "+" коротке натискання
            }
                
            if (KeyPress)                                   //якщо було нажато хоч щось
            {
                if (OldKeyPress == KeyPress) KeyCount++;    //якщо попереднє значення кнопки дорівнює поточному то KeyCount++
                    else KeyCount = 0;                      //інакше скидаємо лічильник утримування кнопки
                OldKeyPress = KeyPress;                     //попередньому значенню кнопки встановити поточне
                if (KeyCount == LPDC)                       //якщо довге утримування кнопки
                {
                    if (KeyPress == KP_MINUS || KeyPress == KP_PLUS)        //якщо "мінус" або "плюс"
                    {
                        if (!KeyMode)
                        {
                            KeyMode = KM_SEC;
                            KeyCount = 0;
                        }
                        else
                        {
                            KeyDelay = KD_SHT;              //встановити малу затримку опитування клавіш
                            KeyCount--;
                        }
                    }
                    else
                    {
                        if (KeyPress == KP_SEL_SHT) KeyPress = KP_SEL_LNG;  //довгий "select/menu"
                        KeyDelay = KD_NRM;                  //встановити нормальну затримку опитування клавіш
                        KeyCount = 0;
                    }
                }
                else KeyDelay = KD_NRM;                     //якщо коротка кнопка то встановити нормальну затримку опитування клавіш 

                if (KeyPress == KP_SEL_SHT) KeyPress = 0;   //скидання нажаття кнопок
                if (!KeyMode && (KeyPress == KP_MINUS || KeyPress == KP_PLUS)) KeyPress = 0;    //скидання нажаття кнопок
            }
            else                                            //якщо нічого не нажималось
            {
                if (OldKeyPress == KP_SEL_SHT) KeyPress = KP_SEL_SHT;       //"select/menu" - коротке нажаття
                if (!KeyMode)
                {
                    if (OldKeyPress == KP_PLUS) KeyPress = KP_PLUS;         //"плюс" - коротке нажаття
                    if (OldKeyPress == KP_MINUS) KeyPress = KP_MINUS;       //"мінус" - коротке нажаття
                }
                KeyCount = 0;                       
                OldKeyPress = 0;
            }
        }
        else KeyDelay--;                                                    //
    }

    //STAB delay
    if (!isStab && stab_delay)                          //затримка перед показом стабілізації температури (зелений діод)
    {
        if (--stab_delay == 0)
        {
            isStab = 1;
            enStabBeep = 1;
        }
    }
    
    //MODE delay
    if (m_delay) m_delay--;                             //якщо включена затримка режиму, зменшити її

    //BLINK delay
    if (!enBlk)                                         //якщо заборонено мигати
    {
        isBlk = 0;
        t_blk = 0;
    }
    else                                                //якщо доволено мигати
    {
        if (!BlinkDelay)
        {
            if (++t_blk == CPS0_S/4)                    //затримка між миганнями
            {
                isBlk = ~isBlk;                         //помигать
                t_blk = 0;
            }
        }
        else
        {
            isBlk = 0;
            BlinkDelay--;
        }
    }
    
    //Кормим собаку,- watchdog timer reset
    #asm("wdr")
}

//--------------------------------------------------------------------
// Timer1 overflow interrupt service routine
//interrupt [TIM1_OVF] void timer1_ovf_isr(void)
//{

//}

//--------------------------------------------------------------------
interrupt [TIM1_COMPB] void TC1compA_ovf(void)
{

//PORTC.1=1;
        //Start the AD conversion
        ADCSRA |= (1<<ADSC);
}

//--------------------------------------------------------------------
// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
    // Reinitialize Timer2 value
    TCNT2=0x80;

    static byte tDisp = 0;
    static byte CurDg=0;                                //поточний розряд
    static byte dSmb;                                   //поточний символ поточного розряду
    
    if (tDisp > 1)
    {
        if (!isBlk) dSmb = Disp[CurDg];                 //перевірка на мигання символом
        else
        {
            switch (CurDg)                              //вибір поточного розряду
            {
                case 0:
                {
                    if (enBlk > 2) dSmb = 10;
                    else dSmb = Disp[CurDg];
                    break;
                };
                case 1:
                {
                    if (enBlk > 1) dSmb = 10;
                    else dSmb = Disp[CurDg];                
                    break;
                };
                case 2:
                {
                    if (enBlk > 0) dSmb = 10;
                    else dSmb = Disp[CurDg];
                    break;
                };
            }
        }
        if (DispCmn == CAD)                                             //якщо індикатор з загальним анодом (CA)
        {
            SgA=1; SgB=1; SgC=1; SgD=1; SgE=1; SgF=1; SgG=1; SgP=1;     //вимикаємо сегменти
            Dg1=0; Dg2=0; Dg3=0;                                        //вимикаємо розряди

            switch (CurDg)                                              //вибір поточного розряду
            {
                case 0:{Dg1=1; break;};                                 //вмикаємо 1й розряд
                case 1:{Dg2=1; break;};                                 //вмикаємо 2й розряд
                case 2:{Dg3=1; break;};                                 //вмикаємо 3й розряд
            }

            SgA = ~(SymbolsCCD[dSmb]>>6)&0b0000001;
            SgB = ~(SymbolsCCD[dSmb]>>5)&0b0000001;
            SgC = ~(SymbolsCCD[dSmb]>>4)&0b0000001;
            SgD = ~(SymbolsCCD[dSmb]>>3)&0b0000001;
            SgE = ~(SymbolsCCD[dSmb]>>2)&0b0000001;
            SgF = ~(SymbolsCCD[dSmb]>>1)&0b0000001;
            SgG = ~(SymbolsCCD[dSmb]>>0)&0b0000001;
            
            switch (DotDisp)                                            //відобразити точку якщо потрібно
            {
                case 0:{SgP = 1; break;};
                case 3:{if (CurDg == 2) SgP = 0; break;};
                case 2:{if (CurDg == 1) SgP = 0; break;};
                case 1:{if (CurDg == 0) SgP = 0; break;};
            }
        }
        else                                                            //якщо індикатор з загальним катодом (CC)
        {
            SgA=0; SgB=0; SgC=0; SgD=0; SgE=0; SgF=0; SgG=0; SgP=0;     //вимикаємо сегменти
            Dg1=1; Dg2=1; Dg3=1;                                        //вимикаємо розряди

            switch (CurDg)                                              //вибір поточного розряду
            {
                case 0:{Dg1=0; break;};
                case 1:{Dg2=0; break;};
                case 2:{Dg3=0; break;};        
            }

            SgA = (SymbolsCCD[dSmb]>>6)&0b0000001;
            SgB = (SymbolsCCD[dSmb]>>5)&0b0000001;
            SgC = (SymbolsCCD[dSmb]>>4)&0b0000001;
            SgD = (SymbolsCCD[dSmb]>>3)&0b0000001;
            SgE = (SymbolsCCD[dSmb]>>2)&0b0000001;
            SgF = (SymbolsCCD[dSmb]>>1)&0b0000001;
            SgG = (SymbolsCCD[dSmb]>>0)&0b0000001;
            
            switch (DotDisp)                                            //відобразити точку якщо потрібно
            {
                case 0:{SgP = 0; break;};
                case 3:{if (CurDg == 2) SgP = 1; break;};
                case 2:{if (CurDg == 1) SgP = 1; break;};
                case 1:{if (CurDg == 0) SgP = 1; break;};
            }
        }

        if (CurDg<3) CurDg++; else CurDg=0;                             //перемикання розрядів
        tDisp = 0;
    }
    tDisp++;
}