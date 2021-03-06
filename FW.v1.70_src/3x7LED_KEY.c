/*****************************************************
This program was produced by the
CodeWizardAVR V2.05.0 Professional
Automatic Program Generator
� Copyright 1998-2010 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Soldering station 3x7LED ENC
Version : 1.70
Date    : 14.09.2014
Author  : Victor Poteraylo
Company : 
Comments: 20.08.2013


Chip type               : ATmega8
Program type            : Application
AVR Core Clock frequency: 8,000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*****************************************************/
/*****************************************************
������� ز� ����� 2 �������� - 100�� � 6.25��, H �� L ��������
*****************************************************/

typedef unsigned char byte;

#include <mega8.h>
#include <delay.h>

#include <3x7LED_KEY.h>
#include <ADC.c>
#include <TC_KEY.c>

void main(void)
{
// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRB=(1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=1 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (1<<PORTB0);

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=Out Bit1=Out Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (0<<DDC0);
// State: Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=0 Bit1=0 Bit0=T 
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=Out Bit0=In 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (0<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=P Bit2=P Bit1=0 Bit0=P 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (1<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (1<<PORTD0);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
MCUCR=0x00;

// USART initialization
// USART disabled
UCSRB=0x00;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
SFIOR=0x00;

// SPI initialization
// SPI disabled
SPCR=0x00;

// TWI initialization
// TWI disabled
TWCR=0x00;

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/1024k
#pragma optsize-
WDTCR=0x1E;
WDTCR=0x0E;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

//Startup init-----------------------------------
    ADCinit();                                  //������������ ���
    TCinit();                                   //������������ �������-���������
    
    DispCmn = eDispCmn;                         //����������� �������� CA/CC � eeprom
    if (DispCmn == 255)
    {
        AutoSetDisp();
        StMode = SM_iDISP;
    }
    
    riTP = eiTP;                                //�������� ������� ������� ����������� � ���
    InitTemp();
    
    r_tval = e_tval;                            //�������� ������� � ���
    
    ADLdp = eADLdp;
    if (ADLdp) PwrTmRstEn = 1;
    else PwrTmRstEn = 0;
//    ADLdp_thr = MIN_ADLP + ADLdp;

    Ks = eKs;
    Kg = eKg;

    #asm("sei")
    
    if (BT_SEL)
    {
        if (StMode == SM_iDISP) StMode = SM_iD_SP;  //����� ������������ ���������� ���� ������� �� ���������� ���������
        else StMode = SM_SUP;                       //����� SetUP
        
        DotDisp = 1;                            //����� � ������� ������
        ReCode(NumVer,3);                       //������� ����� ����
        beep(250);
        
        while (BT_SEL) {};                       //����� ���� ������ ������
        KeyPress = 0;                           //������ ���� OK_LNG
        delay_ms(250);
        KeyPress = 0;                           //������ ���� OK_SHT
        
        isChng = 1;                             //�������� ���� ��������
        m_delay = 4*CPS0_S;                     //�������� ��������� ������
        
        DotDisp = 0;
    }
    else if (StMode != SM_iDISP) StartUp();
    
    //------------------------------------------------------------
    while (1)
    {
        //---Encoder&KEY------------------------------------------
        if (KeyPress)
        {
            if (StMode == SM_STB || StMode == SM_OFF)
            {
                KeyPress = KEY_TIMER;
                beep(50);
                //isSS = 1;
            }
            if (TimerEn) TimerReset();
        }
        if (StMode == KEY_TIMER && KeyPress) KeyPress = KEY_TIMER;

        switch (KeyPress)                                   //������� ����� �� �� ��������
        {
            case KP_MINUS:                                     //Minus
            {
                isChng = CHNG_PCM;                                 //�������� ���� ��������
                m_delay = 4*CPS0_S;                         //�������� ��������� ������ 5sec
                BlinkDelay = CPS0_S/4;                      //�������� ������� 0.25sec
                switch (StMode)
                {
                    case SM_MAIN:                              //�������� ������� �����
                    {
                        if (!KeyMode)
                        {
                            if (riTP) riTP--; else riTP = COUNT_PRESET - 1;    //������� ������ �������
                            DispVal = eTP[riTP];                             //������� �� ������� �������� �������
                        }
                        else
                        {
                            isChng = CHNG_TPS;                  //�������� ���� �������� �����������
                            if (DispVal > MIN_TEMP) DispVal--;  //����������� �� ������ ����� �� ��������� �� �������� ��
                            else beep(50);
                        }
                        break;
                    };
                    
                    case SM_WAIT_STB:
                    case SM_WAIT_OFF:
                    {
                        m_delay = 0;
                        StMode = SM_MAIN;
                        break;
                    };
                    case SM_TMR_SET:                        //����� ������������ �������� �������
                    {
                        if (DispVal) DispVal --;
                        else beep(50);
                        break;
                    };

                    case SM_SUP:                            //����� ���������
                    {
                        StMode = SM_HSM;
                        break;
                    };
                    case SM_CLB:                            //���������
                    case SM_HSM:
                    case SM_ADL:                            //�������� ��������� ���������
                    case SM_SBT:
                    {
                        if (StMode == SM_CLB) StMode = SM_SBT;
                        else StMode -= 2;
                        break;
                    };
                    case SM_CLB_SET:
                    {
                        switch (ClbMode)
                        {
                            case CLM_LPT:
                            {
                                if (clb_ct_val) clb_ct_val--;
                                else beep(50);
                                break;
                            };
                            case CLM_HPT:
                            {
                                if (clb_ct_val > (CLB_TOP - 100)) clb_ct_val--;
                                else beep(50);
                                break;
                            };
                        }
                        break;
                    };
                    case SM_HSM_SET:
                    case SM_ADL_SET:
                    {
                        if (DispVal) DispVal--;
                        else beep(50);
                        break;
                    };
                    case SM_SBT_SET:
                    {
                        if (DispVal > MIN_TEMP) DispVal--;
                        else beep(50);
                        break;
                    };

                    case SM_iDISP:
                    case SM_iD_SP:
                    {
                        if (DispCmn == CAD) DispCmn = CCD;  //CC
                        else DispCmn = CAD;                 //CA
                        break;
                    };
                }
                KeyPress = 0;                               //������ �������� ������ ������
                break;
            };

            case KP_PLUS:                                     //Plus - ��� ��������� "�����"
            {
                isChng = CHNG_PCM;                              //�������� ���� ��������
                m_delay = 4*CPS0_S;
                BlinkDelay = CPS0_S/4;                      //�������� ������� 0.25sec
                switch (StMode)
                {
                    case SM_MAIN:
                    {
                        if (!KeyMode)                           //���� ��������� ����� ������
                        {
                            if (++riTP == COUNT_PRESET) riTP = 0; //������� ������ �������
                            DispVal = eTP[riTP];                 //��������� ����������� �������
                        }
                        else
                        {
                            isChng = CHNG_TPS;                  //�������� ���� �������� �����������
                            if (DispVal < MAX_TEMP) DispVal++;  //����������� �� ������ ����� �� ��������� �� �������� ��
                            else beep(50);
                        }
                        
                    };
                    
                    case SM_WAIT_STB:
                    case SM_WAIT_OFF:
                    {
                        m_delay = 0;                 //�������� ��������� ������ 4sec
                        StMode = SM_MAIN;
                        break;
                    };                    
                    
                    case SM_TMR_SET:
                    {
                        if (DispVal < MAX_TIMER) DispVal++;     //���� �������� �� ����� �� �����������, ��������
                        else beep(50);
                        break;
                    };

                    case SM_SUP:
                    {
                        StMode = SM_ADL; 
                        break;
                    };
                    case SM_CLB:                            //���������
                    case SM_HSM:
                    case SM_ADL:                            //�������� ��������� ���������
                    case SM_SBT:                    
                    {
                        if (StMode == SM_SBT) StMode = SM_CLB;
                        else StMode += 2;
                        break;
                    };
                    case SM_CLB_SET:
                    {
                        switch (ClbMode)
                        {
                            case CLM_LPT:
                            {
                                if (clb_ct_val < 50) clb_ct_val++;
                                else beep(50);
                                break;
                            };
                            case CLM_HPT:
                            {
                                if (clb_ct_val < (CLB_TOP + 100)) clb_ct_val++;
                                else beep(50);
                                break;
                            };
                        }
                        break;
                    };
                    case SM_HSM_SET:
                    {
                        switch (SubMnuMode)
                        {
                            case HSM_KPS:
                            case HSM_KIS:
                            case HSM_KDS:
                            {
                                if (DispVal < 99) DispVal++;
                                else beep(50);
                                break;
                            };
                            case HSM_PFS:
                            {
                                if (!DispVal) DispVal = 1;
                                else beep(50);
                                break;
                            };
                            case HSM_MPS:
                            {
                                if (DispVal < MaxPWM) DispVal++;
                                else beep(50);
                                break;
                            };
                        }
                        break;
                    };
                    case SM_ADL_SET:
                    {
                        if (DispVal < 95) DispVal++;
                        else beep(50);
                        break;
                    };
                    case SM_SBT_SET:
                    {
                        if (DispVal < MAX_TEMP) DispVal++;
                        else beep(50);
                        break;
                    };

                    case SM_iDISP:
                    case SM_iD_SP:
                    {
                        if (DispCmn == CAD) DispCmn = CCD;  //CC
                        else DispCmn = CAD;                 //CA
                        break;
                    };
                }
                KeyPress = 0;
                break;
            };

            case KP_PLMN_SHT:                                   //������� ��������� - �� +
            {
                isChng = CHNG_PCM;                              //�������� �������� ����
                m_delay = 4*CPS0_S;                          //�������� ��������� ������ 4sec

                if (StMode == SM_MAIN) StMode = SM_WAIT_STB;
                KeyPress = KP_RESET;
                break;
            };
            
            case KP_SEL_SHT:                        //Ok short
            {
                if (LngKeyPress)                                //���� ���� ����� ����������
                {
                    LngKeyPress = KP_RESET;                            //�������� ������� ����������� ������
                                                                //(���������� ��������� ���������� ��� ���������� �������)
                    KeyPress = KP_RESET;                               //�������� ������� ��������� ������
                    break;
                }
                else
                {
                    m_delay = 4*CPS0_S;                          //�������� ��������� ������ 4sec
                    KeyMode = KM_SEC;
                }
                
                switch (StMode)
                {
                    case SM_MAIN:
                    {
                        if (!isChng)
                        {
                            isChng = CHNG_TSI;                  //������� ������� � ��������� ���� �������
                            DispVal = e_tval;                    //������� �� ����� ��������� ���������� ���� �������
                            StMode = SM_TMR_SET;               //������� � ����� ��������� ������� �������
                            beep(50);
                        }
                        break;
                    };
                    
                    case SM_TMR_SET:            //����� ������������ �������
                    {
                        if (isChng == CHNG_TSI)
                        {
                            StMode = SM_SUP;            //��������� � ����� ���������
                            isChng = CHNG_PCM;                  //��������� ��� - �������� ����
                        }
                        else m_delay = 0;            //0sec
                        
                        beep(50);
                        break;
                    };
                    
                    case SM_WAIT_STB:{StMode = SM_WAIT_OFF; break;};      //����������� �� �������� �������� �������� � ����
                    case SM_WAIT_OFF:{StMode = SM_WAIT_STB; break;};      //�� � ����� ����������

                    case SM_CLB:
                    {
                        ClbInit();
                        break;
                    };
                    case SM_CLB_SET:
                    {
                        switch (ClbMode)
                        {
                            case CLM_LPT:
                            {
                                ClbMode = CLM_LPT_OK; 
                                break;
                            };
                            case CLM_HPT:
                            {
                                ClbMode = CLM_HPT_OK;
                                break;
                            };
                        }
                        break;
                    };
                    case SM_HSM:
                    {
                        //m_delay = 5*CPS0_S;                     //�������� ������ � ��������� ������    5sec
                        isChng = CHNG_PCM;
                        StMode = SM_HSM_SET;
                        SubMnuMode = HSM_KPS;
                        DispVal = eHSM[0];                      //�������� Kp
                        beep(50);
                        break;
                    };
                    case SM_HSM_SET:
                    {
                        //m_delay = 5*CPS0_S;                     //�������� ������ � ��������� ������    5sec
                        isChng = CHNG_PCM;
                        switch (SubMnuMode)
                        {
                            case HSM_KPS:                       //��������� Kp
                            {
                                if (DispVal != eHSM[0])
                                {
                                    #asm("cli")
                                    eHSM[0] = DispVal;
                                    #asm("sei")
                                }
                                SubMnuMode = HSM_KIS;           //��������� Ki
                                DispVal = eHSM[1];
                                break;
                            };
                            case HSM_KIS:                       //��������� Ki
                            {
                                if (DispVal != eHSM[1])
                                {
                                    #asm("cli")
                                    eHSM[1] = DispVal;
                                    #asm("sei")
                                }
                                SubMnuMode = HSM_KDS;           //��������� Kd
                                DispVal = eHSM[2];
                                break;
                            };
                            case HSM_KDS:                       //��������� Kd
                            {
                                if (DispVal != eHSM[2])
                                {
                                    #asm("cli")
                                    eHSM[2] = DispVal;
                                    #asm("sei")
                                }
                                SubMnuMode = HSM_PFS;           //���� ������� ز�
                                DispVal = eHSM[3];
                                break;
                            };
                            case HSM_PFS:
                            {
                                if (DispVal != eHSM[3])
                                {
                                    #asm("cli")
                                    eHSM[3] = DispVal;
                                    #asm("sei")
                                    TCinit();
                                }
                                SubMnuMode = HSM_MPS;
                                DispVal = eHSM[4];
                                break;
                            };
                            case HSM_MPS:                       //��������� MAX_PWM
                            {
                                if (DispVal != eHSM[4])
                                {
                                    #asm("cli")
                                    eHSM[4] = DispVal;
                                    #asm("sei")
                                }
                                isChng = CHNG_RST;
                                m_delay = 0;
//                                beep(50);
                                StMode = SM_SUP;
                                break;
                            };
                        }
                        beep(50);
                        break;
                    };
                    case SM_ADL:
                    {
                        //m_delay = 4*CPS0_S;                     //�������� ������ � ��������� ������    5sec
                        isChng = CHNG_PCM;
                        StMode = SM_ADL_SET;
                        DispVal = ADLdp;
                        beep(50);
                        break;
                    };
                    case SM_ADL_SET:
                    {
                        if (ADLdp != DispVal)
                        {
                            #asm("cli")
                            eADLdp = DispVal;
                            #asm("sei")
                            ADLdp = eADLdp;
                        }
                        
                        if (ADLdp) PwrTmRstEn = 1;
                        else PwrTmRstEn = 0;
//                        ADLdp_thr = MIN_ADLP + ADLdp;

                        isChng = CHNG_RST;
                        m_delay = 0;                            //0sec
                        beep(50);
                        StMode = SM_SUP;
                        break;
                    };
                    case SM_SBT:
                    {
                        //m_delay = 5*CPS0_S;                     //�������� ������ � ��������� ������    5sec
                        isChng = 1;
                        StMode = SM_SBT_SET;
                        DispVal = eStbTemp;
                        beep(50);
                        break;
                    };
                    case SM_SBT_SET:
                    {
                        if (DispVal != eStbTemp)
                        {
                            #asm("cli")
                            eStbTemp = DispVal;
                            #asm("sei")
                        }
                        isChng = CHNG_RST;
                        m_delay = 0;                            //0sec
                        beep(50);
                        StMode = SM_SUP;
                        break;
                    };
                    
                    case SM_iDISP:
                    case SM_iD_SP:
                    {
                        #asm("cli")
                        eDispCmn = DispCmn;
                        #asm("sei")
                        beep(50);

                        if (StMode == SM_iD_SP)
                        {
                            StMode = SM_SUP;
                        }
                        else
                        {
                            isChng = CHNG_RST;
                            m_delay = 0;                            //0sec
                            StMode = SM_WAIT;
                            StartUp();
                        }
                        break;
                    };
                }
                KeyPress = KP_RESET;                   //�������� ������� ��������� ������
                chDispMode = 0;
                break;
            }; 

            case KP_SEL_LNG:                        //Ok long
            {
                switch (StMode)
                {
                    case SM_MAIN:
                    {
                        if (!KeyMode)                           //���� ��������� ����� ������
                        {
                            if (SetTemp != eTP[riTP])            //����������� ����������� �� ���� ���������� ��������� �������
                            {
                                #asm("cli")
                                eiTP = riTP;                     //�������� ������ ������� � ������
                                eTP[riTP] = SetTemp;             //�������� �������� ������� � ������
                                #asm("sei")
                            }
                        }
                        else                                    //���� ��������� ����� ������
                        {
                            if (DispVal != eTP[riTP])            //����������� �� ������ �� ���� ���������� ��������� �������
                            {
                                #asm("cli")
                                eiTP = riTP;                     //�������� ������ ������� � ������
                                eTP[riTP] = DispVal;             //�������� �������� ������� � ������
                                eLastT = 0;                  //
                                #asm("sei")
                                SetTemp = DispVal;              //������������ ������ �������� ����������� � �������
                            }
                        }
                        
                        StMode = SM_MAIN;
                        break;
                    };
                    
                    case SM_TMR_SET:            //��������� ������� �������
                    {
                        if (e_tval != DispVal)
                        {
                            r_tval = DispVal;
                            #asm("cli")
                            e_tval = r_tval;
                            #asm("sei")
                            TimerInit();                                                      
                            beep(50);
                        }
                        StMode = SM_MAIN;
                        break;
                    };
                    
                    case SM_SUP:
                    {
                        StMode = SM_WAIT;
                        LngKeyPress = 1;
                        beep(50);
                        delay_ms(30);
                        beep(50);
                        StartUp();
                        break;
                    };
                    
                    case SM_CLB:
                    {
                        Ks = 5; Kg = 48;
                        ClbInit();
                        break;
                    };
                }

                if (!LngKeyPress)
                {
                    beep(50);
                    delay_ms(30);
                    beep(50);
                }
                    
                KeyPress = KP_RESET;
                LngKeyPress = 1;            //������� ������� ���������� "��", ��� ���������� ������� "��"
                m_delay = 0;                //0sec
                isChng = CHNG_RST;
                chDispMode = 0;
                break;
            };

            case KEY_TIMER:
            {
                KeyPress = KP_RESET;
                StMode = KEY_TIMER;
                m_delay = CPS0_S/2;         //0.5 sec
                break;
            };
        }

        if (!m_delay && StMode == KEY_TIMER)
        {
            InitTemp();
            StMode = SM_MAIN;
        }
        
        if (!m_delay && isChng && !LngKeyPress)
        {
            switch (StMode)
            {
                case SM_MAIN:
                {
                    if ((eiTP != riTP) || (SetTemp != DispVal))
                    {
                        if (isChng == CHNG_TPS)
                        {
                            if (SetTemp != DispVal)
                            {
                                #asm("cli")
                                eLastT = DispVal;
                                #asm("sei")
                            }
                        }
                        else
                        {
                            if (eiTP != riTP)
                            {
                                #asm("cli")
                                eiTP = riTP;
                                eLastT = 0;
                                #asm("sei")
                            }
                        }
                        SetTemp = DispVal;
                        beep(50);
                    }
                    
                    StMode = SM_MAIN;
                    break;
                };  
                
                case SM_TMR_SET:            //��������� ������� �������
                {
                    if (e_tval != DispVal)
                    {
                        r_tval = DispVal;
                        #asm("cli")
                        e_tval = r_tval;
                        #asm("sei")
                        TimerInit();                                                      
                        beep(50);
                    }
                    StMode = SM_MAIN;
                    break;
                };
                case SM_WAIT_STB:
                {
                    beep(50);
                    SetStb();
                    break;
                };
                case SM_WAIT_OFF:
                {
                    beep(50);
                    SetOff();
                    break;
                };
                case SM_HSM:
                case SM_CLB:
                case SM_ADL:
                case SM_SBT:
                case SM_HSM_SET:
                case SM_ADL_SET:
                case SM_SBT_SET:
                {
                    StMode = SM_SUP;
                    break;
                };
            }
            isChng = CHNG_RST;
            chDispMode = 0;
            KeyMode = KM_PRM;
        }

        //---�������� ���-----------------------------------------
        switch (StMode)
        {
            case SM_STB:            //����� standby
            case SM_TMR_SET:        //��������� ������� �������
            case SM_WAIT_STB:       //�������� ����� ��������� � Stb ��� ������� �����
            case SM_WAIT_OFF:
            case SM_MAIN:
            {
                //---PID--------------------------------------------------
                if (/*!isSS &&*/ enPID)
                {
                    if (!TimerEn || (tval && !TimerStop))       //���� ������ ����������� ��� (��� �� 0 �� TimerStop=0)
                    {                                                                                      
                        ValPWM = PID_calc(SetTemp, ADCValPID);  //������ ������� ���������� ϲ�
//                        OCR1AH = (byte)(ValPWM>>8);         
//                        OCR1AL = (byte)ValPWM;
                        OCR1A = ValPWM;
                        if (TimerEn && PwrTmRstEn) PwrTmRst();  //�������� ������� ��� ������
                    }
                    else
                    {
//                        DotDisp = 0;
                        switch (TimerStop)
                        {
                            case 0:
                            {
                                beep(50);
                                delay_ms(30);
                                beep(50);
                                delay_ms(30);
                                beep(50);
                                SetStb();
                            };
                            case 1:
                            {
                                ValPWM = PID_calc(SetTemp, ADCValPID);  //������ ������� ���������� ϲ�
//                                OCR1AH = (byte)(ValPWM>>8);         
//                                OCR1AL = (byte)ValPWM;
                                OCR1A = ValPWM;
                                if (!tval) TimerStop = 2;               //OFF
                                break;
                            };
                            case 2:
                            {
                                beep(100);
                                delay_ms(30);
                                beep(100);
                                delay_ms(30);
                                beep(100);
                                SetOff();
                                break;
                            };
                        }
                    }
                    enPID = 0;
                }
                /*
                else if (isSS)                                  //���� ���������
                {
                    //---SS---------------------------------------------------
                    if (enSS)
                    {
                        ValPWM = SoftStart(Vss);                //������ �������� ����������, �������� - max. PWM [0-250]
                        OCR1AH = (byte)(ValPWM>>8);         
                        OCR1AL = (byte)ValPWM;
                        enSS = 0;
                    }
                    if (isSScmpl) isSS = 0;
                }
                //--------------------------------------------------------
                */

                //---Stab-------------------------------------------------
                if ((ADCDispVal >= SetTemp - StHyst)
                    &&(ADCDispVal <= SetTemp + StHyst))
                {
                    if (!stab_delay && !isStab) stab_delay = Tstab;
                    else if (isStab && enStabBeep)
                    {
                        if (StMode == SM_MAIN)
                        {
                            if (enStabBeep) beep(50);
                            enStabBeep = 0;
                            g_led();
                        }
                        StHyst = HT_WRK;
                    }
                }
                else
                {
                    stab_delay = 0;
                    isStab = 0;
                    StHyst = HT_LCK;
                    if (StMode == SM_MAIN) r_led();         //----!!!!!!!!!!!!!!!!!! �������� !!!!!!!!!!!
                }
                //--------------------------------------------------------
                break;
            };
            case SM_CLB_SET:
            {
                ClbSI();
                break;
            };
        }

        if (enErrCheck) ErrDetect();                    //---ERROR Detect            

        //---DISP-------------------------------------------------
        switch (StMode)                                             //���� ������ �������
        {
            case SM_MAIN:
            {
                //if (isSS || isChng) enBlk = 3; else enBlk = 0;
                //if (!isChng) DispVal = ADCDispVal;
                if (isChng) enBlk = 3;
                else 
                {
                    enBlk = 0;
                    DispVal = ADCDispVal;
                }
                ReCode(DispVal, 3);
                break;
            };
            case SM_TMR_SET:                                        //����� ������������ ������� �������
            {
                enBlk = 2;                                          //������� �������� ��� 2� �������
                Disp[0]= 13;    //t
                ReCode(DispVal, 2);
                break;
            };
            case SM_STB:                                            //����� ��������
            case SM_WAIT_STB:
            {
                if ((StMode == SM_STB) && isStab) enBlk = 0;        //������� ��������
                else enBlk = 3;                                     //������� �������� ��� 3� �������
                Disp[0]= 5;     //S
                Disp[1]= 13;    //t
                Disp[2]= 14;    //b
//                led_off();
                break;
            };
            case SM_CLB:                                            //���������
            {
                enBlk = 0;
                Disp[0]= 17;    //C
                Disp[1]= 24;    //L
                Disp[2]= 14;    //b
                break;
            };
            case SM_CLB_SET:                                        //����� ���������
            {
                switch (ClbMode)
                {
                    case CLM_BGN: {break;};
                    case CLM_HT:
                    {
                        enBlk = 0;
                        ReCode(ADCDispVal, 3);
                        break;
                    };
                    default:
                    {
                        enBlk = 3;
                        ReCode(clb_ct_val, 3);
                        break;
                    };
                }
                break;
            };
            case SM_HSM:                                            //��������� ��������� ������ [Htr]
            {
                enBlk = 0;
                Disp[0]= 25;    //H
                Disp[1]= 13;    //t
                Disp[2]= 12;    //r
                break;
            };
            case SM_HSM_SET:                            //����� ��������� ����������� ��������� ������
            {
                enBlk = 2;                              //������� �������� ��� 2� �������
                switch (SubMnuMode)
                {
                    case HSM_KPS:                       //��������� Kp
                    {
                        Disp[0]= 18;    //P
                        ReCode(DispVal, 2);
                        break;
                    };
                    case HSM_KIS:                       //��������� Ki
                    {
                        Disp[0]= 1;     //I
                        ReCode(DispVal, 2);
                        break;
                    };
                    case HSM_KDS:                       //��������� Kd
                    {
                        Disp[0]= 19;    //d
                        ReCode(DispVal, 2);
                        break;
                    };
                    case HSM_PFS:
                    {
                        enBlk = 1;                      //������� �������� ��� 1 �������
                        Disp[0]= 15;    //F
                        Disp[1]= 12;    //r[equency]
//                        Disp[2]= DispVal + 26;
//                        ReCode(DispVal, 1);
                        if (DispVal) Disp[2]= 25;   //H[igh]
                        else Disp[2]= 24;           //L[ow]
                        break;
                    };
                    case HSM_MPS:                       //��������� MAX_PWM
                    {
                        Disp[0]= 20;    //U
                        ReCode(DispVal, 2);
                        break;
                    };
                }
                break;
            };
            case SM_ADL:
            {
                enBlk = 0;
                Disp[0]= 21;    //A
                Disp[1]= 19;    //d
                Disp[2]= 24;    //L
                break;
            };
            case SM_ADL_SET:
            {
                enBlk = 2;
                Disp[0]= 24;    //L
                ReCode(DispVal, 2);
                break;
            };
            case SM_SBT:
            {
                enBlk = 0;
                Disp[0]= 5;     //S[tand]
                Disp[1]= 14;    //b[y]
                Disp[2]= 13;    //t[emperature]                
                break;
            };
            case SM_SBT_SET:
            {
                enBlk = 3;
                ReCode(DispVal, 3);                
                break;
            };
            
            case SM_OFF:                                //����� OFF (0)
            case SM_WAIT_OFF:
            {
                if (StMode == SM_OFF) enBlk = 0;        //������� ��������
                else enBlk = 3;
                Disp[0]= 0;     //O
                Disp[1]= 15;    //f
                Disp[2]= 15;    //f
//                led_off();                              //��. ��� ��������
                break;
            };
            case SM_SUP:                                //����� SetUP
            {
                enBlk = 3; 
                Disp[0]= 5;     //S
                Disp[1]= 11;    //E
                Disp[2]= 13;    //t
                break;
            };
            case SM_iDISP:
            case SM_iD_SP:
            {
                enBlk = 3;
                if (DispCmn == CCD)
                { 
                    Disp[0]= 17;    //C
                    Disp[1]= 17;    //C
                    Disp[2]= 19;    //d
                }
                else
                { 
                    Disp[0]= 17;    //C
                    Disp[1]= 21;    //A
                    Disp[2]= 19;    //d
                }
                break;
            };
        }
    }
}

//--------------------------------------------------------------------
void ReCode(unsigned int tmp, byte sn)
{
    Disp[2]= tmp%10;                //����� �� 10 ������� � ����� 3-������
    if (sn < 2) return;
    tmp = tmp/10;                   //��������� 2 �������
    if (!tmp)                       //�������� �� 0
    {
        Disp[1]= 10;                //����� 2� ������
        if (sn == 3) Disp[0]= 10;   //����� 1� ������
        return;
    }
    Disp[1]= tmp%10;                //����� �� 10 ������� � ����� 2-������
    if (sn < 3) return;
    tmp = tmp/10;                   //������� 1 ������
    if (tmp) Disp[0]= tmp;          //�������� �� 0
    else Disp[0]= 10;               //����� 1� ������
}

//--------------------------------------------------------------------
void beep(byte time_ms)
{
    BUZ_ON;
    delay_ms(time_ms);
    BUZ_OFF;
}

//--------------------------------------------------------------------
void g_led(void)
{
    lGrn = 1;
    lRed = 0;
}

//--------------------------------------------------------------------
void r_led(void)
{
    lGrn = 0;
    lRed = 1;
}

//--------------------------------------------------------------------
void led_off(void)
{
    lGrn = 0;
    lRed = 0;
}

//--------------------------------------------------------------------
void PID_init(void)
{
    Kp = eHSM[0];
    Ki = eHSM[1];
    Kd = eHSM[2];
    LimMAX = (long)eHSM[4]*625*256/100;
    
    Tstab = T_STAB*CPS0_S;                      //��� ����������� �����������

    PID_calc(0,0);
}

//--------------------------------------------------------------------
int PID_calc(int tSet, int tInp)
{
   
    static int tInp_p = 0, tInp_pp = 0;         //�������� ���������� �� �������������� ������ (�������) �����������
    static long Pout = 0;
    static int error;
  
   
    //���������� �������
    tSet = tSet*10;
    error = tSet - tInp;
    //���������� ����������� �������� Pc
    Pout = Pout - (long)Kp * (tInp - tInp_p)*8;
    //���������� ����������� �������� Ic
    Pout = Pout + (long)Ki * error/2;
    //���������� ������������ �������� Dc                            
    Pout = Pout - (long)Kd * (tInp - tInp_p*2L + tInp_pp)*4;
    
    //��������� �������
    tInp_pp = tInp_p;
    tInp_p = tInp;
    if (!tSet) Pout = 0;
    
    //��������� ��������� �������� ���������
    if (Pout < 0) Pout = 0;
    if (tInp < 1600) Pout = (LimMAX * ((long)tInp + 200L)) / 1800L;  //160 �������
    if (Pout > LimMAX) Pout = LimMAX;

    return (Pout>>8);
}

/*
//--------------------------------------------------------------------
int SoftStart(int dPWM)
{
    static int retPWM; 
    static byte k, add, fcall = 1;
    
    if (fcall)
    {
        if (ADCDispVal < MIN_TEMP - 15)
        { 
            k = 40;
            retPWM=0;
            isSScmpl = 0;
            add = dPWM/k;

            fcall = 0;
        }
        else
        {
            isSScmpl = 1;
            return 0;
        }
    }                    
    else if (k>0)
         {
             retPWM += add;
             k--;            
         }
         else
         {
             fcall = 1;
             isSScmpl = 1;
         }

    if (retPWM > dPWM) retPWM = dPWM;
    else if (retPWM < 0) retPWM = 0;

    return retPWM;
}
*/

//--------------------------------------------------------------------
void TimerReset(void)
{
    tval = r_tval;
    TimerStop = 0;
    tRst = CPS0_S/4;
}

//--------------------------------------------------------------------
void TimerInit(void)
{
    if (r_tval)                 //���� �� 0 
    {
        TimerEn = 1;            //��������� ������
        TimerReset();           //������� ������
    }
    else TimerEn = 0;           //���������� ������
}

//--------------------------------------------------------------------
void ClbSI(void)
{
    switch (ClbMode)
    {
        case CLM_BGN:
        {
            OCR1AH = 0x00;         
            OCR1AL = 0x00;
            
            beep(200);
            delay_ms(100);
            beep(200);
                
            clb_ct_val = ADCDispVal;
            clb_t_val = clb_ct_val;
                
            ClbMode = CLM_LPT;
            StHyst = HT_CLB;                    //��������� ���������� �����������

            break;
        };
        case CLM_LPT_OK:
        {
            if (clb_ct_val > clb_t_val) Ks += clb_ct_val - clb_t_val;
            else if (clb_ct_val < clb_t_val) Ks -= clb_t_val - clb_ct_val;
            #asm("cli")
            eKs = Ks;
            #asm("sei")
            beep(250);
            delay_ms(250);
            r_led();
            ClbMode = CLM_HT;
            //isSS = 1;
            break;
        };
        case CLM_HPT_OK:
        {
            Kg = ((clb_ct_val - Ks)*100)/clb_t_val;
            #asm("cli")
            eKg = Kg;
            #asm("sei")
            beep(250);
            delay_ms(250);
            ClbMode = CLM_OK;
            break;
        };
        case CLM_OK:
        {
            StHyst = HT_LCK;
            ClbMode = CLM_BGN;
            OCR1AH = 0x00;         
            OCR1AL = 0x00;
            led_off();
            StMode = SM_SUP;
            break;
        };
    }

    if (ClbMode > CLM_LPT_OK)
    {
        /*if (isSS)
        {
            if (enSS)
            {
                ValPWM = SoftStart(Vss);            //������ �������� ����������, �������� - max. PWM [0-1023]
                OCR1AH = (byte)(ValPWM>>8);         
                OCR1AL = (byte)ValPWM;
                enSS = 0;
            }
            if (isSScmpl) isSS = 0;
        }
        else*/ if (enPID)
        {
            ValPWM = PID_calc(CLB_TOP, ADCValPID);
            OCR1AH = (byte)(ValPWM>>8);
            OCR1AL = (byte)ValPWM;

            if ((ADCDispVal >= CLB_TOP - StHyst)
                &&(ADCDispVal <= CLB_TOP + StHyst))
            {
                if (!stab_delay && !isStab) stab_delay = Tstab;
                else if (isStab)
                {
                    if (enStabBeep && (ClbMode == CLM_HT))
                    {
                        beep(100);
                        delay_ms(100);
                        beep(250);

                        clb_ct_val = CLB_TOP;
                        clb_t_val = ((CLB_TOP - Ks)*100)/Kg;
                        ClbMode = CLM_HPT;
                    }
                    enStabBeep = 0;
                    StHyst = HT_WRK;
                    g_led();
                }
            }
            else
            {
                stab_delay = 0;
                isStab = 0;
                StHyst = HT_LCK;
                r_led();
            }
            enPID = 0;
        }
    }
}

//--------------------------------------------------------------------
void ErrDetect(void)
{
    static byte ErrLockDelayMT = 0;
    static byte ErrLockDelayDT = 0;
    static unsigned int OldADCvalPID = 0;    

    if (ADCValPID > ERR_MAX_TEMP*10)
    {
        if (ErrLockDelayMT < ERR_DELAY_MT) ErrLockDelayMT++;
    }
    else ErrLockDelayMT = 0;
    
    if (!isStab && (ADCValPID == OldADCvalPID) && StMode == SM_MAIN)
    {
        if (ErrLockDelayDT < ERR_DELAY_DT) ErrLockDelayDT++;
    }    
    else ErrLockDelayDT = 0;
    OldADCvalPID = ADCValPID;
    
    if (ErrLockDelayMT == ERR_DELAY_MT)
    {
        while (ErrLockDelayMT)
        {
            ErrPrc();    
            if (ADCValPID <= ERR_MAX_TEMP*10)
            {
                ErrLockDelayMT -= 1;
                led_off();
                //isSS = 1;
            }
        }
    }
    
    if (ErrLockDelayDT == ERR_DELAY_DT)
    {
        while (ErrLockDelayDT)
        {
            ErrPrc();    
            if (ADCValPID != OldADCvalPID)
            {
                ErrLockDelayDT >>= 1;
                led_off();
                //isSS = 1;
            }
        }
    }
    
    enErrCheck = 0;
}

//--------------------------------------------------------------------
void ErrPrc(void)
{
    OCR1AH = 0x00;         
    OCR1AL = 0x00;

    enBlk = 3;

    Disp[0]= 11;                        //E
    Disp[1]= 12;                        //r
    Disp[2]= 12;                        //r

    r_led();
    beep(200);
    g_led();
    delay_ms(200);
    
    KeyPress = 0;
}

//--------------------------------------------------------------------
void PwrTmRst(void)
{
    static int Fir[10], p[20];
    static int Sum, p1, dp, Fn = 0, pn = 0;
    int pf = 0;
    
    //FIR-������:
    Sum = Sum - Fir[Fn] + ValPWM;
    Fir[Fn] = ValPWM;
    if(++Fn == 10) Fn = 0;
    pf = Sum;
    //�������� ��������������:
    p1 = p[pn];
    p[pn] = pf;
    dp = pf - p1;
    if(++pn == 20) pn = 0;
    if(/*dp < ADLdp || */dp > ADLdp) TimerReset();
    //if(dp > ADLdp_thr) TimerReset();
}

//--------------------------------------------------------------------
void AutoSetDisp(void)
{
    DDRB.7 = 0;                         //��������� ���� �� ����
    PORTB.7 = 1;                        //������ �������
    delay_ms(10);
    if (PINB.7 == 0) DispCmn = CCD;     //CC
    else DispCmn = CAD;                 //CA
    DDRB.7 = 1;
    PORTB.7 = 0;
}

//--------------------------------------------------------------------
void StartUp(void)
{
    TCinit();
    enBlk = 0;
    DispVal = SetTemp;
    ReCode(DispVal, 3);
    delay_ms(1500);
    beep(50);                           //�������
    TimerInit();
    PID_init();
    //isSS = 1;                           //1- ���������� ���������
    StMode = SM_MAIN;
}

//--------------------------------------------------------------------
void InitTemp(void)
{
    if (eLastT)                                 //�����: Lt (Last temperature)
        SetTemp = eLastT;                       //����������� ������� �����������         
    else SetTemp = eTP[riTP];
}

//--------------------------------------------------------------------
void ClbInit(void)
{
    if (ADCDispVal <= 50)       //��������, �� ��������� ����������� �� ����� �� 50 �������
    {
        StMode = SM_CLB_SET;
        ClbMode = CLM_BGN;
        PID_init();
    }
    else
    {
        Disp[0]= 11;    //E
        Disp[1]= 12;    //r
        Disp[2]= 12;    //r
        beep(200);
        delay_ms(300);
    }
}

//--------------------------------------------------------------------
void SetStb(void)
{
    DotDisp = 0;
    TimerStop = 1;                  //����� ��������
    StMode = SM_STB;                //����� �������� SM_STB
    isStab = 0;
    tval = r_tval *2;               //��������� ������ �� ��� �� ��� x2
    SetTemp = eStbTemp;
    led_off();
}

//--------------------------------------------------------------------
void SetOff(void)
{
    DotDisp = 0;
    TimerStop = 3;                  //����� OFF
    StMode = SM_OFF;                //����� SM_OFF
    OCR1AH = 0x00;         
    OCR1AL = 0x00;
    led_off();
}