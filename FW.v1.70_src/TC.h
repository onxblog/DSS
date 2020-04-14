//

#define CPS0_S  125         //ʳ������ ����������� �������0 �� 1 �������
#define CPS0_M  7500        //ʳ������ ����������� �������0 �� 1 �������
//#define TC1OSC      100             //ʳ������ ����������� �������1 �� 1 �������
//#define TC1OMC      TC1OSC*60       //ʳ������ ����������� �������1 �� 1 �������
#define T_PID   25          //ʳ������ ����������� �������0 �� ��������� ϲ� ��������� 200ms

//flash byte PWMSetReg[3][3] = {{0x15,0x00,0xF9},{0x14,0x00,0xF8},{0x13,0x00,0xF3}};

//TC initialisations
void TCinit(void);

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void);

// Timer1 overflow interrupt service routine
//interrupt [TIM1_OVF] void timer1_ovf_isr(void);

// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void);

//Timer1 interrupt Comp B
interrupt [TIM1_COMPB] void TC1compA_ovf(void);