//

void ADCinit (void);

volatile unsigned int ADCDispVal = 0;
volatile unsigned int ADCValPID = 0;                        //������� ����������� ��� ϲ� (� 10 ���� �����)
volatile int ADCVal = 0;                                    //������� ����������� 4x

void getADC(void);                                          //������� ���

interrupt [ADC_INT] void ADCCmpl(void);