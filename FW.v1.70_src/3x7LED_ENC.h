//

unsigned int NumVer = 170;

#define BUZ_ON  PORTB.0=0       //������� ON
#define BUZ_OFF PORTB.0=1       //������� OFF
    
#define lRed PORTC.3            //������� ���
#define lGrn PORTC.2            //������� ���
    
#define MIN_TEMP 80             //̳�������� �����������
#define MAX_TEMP 420            //����������� �����������

//�������������� ���������-------------------
eeprom byte eDispCmn = 255;     //0-CC, 1-CA, 255-need set up
byte DispCmn;
#define CAD 1                   //CA
#define CCD 0                   //CC

#define Dg1 PORTB.6             //������ 1
#define Dg2 PORTD.4             //������ 2
#define Dg3 PORTD.1             //������ 3

#define SgA PORTB.7             //������� �
#define SgB PORTD.6             //������� B
#define SgC PORTB.2             //������� C
#define SgD PORTB.5             //������� D
#define SgE PORTB.4             //������� E
#define SgF PORTD.5             //������� F
#define SgG PORTD.7             //������� G
#define SgP PORTB.3             //������� DOT

flash byte SymbolsCCD[] = { //����� ������� (Common Catode)
0b1111110,                //0
0b0110000,                //1
0b1101101,                //2
0b1111001,                //3
0b0110011,                //4
0b1011011,                //5
0b1011111,                //6
0b1110000,                //7
0b1111111,                //8
0b1111011,                //9
0b0000000,                //10 - �������
0b1001111,                //11 - E
0b0000101,                //12 - r
0b0001111,                //13 - t
0b0011111,                //14 - b
0b1000111,                //15 - F
0b0000001,                //16 - -
0b1001110,                //17 - C
0b1100111,                //18 - P
0b0111101,                //19 - d
0b0111110,                //20 - U
0b1110111,                //21 - A
0b0001101,                //22 - c
0b0010101,                //23 - n
0b0001110,                //24 - L
0b0110111                 //25 - H 
};

byte Disp[3];                                               //���������� �������� ��� �����������
volatile byte DotDisp = 0;
volatile unsigned int DispVal = 0;
void ReCode(unsigned int, byte);                            //������� ������������� ��������� �������� � �����������
                                                            //ReCode(�������� ��� �����������, ������� ��������)
volatile byte enBlk = 0;                                    //�������: 0- ��������, 1- ������� � ��������� ������, 2- � 2�, 3- � 3�
volatile bit isBlk = 0;                                     //�������
volatile byte BlinkDelay = 0;                               //�������� �������
volatile bit chDispMode = 0;
void AutoSetDisp(void);
//-------------------------------------------

//�������------------------------------------
#define BT_OK       PIND.0 == 0                             //������ ��
#define EncP1       PIND.2
#define EncP2       PIND.3
#define ENC_R       11 
#define ENC_L       10
#define SetBit(port, bit) port|= (1<<bit)

volatile byte EncBuf = 0;                                   //����� ��������
void PollEncoder(void);
byte GetStateEncoder(void);
//-------------------------------------------

//������(�)----------------------------------
#define T_COUNT 500                                         //�������� ����� "������ ������������" ������, msec
#define OK_SHT  2
#define OK_LNG  3
volatile byte KeyPress = 0;                                 //������� ������ ������
byte isChng = 0;                                            //1- �������� ���� ��������, 2- ������� ������� � ����� ����� ��� ���� ��������
volatile bit LngKeyPress = 0;                               //����� ����������� ������
//StMode
byte StMode = 0;                    //�����, 0- OFF
byte SubMnuMode = 0;                //������ ������
unsigned int m_delay = 0;           //�������� �� �������� ������
#define SM_OFF          0           //����� OFF
#define SM_WAIT         1           //����� ����������
#define SM_iDISP        2           //����� ������������ �������
#define SM_iD_SP        3           //����� ������������ ������� � ������ �����������

#define SM_SUP          4           //����� SetUP
#define SM_CLB          5           //���������
#define SM_CLB_SET      6           //����� ���������
#define SM_HSM          7           //����� ��������� ��������� ������
#define SM_HSM_SET      8           //��������� ����������� ��������� ������
#define SM_ADL          9           //�������� ��������� ���������
#define SM_ADL_SET      10          //��������� ���� �������� ��������� ��������� ���������
#define SM_CTRL         11          //����� ��������� ���������
#define SM_CTRL_SET     12          //��������� ��������� ��������� (��� ���������� ��������� �� ������� �������)
#define SM_SBT          13          //����� ��������� ����������� ������ standby
#define SM_SBT_SET      14          //��������� ����������� ������ standby

#define SM_STB          15          //����� standby
#define SM_TMR_SET      16          //��������� ������� �������
#define SM_WAIT_STB     17          //�������� ����� ��������� � Stb ��� ������� �����
#define SM_WAIT_OFF     18          //�������� ����� ��������� � OFF ��� ������� �����
#define SM_MAIN_SEC     19          //�������� ������� �����, �������� ���������
#define SM_MAIN_PRM     20          //�������� ������� �����, �������� ��������� 
//-------------------------------------------

//��������� �� ������� �����������-----------
#define MIN_PRN         3
#define MAX_PRN         8
eeprom unsigned int eTP[8] = {200,265,280,0,0,0,0,0};       //������� �����������
eeprom byte eiTP = 2;                                       //������ ������� � EEPROM
byte riTP;                                                  //�������� ������ �������
eeprom byte eNP = 3;                                        //ʳ������ ������� EEPROM
eeprom unsigned int eLastT = 0;                             //������� ����������� �����������
eeprom unsigned int eStbTemp = 120;                         //����������� ������ ����������
int Ks;                                                     //���������� ������� �����������
unsigned int Kg;                                            //���������� ��������� �����������
eeprom int eKs = 5;                                         //���������� ������� ����������� � EEPROM
eeprom unsigned int eKg = 48;                               //���������� ��������� ����������� � EEPROM 
volatile int SetTemp = 0;                                   //������ �����������
//-------------------------------------------

//��� ��������� �� ������� �������--------
#define PCM_TEMP        0
#define PCM_PRST        1
#define CTRL_PCM        0
#define CTRL_NOP        1
eeprom byte ePCM = 0;
//-------------------------------------------

//Գ������ ������� ��������� ��� ��������---
#define ERR_MAX_TEMP 460
#define ERR_DELAY_MT 5
#define ERR_DELAY_DT 50
volatile bit enErrCheck = 0;
void ErrDetect(void);
void ErrPrc(void); 
//-------------------------------------------

//������-------------------------------------
#define MAX_TIMER 90
eeprom byte e_tval = 5;                                     //�������� ������� � �������� � EEPROM
byte r_tval;                                                //�������� ������� � ��������
volatile byte tval = 0;                                     //������� �������� �������
bit TimerEn = 0, PwrTmRstEn = 0;                            //1- ������ ����������, �������� ������� ��� ��������� ��������� ���������
volatile byte tRst = 0;
volatile byte TimerStop = 0;                                //������� �� �������, 1- ��������, 2- ����
void TimerReset(void);
void TimerInit(void);
#define KEY_TIMER 31
#define MIN_ADLP  59
eeprom byte eADLdp = 32;
byte ADLdp;//, ADLdp_thr;
void PwrTmRst(void);                                        //������� �������� ������� ������� ��� ������
//-------------------------------------------

//��������� ������ Stb �� OFF---------------
void SetStb(void);
void SetOff(void);
//-------------------------------------------

//PID-��������-------------------------------
long LimMAX = 0;
#define T_SMPL  64                                          //����� ������� ���������� ϲ�, �� (65536 ��� ~= 64 ��)
byte Kp;
byte Ki;
byte Kd;
void PID_init(void);                                        //������������ PID-���������
int PID_calc(int, int);                                     //PID-��������
volatile int ValPWM = 0;                                    //�������� PWM
volatile bit enPID = 0;

#define HSM_KPS         0                                   //��������� Kp
#define HSM_KIS         1                                   //��������� Ki
#define HSM_KDS         2                                   //��������� Kd
#define HSM_PFS         3                                   //��������� ������� ز� ��������
#define HSM_MPS         4                                   //��������� ��������� ���������� ز� ��������
#define MaxPWM          95

eeprom byte eHSM[] = {65,62,0,1,75};                        //Kp, Ki, Kd, PWM Frequency (6.25, 100), Max Duty Cycle %
//-------------------------------------------
    
//SoftSTART----------------------------------
#define Tss 300                                             //SoftStart time delay, msec
#define Vss 125                                             //destination value PWM for SoftStart --- 50%
//volatile bit isSS = 0;                                      //1- run SoftStart 
//volatile bit isSScmpl = 0;                                  //1- SoftStart complete
//volatile bit enSS = 0;                                      //
//int SoftStart(int);                                         //������������ �������� �����
//-------------------------------------------

//��������� ���������� ���������� �����������
#define HT_LCK 1                                            //��������� ����������� ������� ����������
#define HT_WRK 3                                            //��������� ����������� ����������� ����������
#define HT_CLB 2                                            //��������� ����������� ���������� ���������
byte StHyst = HT_LCK;                                       //��������� �������� �����������
#define T_STAB        4                                     //��������� �������� ���� ����������, sec
volatile unsigned int Tstab;                                //���� �� ����� ��� ����������� �� ���������, ������� ���� �������������
volatile bit isStab = 0;                                    //1- ����������� �������������
volatile unsigned int stab_delay = 0;                       //�������� isStab
volatile bit enStabBeep = 0;                                //����� �������
//-------------------------------------------

//���������---------------------------------
#define CLB_TOP 350
#define CLM_BGN          0                                  //�������� ������������ ��� ���������
#define CLM_LPT          1                                  //��������� ����������� � ������ �����
#define CLM_LPT_OK       2                                  //��������� � ������ �����
#define CLM_HT           3                                  //�����
#define CLM_HPT          4                                  //��������� ����������� � ������� �����
#define CLM_HPT_OK       5                                  //��������� � ������� �����
#define CLM_OK           6                                  //��������� ��������
byte ClbMode = 0;
unsigned int clb_ct_val, clb_t_val;
void ClbSI(void);                                           //������� ��������� ���������
void ClbInit(void);                                         //������� ���������� �������� �� ��������� ����� �������� ���������
//-------------------------------------------

void beep(byte);
void g_led(void);
void r_led(void);
void led_off(void);
void StartUp(void);
void InitTemp(void);