//

unsigned int NumVer = 170;

#define BUZ_ON  PORTB.0=0       //Свисток ON
#define BUZ_OFF PORTB.0=1       //Свисток OFF
    
#define lRed PORTC.3            //Красний діод
#define lGrn PORTC.2            //Зелений діод
    
#define MIN_TEMP 80             //Мінімальна температура
#define MAX_TEMP 420            //Максимальна температура

//Семисегментний індикатор-------------------
eeprom byte eDispCmn = 255;     //0-CC, 1-CA, 255-need set up
byte DispCmn;
#define CAD 1                   //CA
#define CCD 0                   //CC

#define Dg1 PORTB.6             //Розряд 1
#define Dg2 PORTD.4             //Розряд 2
#define Dg3 PORTD.1             //Розряд 3

#define SgA PORTB.7             //Сегмент А
#define SgB PORTD.6             //Сегмент B
#define SgC PORTB.2             //Сегмент C
#define SgD PORTB.5             //Сегмент D
#define SgE PORTB.4             //Сегмент E
#define SgF PORTD.5             //Сегмент F
#define SgG PORTD.7             //Сегмент G
#define SgP PORTB.3             //Сегмент DOT

flash byte SymbolsCCD[] = { //Масив символів (Common Catode)
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
0b0000000,                //10 - пустота
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

byte Disp[3];                                               //порозрядні значення для відображення
volatile byte DotDisp = 0;
volatile unsigned int DispVal = 0;
void ReCode(unsigned int, byte);                            //функція перекодування цифрового значення в посегментне
                                                            //ReCode(значення для відображення, кількість сегментів)
volatile byte enBlk = 0;                                    //мигання: 0- вимкнуто, 1- мигання в молодшому розряді, 2- в 2х, 3- в 3х
volatile bit isBlk = 0;                                     //мигалка
volatile byte BlinkDelay = 0;                               //затримка мигання
volatile bit chDispMode = 0;
void AutoSetDisp(void);
//-------------------------------------------

//Кнопки-------------------------------------
#define BT_SEL       PIND.0 == 0                             //Кнопка ОК
#define BT_MNS      PIND.3 == 0                             //Кнопка -
#define BT_PLS      PIND.2 == 0                             //Кнопка +

//KeyPress
#define KP_RESET        0           //скидання функції нажатої кнопки
#define KP_MINUS        1           //натисканя -
#define KP_PLUS         2           //натисканя +
#define KP_SEL_SHT      3           //коротке натисканя select/ok
#define KP_PLMN_SHT     4           //коротке натисканя - та +
#define KP_SEL_LNG      5           //довге натисканя (утримування) select/ok
#define KEY_TIMER       127         //вихід з режиму очікування/вимкнення

//KeyPressDelay
#define KD_SHT      CPS0_S/25       //Затримка повторного нажаття кнопок мала, 40msec
#define KD_NRM      CPS0_S/5        //Затримка повторного нажаття кнопок велика, 200msec
#define LPDC        KD_NRM/4        //Затримка перед "довгим утримуванням" кнопки, 1sec

//KeyMode
#define KM_PRM          0           //первинний режим роботи кнопок + і -
#define KM_SEC          1           //вторинний режим роботи кнопок + і -

volatile byte KeyPress = 0;                                 //Функція нажатої кнопки
volatile byte KeyMode = 0;
byte isChng = 0;                                            //1- відбулась зміна значення, 2- відбувся перехід в інший режим без зміни значення
volatile bit LngKeyPress = 0;                               //довге утримування кнопки
//StMode
byte StMode = 0;                    //режим, 0- OFF
byte SubMnuMode = 0;                //режими підменю
unsigned int m_delay = 0;           //затримка до скидання режиму
#define SM_OFF          0           //режим OFF
#define SM_WAIT         1           //режим очікування
#define SM_iDISP        2           //режим ініціалізації дисплея
#define SM_iD_SP        3           //режим ініціалізації дисплея і запуск налаштувань

#define SM_SUP          4           //режим SetUP
#define SM_CLB          5           //калібровка
#define SM_CLB_SET      6           //режим калібровки
#define SM_HSM          7           //режим настройки алгоритму нагріву
#define SM_HSM_SET      8           //установка коефіцієнтів алгоритму нагріву
#define SM_ADL          9           //детектор активності паяльника
#define SM_ADL_SET      10          //установка рівня чуттєвості детектора активності паяльника
#define SM_SBT          11          //режим установки температури режима standby
#define SM_SBT_SET      12          //установка температури режима standby

#define SM_STB          15          //режим standby
#define SM_TMR_SET      16          //установка значень таймера
#define SM_WAIT_STB     17          //затримка перед переходом в Stb при ручному виборі
#define SM_WAIT_OFF     18          //затримка перед переходом в OFF при ручному виборі
//#define SM_MAIN_SEC     19          //основний робочий режим, вторинне керування
//#define SM_MAIN_PRM     20          //основний робочий режим, первинне керування
#define SM_MAIN         21

//режими змін-------------------------------------------------------------------------------------
#define CHNG_RST        0           //скидання режиму змін
#define CHNG_PCM        1           //відбувся вибір пресету або загальні зміни
#define CHNG_TPS        2           //відбулась зміна значення температури
#define CHNG_TSI        3           //відбувся перехід в настройки часу таймера
#define CHNG_TMS        4           //відбулась зміна значення часу таймера 
//-------------------------------------------

//Установка та пресети температури-----------
#define MIN_PRN         3
#define MAX_PRN         8
#define COUNT_PRESET 5                  //кількість пресетів температури
eeprom unsigned int eTP[8] = {200,265,280,0,0,0,0,0};       //Пресети температури
eeprom byte eiTP = 2;                                       //Індекс пресета в EEPROM
byte riTP;                                                  //Поточний індекс пресета
//eeprom byte eNP = 3;                                        //Кількість пресетів EEPROM
eeprom unsigned int eLastT = 0;                             //Остання встановлена температура
eeprom unsigned int eStbTemp = 120;                         //Температура режиму очікування
int Ks;                                                     //Коефіцієнт зміщення температури
unsigned int Kg;                                            //Коефіцієнт підсилення температури
eeprom int eKs = 5;                                         //Коефіцієнт зміщення температури в EEPROM
eeprom unsigned int eKg = 48;                               //Коефіцієнт підсилення температури в EEPROM 
volatile int SetTemp = 0;                                   //Задана температура
//-------------------------------------------

//Тип керування та кількість пресетів--------
#define PCM_TEMP        0
#define PCM_PRST        1
#define CTRL_PCM        0
#define CTRL_NOP        1
//eeprom byte ePCM = 0;
//-------------------------------------------

//Фіксація помилки паяльника або перегріву---
#define ERR_MAX_TEMP 460
#define ERR_DELAY_MT 5
#define ERR_DELAY_DT 50
volatile bit enErrCheck = 0;
void ErrDetect(void);
void ErrPrc(void); 
//-------------------------------------------

//Таймер-------------------------------------
#define MAX_TIMER 90
eeprom byte e_tval = 5;                                     //Значення таймера в хвилинах в EEPROM
byte r_tval;                                                //значення таймера в хвилинах
volatile byte tval = 0;                                     //поточне значення таймера
bit TimerEn = 0, PwrTmRstEn = 0;                            //1- таймер дозволений, скидання таймеру при активності паяльника дозволено
volatile byte tRst = 0;
volatile byte TimerStop = 0;                                //зупинка по таймеру, 1- стендбай, 2- вимк
void TimerReset(void);
void TimerInit(void);
#define MIN_ADLP  59
eeprom byte eADLdp = 32;
byte ADLdp;//, ADLdp_thr;
void PwrTmRst(void);                                        //функція скидання значень таймера при паянні
//-------------------------------------------

//Установка режимів Stb та OFF---------------
void SetStb(void);
void SetOff(void);
//-------------------------------------------

//PID-алгоритм-------------------------------
long LimMAX = 0;
#define T_SMPL  64                                          //період запуску розразунку ПІД, мс (65536 мкс ~= 64 мс)
byte Kp;
byte Ki;
byte Kd;
void PID_init(void);                                        //ініціалізація PID-алгоритму
int PID_calc(int, int);                                     //PID-алгоритм
volatile int ValPWM = 0;                                    //Значення PWM
volatile bit enPID = 0;

#define HSM_KPS         0                                   //установка Kp
#define HSM_KIS         1                                   //установка Ki
#define HSM_KDS         2                                   //установка Kd
#define HSM_PFS         3                                   //установка частоти ШІМ нагрівача
#define HSM_MPS         4                                   //установка обмеження заповнення ШІМ нагрівача
#define MaxPWM          95

eeprom byte eHSM[] = {65,62,0,1,75};                        //Kp, Ki, PWM Frequency (6.25, 100), Max Duty Cycle %
//-------------------------------------------
    
/*
//SoftSTART----------------------------------
#define Tss 300                                             //SoftStart time delay, msec
#define Vss 125                                             //destination value PWM for SoftStart --- 50%
volatile bit isSS = 0;                                      //1- run SoftStart 
volatile bit isSScmpl = 0;                                  //1- SoftStart complete
volatile bit enSS = 0;                                      //
int SoftStart(int);                                         //ініціалізація плавного пуску
//-------------------------------------------
*/

//Параметри фіксування стабілізації температури
#define HT_LCK 1                                            //гістерезис температури захвату стабілізації
#define HT_WRK 3                                            //гістерезис температури утримування стабілізації
#define HT_CLB 2                                            //гістерезис температури стабілізації калібровки
byte StHyst = HT_LCK;                                       //гістерезис стабільної температури
#define T_STAB        4                                     //початкове значення часу стабілізації, sec
volatile unsigned int Tstab;                                //якщо за даний час температура не змінюється, значить вона стабілізувалась
volatile bit isStab = 0;                                    //1- температура стабілізувалася
volatile unsigned int stab_delay = 0;                       //затримка isStab
volatile bit enStabBeep = 0;                                //можна бібікнуть
//-------------------------------------------

//Калібровка---------------------------------
#define CLB_TOP 350
#define CLM_BGN          0                                  //початкові налаштування для калібровки
#define CLM_LPT          1                                  //установка температури в нижній точці
#define CLM_LPT_OK       2                                  //калібровка в нижній точці
#define CLM_HT           3                                  //нагрів
#define CLM_HPT          4                                  //установка температури в верхній точці
#define CLM_HPT_OK       5                                  //калібровка в верхній точці
#define CLM_OK           6                                  //калібровка виконана
byte ClbMode = 0;
unsigned int clb_ct_val, clb_t_val;
void ClbSI(void);                                           //функція калібровки паяльника
void ClbInit(void);                                         //функція попередніх перевірок та установок перед запуском калібровки
//-------------------------------------------

void beep(byte);
void g_led(void);
void r_led(void);
void led_off(void);
void StartUp(void);
void InitTemp(void);