// ����������� ����������� GSM-�������� ProtoPhone Mod01 Mammoth
// ������ 0.1 �� 23 ��� 2011

#include "iom8.h"								// ����������� ���������� ���������

#include "inavr.h"							// Intrinsic-�������

#include "ctype.h"							// �������� � ���������

#include "string.h"							// �������� �� ��������

#define bit(n)(1 << (n)) // ����������� �������� � ������
#define setbit(p, n)(p |= bit(n)) // ���������� ���
#define clrbit(p, n)(p &= ~bit(n)) // �������� ���

#define MASTERCLOCK 7372800 // �������� ������� ���������� � ��
#define delay_us(c) __delay_cycles(MASTERCLOCK / 1000000 * c) // �������������� ��������. c max = 268435455
#define delay_ms(c) __delay_cycles(MASTERCLOCK / 1000 * c) // �������������� ��������. c max = 268435

// ���� B
#define PWRKEY 1 // ��������� GSM ������
#define ENA 2 // ��������� ������� GSM-������
#define MOSI 3 // ����� MOSI SPI-����������
#define MISO 4 // ����� MISO SPI-����������
#define SCK 5 // ����� SCK SPI-����������
// ���� C
#define COL2 0 // ������� 2 ������ ����������
#define COL1 1 // ������� 1 ������ ����������
#define BUZZ 2 // �������� �������
#define H_P 3 // ��������� "�������"
#define H_C 4 // ��������� "������"
#define H_SL 5 // ��������� "������� �������"
#define RST 6 // �����
// ���� D
#define TXD 0 // ����� UART GSM-������, ���� UART ����������������
#define RXD 1 // ���� UART GSM-������, ����� UART ����������������
#define ROW1 2 // ������ 1 ������ ����������
#define ROW2 3 // ������ 2 ������ ����������
#define COL3 4 // ������� 3 ������ ����������
#define ROW5 5 // ������ 5 ������ ����������
#define ROW4 6 // ������ 4 ������ ����������
#define ROW3 7 // ������ 3 ������ ����������
#define ROWMASK 0xEC // = 11101100b

void PortInit(void) // ��������� ������ �����-������
{
    DDRB = (1 << PWRKEY) | (1 << ENA); //	���� B
    DDRC = (1 << COL2) | (1 << COL1) | (1 << BUZZ) | (1 << H_P) | (1 << H_C) | (1 << H_SL); //	���� C
    DDRD = (1 << RXD) | (1 << COL3); //	���� D
    PORTD = (1 << ROW1) | (1 << ROW2) | (1 << ROW3) | (1 << ROW4) | (1 << ROW5); // �������� �������� � ������ ������������ ����������
} // PortInit

#define FRAMING_ERROR(1 << FE)
#define PARITY_ERROR(1 << UPE)
#define DATA_OVERRUN(1 << DOR)
#define DATA_REGISTER_EMPTY(1 << UDRE)
#define BAUD 115200 // �������� UART
#define MYUBRR(MASTERCLOCK / 16 / BAUD - 1)

#define RXBUFLENGTH 100
char RxBuf[RXBUFLENGTH]; // �����, � ������� ��� ����������� ������� ������������ ������ �� GSM-������
volatile unsigned char RxBufWrPoint = 0;

#define NUMBUFLENGTH 30
char NumBuf[NUMBUFLENGTH]; // ����� ��� ����������� ������
volatile unsigned char NumBufWrPoint = 0;

unsigned char GSMStatus = 0; // ��������� GSM-������: 0 - �������� ; 1 - �������; 2 - ��������� ������; 3 - �������� ������; 4 - ���� �������� �� ��������� ������
char GSMSigStrength = 0; // ������� �������

void UARTinit(void) // ������������� UART
{
    // ������������� UART: 8 Data, 1 Stop; RX interrupt ON
    UCSRA = 0x00;
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE); // �������� �������, ���������� �������, ���������� �� ��������� ��������
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // ����� ������ 8 ���
    UBRRH = (unsigned char)(MYUBRR >> 8); // ������ ��������
    UBRRL = (unsigned char)(MYUBRR);
} // UARTinit

int putchar(int data) // ����� ����� � UART
{
    while (!(UCSRA & DATA_REGISTER_EMPTY));
    UDR = data;
    while (!(UCSRA & DATA_REGISTER_EMPTY));

    return data;
} //putchar

void OutText(char * text) // ����� ������ � UART
{
    while ( * text) putchar( * text++);
} //OutText

void OutDat(unsigned long int val, unsigned char len, unsigned char Const) // ����� ����� � UART
{
    unsigned char Str[16]; // ������������ ����� ����� (� ���������� ����)
    unsigned char k = 0;

    for (k = 0; k < len; k++) // ���������� ����� � ������
    {
        *(Str + (len - k - 1)) = (val % Const) + '0';

        if ( * (Str + (len - k - 1)) > '9')
            *
            (Str + (len - k - 1)) += 'A' - '0' - 10;
        val /= Const;
    }

    Str[len] = 0; // �������� �����
    k = 0;
    while (Str[k] != 0)
        putchar(Str[k++]);
} //OutDat

#pragma vector = USART_RXC_vect // ���������� �� ������� ����� � UART
__interrupt void USART_RXC_Interrupt(void) {
    char InData = UDR; // ������ �� UDR ���� ������� �����������, ���� ���� ��� ��� �� ����������
    if (InData > 20) // ����������� ��������� �������
    {
        RxBuf[RxBufWrPoint] = InData; // ���������� ��������� ���� � ������ RxBuf
        RxBufWrPoint++;
    }
} //USART_RXC_Interrupt

void ClearRxBuf(void) // ������� ������� RxBuf
{
    for (unsigned char ClearPoint = 0; ClearPoint < RXBUFLENGTH; RxBuf[ClearPoint++] = 0);
    RxBufWrPoint = 0;
} //ClearRxBuf

void ClearNumBuf(void) // ������� ������� RxBuf
{
    for (unsigned char ClearPoint = 0; ClearPoint < NUMBUFLENGTH; NumBuf[ClearPoint++] = 0);
    NumBufWrPoint = 0;
} //ClearNumBuf

char KeyConvert(char RawKey) // ������������ ��������, ��������� � ���������� � �������� ���
{
    char ConvertedKey = 0;

    switch (RawKey) {
    case (0xE8):
        ConvertedKey = 'Y';
        break;
    case (0xE4):
        ConvertedKey = '1';
        break;
    case (0x6C):
        ConvertedKey = '4';
        break;
    case (0xAC):
        ConvertedKey = '7';
        break;
    case (0xCC):
        ConvertedKey = '*';
        break;
    case (0xF7):
        ConvertedKey = 'S';
        break;
    case (0xF3):
        ConvertedKey = '2';
        break;
    case (0x7B):
        ConvertedKey = '5';
        break;
    case (0xBB):
        ConvertedKey = '8';
        break;
    case (0xDB):
        ConvertedKey = '0';
        break;
    case (0x06):
        ConvertedKey = 'N';
        break;
    case (0x02):
        ConvertedKey = '3';
        break;
    case (0x8A):
        ConvertedKey = '6';
        break;
    case (0xCA):
        ConvertedKey = '9';
        break;
    case (0xEA):
        ConvertedKey = '#';
        break;
    default:
        ConvertedKey = 0;
        break;
    }
    return ConvertedKey;
} //KeyConvert

char KeyScan(void) // ������������ ����������. ������ 0, ���� �� ���� ������� �� ������ ��� ��� ������� ������� (�� 1 �� 15)
{
    unsigned char PreKeyCode = 0;
    unsigned char KeyCode = 0;
    #define FIRST_DEBOUNCE_TIME 100
    #define DEBOUNCE_TIME 20

    clrbit(PORTC, COL1); // ������������ ������� 1
    setbit(PORTC, COL2);
    setbit(PORTD, COL3);
    delay_ms(DEBOUNCE_TIME);
    PreKeyCode = ROWMASK & PIND;
    delay_ms(DEBOUNCE_TIME);
    KeyCode = ROWMASK & PIND;
    if ((KeyCode == PreKeyCode) && (KeyCode != ROWMASK))
        return KeyConvert(KeyCode);

    setbit(PORTC, COL1); // ������������ ������� 2
    clrbit(PORTC, COL2);
    setbit(PORTD, COL3);
    delay_ms(DEBOUNCE_TIME);
    PreKeyCode = ROWMASK & PIND;
    delay_ms(DEBOUNCE_TIME);
    KeyCode = ROWMASK & PIND;
    if ((KeyCode == PreKeyCode) && (KeyCode != ROWMASK))
        return KeyConvert(KeyCode + 0x0F);

    setbit(PORTC, COL1); // ������������ ������� 3
    setbit(PORTC, COL2);
    clrbit(PORTD, COL3);
    delay_ms(DEBOUNCE_TIME);
    PreKeyCode = ROWMASK & PIND;
    delay_ms(DEBOUNCE_TIME);
    KeyCode = ROWMASK & PIND;
    if ((KeyCode == PreKeyCode) && (KeyCode != ROWMASK))
        return KeyConvert(KeyCode + 0x1E);

    setbit(PORTC, COL1); // ����� ������������
    setbit(PORTC, COL2);
    setbit(PORTD, COL3);
    return 0;
} //KeyScan

void GSM_On(void) // ��������� GSM-������
{
    setbit(PORTB, PWRKEY);
    delay_ms(3000);
    clrbit(PORTB, PWRKEY);
    delay_ms(8000);
    setbit(PORTB, PWRKEY);
    delay_ms(8000);
} //GSM_On

void Beep(void) // �������� �������� ������
{
    setbit(PORTC, H_C);
    setbit(PORTC, BUZZ);
    delay_ms(10);
    if ((GSMStatus != 2) && (GSMStatus != 3)) // ���� ���� �������� ��� ��������� ������, ��������� "Call" ������ �� ����
        clrbit(PORTC, H_C);
    clrbit(PORTC, BUZZ);
    delay_ms(20);
} //Beep

void LongBeep(void) // �������� ������ ���������
{
    setbit(PORTC, H_C);
    setbit(PORTC, BUZZ);
    delay_ms(15);
    if ((GSMStatus != 2) && (GSMStatus != 3)) // ���� ���� �������� ��� ��������� ������, ��������� "Call" ������ �� ����
        clrbit(PORTC, H_C);
    clrbit(PORTC, BUZZ);
    delay_ms(100);
} //LongBeep

void IncomingCallBeep(void) // ������� �������� ������
{
    setbit(PORTC, H_C);
    setbit(PORTC, BUZZ);
    delay_ms(100);
    if ((GSMStatus != 2) && (GSMStatus != 3)) // ���� ���� �������� ��� ��������� ������, ��������� "Call" ������ �� ����
        clrbit(PORTC, H_C);
    clrbit(PORTC, BUZZ);
    delay_ms(100);
} //IncomingCallBeep

unsigned int PowerFlashOrder = 0xAAAA; // = 1010101010101010b
unsigned int SignalLevelFlashOrder = 0x0000;

void TimerInit(void) // ������������� �������
{
    TCCR0 = (1 << CS00) | (1 << CS01); // ������������ �� ���������� �����, � ������������� 1024 = ~30 �� @ 7 ���
    TIMSK |= 1 << TOIE0; // ��������� ���������� �� ���������� OCR0A
    TCNT0 = 0x00;
} //TimerInit()

unsigned int TimerCounter = 0;
unsigned char SignalLevelObsolete = 0;

#pragma vector = TIMER0_OVF_vect // ���������� �� ������������ Timer0
__interrupt void TIMER0_OVF_Interrupt(void) {
    TimerCounter++;

    if ((TimerCounter % 0x80) == 0) {
        PowerFlashOrder = (PowerFlashOrder << 1) | (PowerFlashOrder >> 15);
        if (PowerFlashOrder & 0x0001) // ������� �������� ���������� "Power"
            setbit(PORTC, H_P);
        else
            clrbit(PORTC, H_P);

        SignalLevelFlashOrder = (SignalLevelFlashOrder << 1) | (SignalLevelFlashOrder >> 15);
        if (SignalLevelFlashOrder & 0x0001) // ������� �������� ���������� "Signsl level"
            setbit(PORTC, H_SL);
        else
            clrbit(PORTC, H_SL);

        if ((TimerCounter % 0x5000) == 0) // ���� ����� ���������� ������� �������
        {
            SignalLevelObsolete = 1;
        }
    }
} //TIMER0_OVF_Interrupt

int StrToInt(char * InputStr) // ��������� ������������ ������ � ����� ���� int. �������������� ������ �����
{
    int ReturnValue = 0;
    unsigned char StrCount = 0;

    for (; StrCount <= strlen(InputStr); StrCount++)
        if (isdigit(InputStr[StrCount]))
            ReturnValue = 10 * ReturnValue + (InputStr[StrCount] - 48);

    return ReturnValue;
} //StrToInt

void SignalLevelDefinition(void) {
    ClearRxBuf(); // ����� AT+CSQ
    OutText("AT+CSQ\n\r"); // ������ ������ ������� ������� ����
    delay_ms(300);
    OutText(RxBuf);
    OutText("\n\r");
    strtok(RxBuf, ","); // �������� bit error rate <ber>, ������� ���� ����� �������
    GSMSigStrength = StrToInt(RxBuf);
    if (GSMSigStrength >= 25) // ���������� ��������
        GSMSigStrength = 5;
    else {
        GSMSigStrength /= 5;
        if (GSMSigStrength == 0) GSMSigStrength = 1; // ����� �������� ���� �� ��� � �������� ������������, ��� GSM-������ ����� ����
    }
    if (GSMSigStrength == 99) //	������������ ��������, ������ �����, 99 - "Not known or not detectable"
        GSMSigStrength = 0;

    switch (GSMSigStrength) { // ������� �������� ���������� "Signsl level"
    case 5:
        SignalLevelFlashOrder = 0xAA80;
        break; // = 1010101010000000b
    case 4:
        SignalLevelFlashOrder = 0xAA00;
        break; // = 1010101000000000b
    case 3:
        SignalLevelFlashOrder = 0xA800;
        break; // = 1010100000000000b
    case 2:
        SignalLevelFlashOrder = 0xA000;
        break; // = 1010000000000000b
    case 1:
        SignalLevelFlashOrder = 0x8000;
        break; // = 1000000000000000b		
    case 0:
        SignalLevelFlashOrder = 0xAAAA;
        break; // = 1010101010101010b
    default:
        break;
    }
} //SignalLevelDefinition

void main(void) {
    PortInit(); // ��������� ������ �����-������
    UARTinit(); // ������������� UART
    TimerInit(); // ������������� �������
    ClearRxBuf();

    asm("sei"); // ��������� ����������

    setbit(PORTC, H_P); // ����������, ��� ������� ������
    delay_ms(300);
    Beep();
    delay_ms(300);
    Beep();
    delay_ms(1000);

    setbit(PORTB, ENA); // �������� ������� GSM ������
    delay_ms(1000);

    while (GSMStatus == 0) {
        GSM_On(); // �������� GSM ������
        ClearRxBuf();
        OutText("AT+IPR=115200\n\r"); // ������������� ������������� �������� �����
        delay_ms(300);
        OutText("ATE0\n\r"); // ��������� ��� � UART
        delay_ms(300);
        OutText("AT+CHFA=0\n\r"); // �������� ����������
        delay_ms(300);
        OutText("AT+CLVL=90\n\r"); // ������� �������� ��������
        delay_ms(300);
        OutText("AT+CMIC=0,7\n\r"); // ���������������� ���������
        delay_ms(300);

        if ((strstr(RxBuf, "OK") != NULL) && (strstr(RxBuf, "ERROR") == NULL)) // ���� GSM-������ �� ����� ������ � ����� OK...
            GSMStatus = 1;
        ClearRxBuf();
    }

    LongBeep();
    LongBeep();
    LongBeep();
    SignalLevelDefinition();
    PowerFlashOrder = 0xFFFF; // = 1111111111111111b, �. �. ��������� "Power" ����� ���������

    delay_ms(FIRST_DEBOUNCE_TIME); // ��� ������������� ���������� ��������� ����� ��������� �������, ������� �������� �� ����� ������������ ����������

    char Key = 0;
    char OldKey = 0;

    while (1) {
        if (SignalLevelObsolete == 1) {
            SignalLevelDefinition();
            SignalLevelObsolete = 0;
        }

        if (strstr(RxBuf, "RING") != NULL) {
            IncomingCallBeep();
            IncomingCallBeep();
            IncomingCallBeep();
            IncomingCallBeep();
            GSMStatus = 3;
            setbit(PORTC, H_C);
        }

        if (strstr(RxBuf, "NO CARRIER") != NULL) {
            GSMStatus = 1;
            clrbit(PORTC, H_C);
        }

        if (strstr(RxBuf, "BUSY") != NULL) {
            GSMStatus = 1;
            clrbit(PORTC, H_C);
        }

        ClearRxBuf();

        OldKey = Key;
        Key = KeyScan();
        if ((Key != 0) && (Key != OldKey)) // ���� ���-�� ������������� � �� ������, ������� ��������� � ������ ���
        {
            if ((Key != 'Y') && (Key != 'S') && (Key != 'N') && (GSMStatus != 2) && (GSMStatus != 3) && (GSMStatus != 4)) // ������ �������� ������� ��� * ��� # ���� �� ����� ��������� �������������� ������ ��������� ������� "YES" � "NO"
            {
                NumBuf[NumBufWrPoint] = Key;
                NumBufWrPoint++;
                Beep();
            }

            if (Key == 'N') //	������ ������� "NO"
            {
                ClearNumBuf();
                OutText("ATH\n\r"); // ...�� ���������� ��������
                clrbit(PORTC, H_C); // ����� ��������� "Call"
                GSMStatus = 1;
                delay_ms(200);
                LongBeep();
                LongBeep();
            }

            if ((Key == 'Y') && (GSMStatus == 1)) //	������ ������� "YES"
            {
                OutText("ATD");
                OutText(NumBuf);
                OutText(";\n\r"); // ������� �� ����� � ������� �������
                ClearNumBuf();
                delay_ms(200);
                LongBeep();
                LongBeep();
                GSMStatus = 2;
                setbit(PORTC, H_C); // �������� ��������� "Call"
            }
            if ((Key == 'Y') && (GSMStatus == 3)) //	������ ������� "YES"
            {
                OutText("ATA\n\r");
                ClearNumBuf();
                GSMStatus = 4;
                delay_ms(200);
                LongBeep();
                LongBeep();
                setbit(PORTC, H_C); // �������� ��������� "Call"
            }

            if ((Key == 'S') && (GSMStatus == 1)) //	������ ������� "SOS"
            {
                OutText("ATD89177985198;\n\r"); // ������� �� ����� �� ������� ����������� ��������� protoboardfab.com
                ClearNumBuf();
                delay_ms(200);
                LongBeep();
                LongBeep();
                GSMStatus = 2;
                setbit(PORTC, H_C); // �������� ��������� "Call"
            }
        }
    } //while (1)
} // main()