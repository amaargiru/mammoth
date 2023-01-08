// Программное обеспечение GSM-телефона ProtoPhone Mod01 Mammoth
// Версия 0.1 от 23 мая 2011

#include "iom8.h" // Определения внутренних регистров
#include "inavr.h" // Intrinsic-функции
#include "ctype.h" // Операции с символами
#include "string.h" // Операции со строками

#define bit(n)(1 << (n)) // Определение операций с битами
#define setbit(p, n)(p |= bit(n)) // Установить бит
#define clrbit(p, n)(p &= ~bit(n)) // Сбросить бит

#define MASTERCLOCK 7372800 // Тактовая частота процессора в Гц
#define delay_us(c) __delay_cycles(MASTERCLOCK / 1000000 * c) // Микросекундная задержка. c max = 268435455
#define delay_ms(c) __delay_cycles(MASTERCLOCK / 1000 * c) // Миллисекундная задержка. c max = 268435

// Порт B
#define PWRKEY 1 // Включение GSM модуля
#define ENA 2 // Включение питания GSM-модуля
#define MOSI 3 // Линия MOSI SPI-интерфейса
#define MISO 4 // Линия MISO SPI-интерфейса
#define SCK 5 // Линия SCK SPI-интерфейса
// Порт C
#define COL2 0 // Колонка 2 опроса клавиатуры
#define COL1 1 // Колонка 1 опроса клавиатуры
#define BUZZ 2 // Вызывной динамик
#define H_P 3 // Светодиод "Питание"
#define H_C 4 // Светодиод "Звонок"
#define H_SL 5 // Светодиод "Уровень сигнала"
#define RST 6 // Сброс
// Порт D
#define TXD 0 // Выход UART GSM-модуля, вход UART микроконтроллера
#define RXD 1 // Вход UART GSM-модуля, выход UART микроконтроллера
#define ROW1 2 // Строка 1 опроса клавиатуры
#define ROW2 3 // Строка 2 опроса клавиатуры
#define COL3 4 // Колонка 3 опроса клавиатуры
#define ROW5 5 // Строка 5 опроса клавиатуры
#define ROW4 6 // Строка 4 опроса клавиатуры
#define ROW3 7 // Строка 3 опроса клавиатуры
#define ROWMASK 0xEC // = 11101100b

void PortInit(void) // Активация портов ввода-вывода
{
    DDRB = (1 << PWRKEY) | (1 << ENA); //	Порт B
    DDRC = (1 << COL2) | (1 << COL1) | (1 << BUZZ) | (1 << H_P) | (1 << H_C) | (1 << H_SL); //	Порт C
    DDRD = (1 << RXD) | (1 << COL3); //	Порт D
    PORTD = (1 << ROW1) | (1 << ROW2) | (1 << ROW3) | (1 << ROW4) | (1 << ROW5); // Включаем подтяжки у входов сканирования клавиатуры
} // PortInit

#define FRAMING_ERROR(1 << FE)
#define PARITY_ERROR(1 << UPE)
#define DATA_OVERRUN(1 << DOR)
#define DATA_REGISTER_EMPTY(1 << UDRE)
#define BAUD 115200 // Скорость UART
#define MYUBRR(MASTERCLOCK / 16 / BAUD - 1)

#define RXBUFLENGTH 100
char RxBuf[RXBUFLENGTH]; // Буфер, в который для дальнейшего анализа записываются данные от GSM-модема
volatile unsigned char RxBufWrPoint = 0;

#define NUMBUFLENGTH 30
char NumBuf[NUMBUFLENGTH]; // Буфер для набираемого номера
volatile unsigned char NumBufWrPoint = 0;

unsigned char GSMStatus = 0; // Состояние GSM-модуля: 0 - выключен ; 1 - включен; 2 - исходящий звонок; 3 - входящий звонок; 4 - идет разговор по входящему звонку
char GSMSigStrength = 0; // Уровень сигнала

void UARTinit(void) // Инициализация UART
{
    // Инициализация UART: 8 Data, 1 Stop; RX interrupt ON
    UCSRA = 0x00;
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE); // Приемник включен, передатчик включен, прерывание от приемника включено
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Длина данных 8 бит
    UBRRH = (unsigned char)(MYUBRR >> 8); // Задаем скорость
    UBRRL = (unsigned char)(MYUBRR);
} // UARTinit

int putchar(int data) // Вывод байта в UART
{
    while (!(UCSRA & DATA_REGISTER_EMPTY));
    UDR = data;
    while (!(UCSRA & DATA_REGISTER_EMPTY));

    return data;
} //putchar

void OutText(char* text) // Вывод текста в UART
{
    while (*text) putchar(*text++);
} //OutText

void OutDat(unsigned long int val, unsigned char len, unsigned char Const) // Вывод числа в UART
{
    unsigned char Str[16]; // Максимальная длина числа (в печатаемом виде)
    unsigned char k = 0;

    for (k = 0; k < len; k++) // Превращаем число в строку
    {
        *(Str + (len - k - 1)) = (val % Const) + '0';

        if (*(Str + (len - k - 1)) > '9')
            *
            (Str + (len - k - 1)) += 'A' - '0' - 10;
        val /= Const;
    }

    Str[len] = 0; // Печатаем число
    k = 0;
    while (Str[k] != 0)
        putchar(Str[k++]);
} //OutDat

#pragma vector = USART_RXC_vect // Прерывание по приходу байта в UART
__interrupt void USART_RXC_Interrupt(void) {
    char InData = UDR; // Данные из UDR надо считать обязательно, даже если они нам не пригодятся
    if (InData > 20) // Отбрасываем служебные символы
    {
        RxBuf[RxBufWrPoint] = InData; // Записываем пришедший байт в массив RxBuf
        RxBufWrPoint++;
    }
} //USART_RXC_Interrupt

void ClearRxBuf(void) // Очистка массива RxBuf
{
    for (unsigned char ClearPoint = 0; ClearPoint < RXBUFLENGTH; RxBuf[ClearPoint++] = 0);
    RxBufWrPoint = 0;
} //ClearRxBuf

void ClearNumBuf(void) // Очистка массива RxBuf
{
    for (unsigned char ClearPoint = 0; ClearPoint < NUMBUFLENGTH; NumBuf[ClearPoint++] = 0);
    NumBufWrPoint = 0;
} //ClearNumBuf

char KeyConvert(char RawKey) // Конвертирует значение, считанное с клавиатуры в читаемый вид
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

char KeyScan(void) // Сканирование клавиатуры. Выдает 0, если ни одна клавиша не нажата или код нажатой клавиши (от 1 до 15)
{
    unsigned char PreKeyCode = 0;
    unsigned char KeyCode = 0;
#define FIRST_DEBOUNCE_TIME 100
#define DEBOUNCE_TIME 20

    clrbit(PORTC, COL1); // Сканирование колонки 1
    setbit(PORTC, COL2);
    setbit(PORTD, COL3);
    delay_ms(DEBOUNCE_TIME);
    PreKeyCode = ROWMASK & PIND;
    delay_ms(DEBOUNCE_TIME);
    KeyCode = ROWMASK & PIND;
    if ((KeyCode == PreKeyCode) && (KeyCode != ROWMASK))
        return KeyConvert(KeyCode);

    setbit(PORTC, COL1); // Сканирование колонки 2
    clrbit(PORTC, COL2);
    setbit(PORTD, COL3);
    delay_ms(DEBOUNCE_TIME);
    PreKeyCode = ROWMASK & PIND;
    delay_ms(DEBOUNCE_TIME);
    KeyCode = ROWMASK & PIND;
    if ((KeyCode == PreKeyCode) && (KeyCode != ROWMASK))
        return KeyConvert(KeyCode + 0x0F);

    setbit(PORTC, COL1); // Сканирование колонки 3
    setbit(PORTC, COL2);
    clrbit(PORTD, COL3);
    delay_ms(DEBOUNCE_TIME);
    PreKeyCode = ROWMASK & PIND;
    delay_ms(DEBOUNCE_TIME);
    KeyCode = ROWMASK & PIND;
    if ((KeyCode == PreKeyCode) && (KeyCode != ROWMASK))
        return KeyConvert(KeyCode + 0x1E);

    setbit(PORTC, COL1); // Конец сканирования
    setbit(PORTC, COL2);
    setbit(PORTD, COL3);
    return 0;
} //KeyScan

void GSM_On(void) // Включение GSM-модуля
{
    setbit(PORTB, PWRKEY);
    delay_ms(3000);
    clrbit(PORTB, PWRKEY);
    delay_ms(8000);
    setbit(PORTB, PWRKEY);
    delay_ms(8000);
} //GSM_On

void Beep(void) // Короткий звуковой сигнал
{
    setbit(PORTC, H_C);
    setbit(PORTC, BUZZ);
    delay_ms(10);
    if ((GSMStatus != 2) && (GSMStatus != 3)) // Если идет входящий или исходящий звонок, светодиод "Call" гасить не надо
        clrbit(PORTC, H_C);
    clrbit(PORTC, BUZZ);
    delay_ms(20);
} //Beep

void LongBeep(void) // Звуковой сигнал подлиннее
{
    setbit(PORTC, H_C);
    setbit(PORTC, BUZZ);
    delay_ms(15);
    if ((GSMStatus != 2) && (GSMStatus != 3)) // Если идет входящий или исходящий звонок, светодиод "Call" гасить не надо
        clrbit(PORTC, H_C);
    clrbit(PORTC, BUZZ);
    delay_ms(100);
} //LongBeep

void IncomingCallBeep(void) // Длинный звуковой сигнал
{
    setbit(PORTC, H_C);
    setbit(PORTC, BUZZ);
    delay_ms(100);
    if ((GSMStatus != 2) && (GSMStatus != 3)) // Если идет входящий или исходящий звонок, светодиод "Call" гасить не надо
        clrbit(PORTC, H_C);
    clrbit(PORTC, BUZZ);
    delay_ms(100);
} //IncomingCallBeep

unsigned int PowerFlashOrder = 0xAAAA; // = 1010101010101010b
unsigned int SignalLevelFlashOrder = 0x0000;

void TimerInit(void) // Инициализация таймера
{
    TCCR0 = (1 << CS00) | (1 << CS01); // Тактирование от системного клока, с предделителем 1024 = ~30 мс @ 7 МГц
    TIMSK |= 1 << TOIE0; // Разрешить прерывания по достижению OCR0A
    TCNT0 = 0x00;
} //TimerInit()

unsigned int TimerCounter = 0;
unsigned char SignalLevelObsolete = 0;

#pragma vector = TIMER0_OVF_vect // Прерывание по переполнению Timer0
__interrupt void TIMER0_OVF_Interrupt(void) {
    TimerCounter++;

    if ((TimerCounter % 0x80) == 0) {
        PowerFlashOrder = (PowerFlashOrder << 1) | (PowerFlashOrder >> 15);
        if (PowerFlashOrder & 0x0001) // Порядок свечения светодиода "Power"
            setbit(PORTC, H_P);
        else
            clrbit(PORTC, H_P);

        SignalLevelFlashOrder = (SignalLevelFlashOrder << 1) | (SignalLevelFlashOrder >> 15);
        if (SignalLevelFlashOrder & 0x0001) // Порядок свечения светодиода "Signsl level"
            setbit(PORTC, H_SL);
        else
            clrbit(PORTC, H_SL);

        if ((TimerCounter % 0x5000) == 0) // Пора вновь определить уровень сигнала
        {
            SignalLevelObsolete = 1;
        }
    }
} //TIMER0_OVF_Interrupt

int StrToInt(char* InputStr) // Процедура конвертирует строку в число типа int. Конвертируются только цифры
{
    int ReturnValue = 0;
    unsigned char StrCount = 0;

    for (; StrCount <= strlen(InputStr); StrCount++)
        if (isdigit(InputStr[StrCount]))
            ReturnValue = 10 * ReturnValue + (InputStr[StrCount] - 48);

    return ReturnValue;
} //StrToInt

void SignalLevelDefinition(void) {
    ClearRxBuf(); // Перед AT+CSQ
    OutText("AT+CSQ\n\r"); // Запрос уровня сигнала сотовой сети
    delay_ms(300);
    OutText(RxBuf);
    OutText("\n\r");
    strtok(RxBuf, ","); // Отсекаем bit error rate <ber>, который идет после запятой
    GSMSigStrength = StrToInt(RxBuf);
    if (GSMSigStrength >= 25) // Нормальное значение
        GSMSigStrength = 5;
    else {
        GSMSigStrength /= 5;
        if (GSMSigStrength == 0) GSMSigStrength = 1; // Чтобы моргнуть хотя бы раз и показать пользователю, что GSM-модуль видит сеть
    }
    if (GSMSigStrength == 99) // Ненормальное значение, скорее всего, 99 - "Not known or not detectable"
        GSMSigStrength = 0;

    switch (GSMSigStrength) { // Порядок свечения светодиода "Signsl level"
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
    PortInit(); // Активация портов ввода-вывода
    UARTinit(); // Инициализация UART
    TimerInit(); // Инициализация таймера
    ClearRxBuf();

    asm("sei"); // Разрешаем прерывания

    setbit(PORTC, H_P); // Показываем, что питание подано
    delay_ms(300);
    Beep();
    delay_ms(300);
    Beep();
    delay_ms(1000);

    setbit(PORTB, ENA); // Включаем питание GSM модуля
    delay_ms(1000);

    while (GSMStatus == 0) {
        GSM_On(); // Включаем GSM модуль
        ClearRxBuf();
        OutText("AT+IPR=115200\n\r"); // Устанавливаем фиксированную скорость связи
        delay_ms(300);
        OutText("ATE0\n\r"); // Отключаем эхо в UART
        delay_ms(300);
        OutText("AT+CHFA=0\n\r"); // Выбираем аудиоканал
        delay_ms(300);
        OutText("AT+CLVL=90\n\r"); // Уровень усиления динамика
        delay_ms(300);
        OutText("AT+CMIC=0,7\n\r"); // Чувствительность микрофона
        delay_ms(300);

        if ((strstr(RxBuf, "OK") != NULL) && (strstr(RxBuf, "ERROR") == NULL)) // Если GSM-модуль не выдал ошибки и выдал OK...
            GSMStatus = 1;
        ClearRxBuf();
    }

    LongBeep();
    LongBeep();
    LongBeep();
    SignalLevelDefinition();
    PowerFlashOrder = 0xFFFF; // = 1111111111111111b, т. е. светодиод "Power" горит постоянно

    delay_ms(FIRST_DEBOUNCE_TIME); // Для устаканивания переходных процессов после включения питания, могущих пролезть на входы сканирования клавиатуры

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
        if ((Key != 0) && (Key != OldKey)) // Если что-то отсканировано и не повтор, моргнем лампочкой и вышлем код
        {
            if ((Key != 'Y') && (Key != 'S') && (Key != 'N') && (GSMStatus != 2) && (GSMStatus != 3) && (GSMStatus != 4)) // Нажата цифровая клавиша или * или # плюс во время разговора разблокированы только служебные клавиши "YES" и "NO"
            {
                NumBuf[NumBufWrPoint] = Key;
                NumBufWrPoint++;
                Beep();
            }

            if (Key == 'N') //	Нажата клавиша "NO"
            {
                ClearNumBuf();
                OutText("ATH\n\r"); // ...то сбрасываем разговор
                clrbit(PORTC, H_C); // Гасим светодиод "Call"
                GSMStatus = 1;
                delay_ms(200);
                LongBeep();
                LongBeep();
            }

            if ((Key == 'Y') && (GSMStatus == 1)) // Нажата клавиша "YES"
            {
                OutText("ATD");
                OutText(NumBuf);
                OutText(";\n\r"); // Выходим на связь с набитым номером
                ClearNumBuf();
                delay_ms(200);
                LongBeep();
                LongBeep();
                GSMStatus = 2;
                setbit(PORTC, H_C); // Зажигаем светодиод "Call"
            }
            if ((Key == 'Y') && (GSMStatus == 3)) // Нажата клавиша "YES"
            {
                OutText("ATA\n\r");
                ClearNumBuf();
                GSMStatus = 4;
                delay_ms(200);
                LongBeep();
                LongBeep();
                setbit(PORTC, H_C); // Зажигаем светодиод "Call"
            }

            if ((Key == 'S') && (GSMStatus == 1)) // Нажата клавиша "SOS"
            {
                OutText("ATD89177985198;\n\r"); // Выходим на связь со службой технической поддержки protoboardfab.com
                ClearNumBuf();
                delay_ms(200);
                LongBeep();
                LongBeep();
                GSMStatus = 2;
                setbit(PORTC, H_C); // Зажигаем светодиод "Call"
            }
        }
    } //while (1)
} // main()