#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h> // Для abs()

// --- НАСТРОЙКИ LCD (Как в 4 лабе) ---
#define LCD_DATA_PORT PORTB
#define LCD_DATA_DDR  DDRB
#define LCD_CTRL_PORT PORTD
#define LCD_CTRL_DDR  DDRD
#define LCD_RS        PD4
#define LCD_RW        PD5
#define LCD_E         PD6

// --- НАСТРОЙКИ UART (Как в 5 лабе) ---
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

// --- БИТОВЫЕ КАРТЫ СИМВОЛОВ ---
// 1. Знак Мальчика (♂)
uint8_t male_bmp[8] = {
    0b00011, //   **
    0b00010, //    *
    0b00101, //  * *
    0b00100, //  *
    0b01110, //  ***
    0b10001, // *   *
    0b10001, // *   *
    0b01110  //  ***
};

// 2. Знак Девочки (♀)
uint8_t female_bmp[8] = {
    0b01110, //  ***
    0b10001, // *   *
    0b10001, // *   *
    0b01110, //  ***
    0b00100, //   *
    0b01110, //  ***
    0b00100, //   *
    0b00000  //
};

// --- ФУНКЦИИ LCD ---
void LCD_PulseEnable(void) {
    LCD_CTRL_PORT |= (1 << LCD_E); _delay_us(1);
    LCD_CTRL_PORT &= ~(1 << LCD_E); _delay_us(50);
}

void LCD_SendNibble(uint8_t nibble) {
    LCD_DATA_PORT &= 0x0F;
    LCD_DATA_PORT |= (nibble & 0xF0);
    LCD_PulseEnable();
}

void LCD_SendByte(uint8_t data, uint8_t is_data) {
    if (is_data) LCD_CTRL_PORT |= (1 << LCD_RS);
    else         LCD_CTRL_PORT &= ~(1 << LCD_RS);
    LCD_CTRL_PORT &= ~(1 << LCD_RW);
    LCD_SendNibble(data);
    LCD_SendNibble(data << 4);
}

void LCD_Init(void) {
    LCD_DATA_DDR |= 0xF0;
    LCD_CTRL_DDR |= (1<<LCD_RS)|(1<<LCD_RW)|(1<<LCD_E);
    _delay_ms(50);
    LCD_CTRL_PORT &= ~(1<<LCD_RS); LCD_CTRL_PORT &= ~(1<<LCD_RW); LCD_CTRL_PORT &= ~(1<<LCD_E);
    LCD_SendNibble(0x30); _delay_ms(5);
    LCD_SendNibble(0x30); _delay_us(150);
    LCD_SendNibble(0x30); _delay_us(150);
    LCD_SendNibble(0x20); _delay_us(150);
    LCD_SendByte(0x28, 0); _delay_ms(2);
    LCD_SendByte(0x0C, 0); _delay_ms(2);
    LCD_SendByte(0x06, 0);
    LCD_SendByte(0x01, 0); _delay_ms(20);
}

void LCD_Goto(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_SendByte(address, 0);
}

void LCD_String(char *str) {
    while (*str) LCD_SendByte(*str++, 1);
}

// Загрузка своего символа в память экрана
// code (0-7) - номер символа, bitmap - массив из 8 байт
void LCD_LoadCustomChar(uint8_t code, uint8_t *bitmap) {
    LCD_SendByte(0x40 + (code * 8), 0); // Устанавливаем адрес в CGRAM
    for (uint8_t i = 0; i < 8; i++) {
        LCD_SendByte(bitmap[i], 1); // Пишем данные (строки пикселей)
    }
    LCD_SendByte(0x80, 0); // Возвращаемся в обычный режим DDRAM
}

// --- ФУНКЦИИ UART ---
void USART_Init(unsigned int ubrr) {
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (0 << USBS) | (1 << UCSZ1) | (1 << UCSZ0);
}

// Неблокирующая проверка: пришел ли символ?
// Возвращает символ или 0, если пусто
unsigned char USART_Check(void) {
    if (UCSRA & (1 << RXC)) return UDR;
    return 0;
}

// --- ГЛАВНАЯ ПРОГРАММА ---
int main(void) {
    // Инициализация
    USART_Init(MYUBRR);
    LCD_Init();

    // Загружаем наши символы
    LCD_LoadCustomChar(0, male_bmp);   // Символ 0 - Мальчик
    LCD_LoadCustomChar(1, female_bmp); // Символ 1 - Девочка

    // Переменные состояния
    int8_t m_x = 0, f_x = 15;    // Координаты X (0-15)
    int8_t m_row = 0, f_row = 1; // Координаты Y (строки 0 и 1)
    
    int8_t m_dir = 1;  // Направление мальчика (+1 вправо, -1 влево)
    int8_t f_dir = -1; // Направление девочки

    uint16_t step = 0; // Счетчик шагов для скорости

    while (1) {
        // --- 1. ОТРИСОВКА ---
        LCD_SendByte(0x01, 0); // Очистить экран (стирает старые позиции)
        _delay_ms(2); // Пауза для очистки

        // Рисуем мальчика (код 0)
        LCD_Goto(m_row, m_x);
        LCD_SendByte(0, 1);

        // Рисуем девочку (код 1)
        LCD_Goto(f_row, f_x);
        LCD_SendByte(1, 1);

        // --- 2. ДВИЖЕНИЕ ---
        // Мальчик двигается каждый шаг
        m_x += m_dir;
        if (m_x >= 15) m_dir = -1; // Отскок от правой стены
        if (m_x <= 0)  m_dir = 1;  // Отскок от левой стены

        // Девочка двигается каждый второй шаг (медленнее)
        if (step % 2 == 0) {
            f_x += f_dir;
            if (f_x >= 15) f_dir = -1;
            if (f_x <= 0)  f_dir = 1;
        }
        step++;

        // --- 3. ОЖИДАНИЕ И ПРОВЕРКА КЛАВИАТУРЫ ---
        // Вместо одного длинного _delay_ms(200), делаем много коротких,
        // чтобы чаще проверять кнопку
        for (int i = 0; i < 20; i++) { 
            unsigned char c = USART_Check();
            
            if (c == ' ') { // Если нажат ПРОБЕЛ
                // Проверяем столкновение (они друг над другом)
                if (m_x == f_x) {
                    // СЦЕНА "BABY"
                    LCD_SendByte(0x01, 0); _delay_ms(2);
                    LCD_Goto(0, 6);
                    LCD_String("BABY");
                    _delay_ms(2000); // Показываем 2 секунды
                    // После этого продолжаем с тех же позиций
                } 
                else {
                    // ОБМЕН СТРОКАМИ
                    int8_t temp = m_row;
                    m_row = f_row;
                    f_row = temp;
                }
            }
            _delay_ms(10); // Маленькая пауза (итого 20 * 10 = 200мс на шаг)
        }
    }
}