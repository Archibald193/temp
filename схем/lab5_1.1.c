#define F_CPU 8000000UL // Частота 8 МГц (как кварц на плате)

#include <avr/io.h>
#include <util/delay.h>

// --- НАСТРОЙКИ ПОДКЛЮЧЕНИЯ (строго по схеме методички) ---
#define LCD_DATA_PORT PORTB
#define LCD_DATA_DDR  DDRB
// Линии управления на Порту D
#define LCD_CTRL_PORT PORTD
#define LCD_CTRL_DDR  DDRD
#define LCD_RS        PD4
#define LCD_RW        PD5
#define LCD_E         PD6

// --- ФУНКЦИИ УПРАВЛЕНИЯ ЭКРАНОМ ---

void LCD_PulseEnable(void) {
    LCD_CTRL_PORT |= (1 << LCD_E);  
    _delay_us(1);                   
    LCD_CTRL_PORT &= ~(1 << LCD_E); 
    _delay_us(50);                  
}

void LCD_SendNibble(uint8_t nibble) {
    // Сначала очищаем биты данных (PB4-PB7), сохраняя остальные (PB0-PB3)
    LCD_DATA_PORT &= 0x0F;          
    // Записываем новые данные
    LCD_DATA_PORT |= (nibble & 0xF0); 
    LCD_PulseEnable();
}

void LCD_SendByte(uint8_t data, uint8_t is_data) {
    if (is_data) {
        LCD_CTRL_PORT |= (1 << LCD_RS); // RS=1 (Данные)
    } else {
        LCD_CTRL_PORT &= ~(1 << LCD_RS); // RS=0 (Команда)
    }
    
    LCD_CTRL_PORT &= ~(1 << LCD_RW); // RW=0 (Запись)

    // Шлем старшие 4 бита
    LCD_SendNibble(data);
    // Шлем младшие 4 бита (сдвигая их)
    LCD_SendNibble(data << 4);
}

void LCD_Init(void) {
    LCD_DATA_DDR |= 0xF0; // PB4-PB7 на выход
    LCD_CTRL_DDR |= (1<<LCD_RS)|(1<<LCD_RW)|(1<<LCD_E); // PD4-PD6 на выход

    _delay_ms(50); // Ждем, пока экран "проснется" после подачи питания

    // Сброс настроек экрана (магическая последовательность)
    LCD_CTRL_PORT &= ~(1 << LCD_RS);
    LCD_CTRL_PORT &= ~(1 << LCD_RW);
    LCD_CTRL_PORT &= ~(1 << LCD_E);
    
    LCD_SendNibble(0x30); _delay_ms(5);
    LCD_SendNibble(0x30); _delay_us(150);
    LCD_SendNibble(0x30); _delay_us(150);
    LCD_SendNibble(0x20); _delay_us(150); // Переключаем в 4-битный режим

    // Настройка параметров отображения
    LCD_SendByte(0x28, 0); // 4 бита, 2 строки
    LCD_SendByte(0x0C, 0); // Дисплей ВКЛ, курсор ВЫКЛ
    LCD_SendByte(0x06, 0); // Курсор едет вправо
    LCD_SendByte(0x01, 0); // Очистить экран
    _delay_ms(20);
}

void LCD_String(char *str) {
    while (*str) {
        LCD_SendByte(*str, 1);
        str++;
    }
}

// Функция установки курсора (строка 0 или 1, столбец 0-15)
void LCD_Goto(uint8_t row, uint8_t col) {
    uint8_t address;
    if (row == 0) address = 0x80 + col;
    else          address = 0xC0 + col;
    LCD_SendByte(address, 0);
}

// --- ГЛАВНАЯ ПРОГРАММА ---
int main(void) {
    LCD_Init(); // Инициализация экрана

    while (1) {
        // Очистка
        LCD_SendByte(0x01, 0); 
        _delay_ms(20);

        // Вывод текста 1
        LCD_Goto(0, 0);
        LCD_String("Hello");
        LCD_Goto(1, 0);
        LCD_String("World!");
        
        _delay_ms(2000); // Пауза 2 сек

        // Очистка
        LCD_SendByte(0x01, 0);
        _delay_ms(20);

        // Вывод текста 2
        LCD_Goto(0, 0);
        LCD_String("Slava");
        LCD_Goto(1, 0);
        LCD_String("KPSS!");

        _delay_ms(2000);
    }
}
