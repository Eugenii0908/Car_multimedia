#include "stm32f446xx.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <float.h>

volatile char frame_gps[2048] = {0};
char gps_data[2048] = {0};
volatile bool frame_gps_saved = false;
uint16_t size_gps_data = 0;

struct Gnss_data
{
    float longitude;
    float latitude;
    float speed;
    float course;
    uint8_t num_sat;
    float hdop;
};
struct Gnss_data Gnss_data;

// Считывание байта
uint8_t uart1_read_byte()
{
    while (!(USART1->SR & USART_SR_RXNE))
        ;
    return (USART1->DR);
}

// Проверка возможности считывания бита данных
uint8_t uart1_available(void)
{
    if (USART1->SR & USART_SR_RXNE)
        return 1;
    return 0;
}

// // Отправка одного байта uart1
// void uart1_write_byte(uint8_t one_byte)
// {
//     while (!(USART1->SR & USART_SR_TXE))
//         ;
//     USART1->DR = one_byte;
// }

// // Отправка строки uart1
// void uart1_write_string(const char *s)
// {
//     while (*s != '\0')
//         uart1_write_byte((uint8_t)*s++);
// }

// Отправка одного байта uart2
void uart2_write_byte(uint8_t one_byte)
{
    while (!(USART2->SR & USART_SR_TXE))
        ;
    USART2->DR = one_byte;
}

// Вывод в Serial строки
void uart2_write_string(const char *s)
{
    while (*s != '\0')
        uart2_write_byte((uint8_t)*s++);
}

// Инициализация UART1 для модуля ГНСС
void uart1_init()
{
    // Частота без предделителя (16МГц)
    RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos);          // Разрешить тактирование на шине APB2 (USART1)
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7); // Сбрасываем биты
    GPIOB->MODER |= (2 << GPIO_MODER_MODER6_Pos) |
                    (2 << GPIO_MODER_MODER7_Pos); // Включаем альтернативную функцию для пинов
    GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFSEL6_Pos) |
                     (7 << GPIO_AFRL_AFSEL7_Pos); // Выбираем альтернативную функцию для нашего UART
    USART1->BRR = (104 << USART_BRR_DIV_Mantissa_Pos) |
                  (3 << USART_BRR_DIV_Fraction_Pos); // Записываем скорость (9600 бод)
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART1->CR3 |= USART_CR3_DMAR;
    USART1->CR1 |= USART_CR1_IDLEIE;
}

// Инициализация UART2 для передачи данных на ПК
void uart2_init()
{
    // Частота без предделителя (16МГц)
    RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN_Pos); // Разрешить тактирование на шине APB2 (USART1)                                    // Включение тактирования GPIOB
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 |
                      GPIO_MODER_MODER3); // Сбрасываем биты
    GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) |
                    (2 << GPIO_MODER_MODER3_Pos); // Включаем альтернативную функцию для пинов
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) |
                     (7 << GPIO_AFRL_AFSEL3_Pos); // Выбираем альтернативную функцию для нашего UART
    USART2->BRR = (104 << USART_BRR_DIV_Mantissa_Pos) |
                  (3 << USART_BRR_DIV_Fraction_Pos); // Записываем скорость (9600 бод)
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    NVIC_EnableIRQ(USART1_IRQn);
}

void dma2_init()
{
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA2EN_Pos);
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;

    // Ждём, пока реально выключится
    while (DMA2_Stream2->CR & DMA_SxCR_EN)
        ;

    // Полностью очищаем настройки Stream 2
    DMA2_Stream2->CR = 0;

    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream2->M0AR = (uint32_t)frame_gps;
    DMA2_Stream2->CR |= (4 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | (DMA_SxCR_MINC);
    DMA2_Stream2->NDTR = sizeof(frame_gps);
    DMA2_Stream2->CR |= DMA_SxCR_EN;
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_IDLE)
    {
        volatile uint32_t tmp;

        tmp = USART1->SR;
        tmp = USART1->DR;

        uint16_t size = sizeof(frame_gps) - DMA2_Stream2->NDTR;

        // Останавливаем DMA
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;

        // Ждём фактического отключения
        while (DMA2_Stream2->CR & DMA_SxCR_EN)
            ;

        DMA2->LIFCR = DMA_LIFCR_CTCIF2 |
                      DMA_LIFCR_CHTIF2 |
                      DMA_LIFCR_CTEIF2 |
                      DMA_LIFCR_CDMEIF2 |
                      DMA_LIFCR_CFEIF2;

        if (size > 0 && size < sizeof(gps_data))
        {
            size_gps_data = size;

            for (uint16_t i = 0; i < size; i++)
            {
                gps_data[i] = frame_gps[i];
            }

            gps_data[size] = '\0';

            frame_gps_saved = true;
        }

        // Подготавливаем DMA заново
        DMA2_Stream2->NDTR = sizeof(frame_gps);
        DMA2_Stream2->M0AR = (uint32_t)frame_gps;

        // Запускаем DMA
        DMA2_Stream2->CR |= DMA_SxCR_EN;
    }
}

float convert_coordinates_lat(char *source_data, uint16_t message_pos, uint16_t data_size)
{
    uint16_t i = message_pos;

    // Ищем запятую
    while (i < data_size && source_data[i] != ',')
        i++;

    // Слишком короткое поле
    if (i - message_pos < 4)
        return FLT_MAX;

    float degrees = (source_data[message_pos] - '0') * 10.0f + (source_data[message_pos + 1] - '0');

    float minutes = (source_data[message_pos + 2] - '0') * 10.0f + (source_data[message_pos + 3] - '0');

    float decimal = 0.1f;
    uint16_t j = message_pos + 5; // пропускаем ddmm.

    while (j < i)
    {
        minutes += (source_data[j] - '0') * decimal;
        decimal *= 0.1f;
        j++;
    }

    return degrees + minutes / 60.0f;
}

float convert_coordinates_long(char *source_data, uint16_t message_pos, uint16_t data_size)
{
    uint16_t i = message_pos;

    // Ищем запятую
    while (i < data_size && source_data[i] != ',')
        i++;

    // Слишком короткое поле
    if (i - message_pos < 5)
        return FLT_MAX;

    float degrees = (source_data[message_pos] - '0') * 100.0f + (source_data[message_pos + 1] - '0') * 10.0f + (source_data[message_pos + 2] - '0');

    float minutes = (source_data[message_pos + 3] - '0') * 10.0f + (source_data[message_pos + 4] - '0');

    float decimal = 0.1f;
    uint16_t j = message_pos + 6; // пропускаем dddmm.

    while (j < i)
    {
        minutes += (source_data[j] - '0') * decimal;
        decimal *= 0.1f;
        j++;
    }

    return degrees + minutes / 60.0f;
}

float convert_float(char *source_data, uint16_t message_pos, uint16_t data_size)
{
    uint16_t i = message_pos;
    uint16_t point_pos = 0;

    // Ищем запятую
    while (i < data_size && source_data[i] != ',')
    {
        if (source_data[i] == '.')
            point_pos = i;
        i++;
    }

    if (point_pos == 0)
        return FLT_MAX;

    float number = 0.0f;
    float decimal = 0.1f;
    float integer = 1.0f;
    int16_t j = point_pos - 1;
    while (j >= message_pos)
    {
        number += (source_data[j] - '0') * integer;
        integer *= 10.0f;
        j--;
    }

    uint16_t k = point_pos + 1;
    while (k < i)
    {
        number += (source_data[k] - '0') * decimal;
        decimal *= 0.1f;
        k++;
    }

    return number;
}

uint16_t convert_int(char *source_data, uint16_t message_pos, uint16_t data_size)
{
    uint16_t i = message_pos;

    // Ищем запятую
    while (i < data_size && source_data[i] != ',')
    {
        i++;
    }

    uint16_t number = 0;
    uint16_t integer = 1;
    int16_t j = i - 1;
    while (j >= message_pos)
    {
        number += (source_data[j] - '0') * integer;
        integer *= 10;
        j--;
    }
    return number;
}

bool upload_gnss_data(uint16_t data_size, char *source_data)
{
    bool start_message = false, rmc = false, gga = false;
    uint8_t data_register = 0b00000000;
    uint16_t message_pos = 0;
    uint8_t num_field = 0;
    for (uint16_t i = 0; i < data_size; i++)
    {
        message_pos++;
        if (source_data[i] == '$')
        {
            start_message = true;
            message_pos = 0;
            rmc = false;
            gga = false;
            num_field = 0;
        }
        if (data_register == 0b00111111)
            return true;
        if (start_message && (message_pos == 5))
        {
            if ((source_data[i] == 'C') &&
                (source_data[i - 1] == 'M') &&
                (source_data[i - 2] == 'R'))
            {
                rmc = true;
            }
            else if ((source_data[i] == 'A') &&
                     (source_data[i - 1] == 'G') &&
                     (source_data[i - 2] == 'G'))
            {
                gga = true;
            }
            else
                start_message = false;
        }
        if (rmc)
        {
            if (source_data[i] == ',')
            {
                num_field++;
                continue;
            }
            switch (num_field)
            {
            case 2:
                if (source_data[i] == 'V')
                    return false;
                break;
            case 3:
                if (!((1 << 0) & data_register))
                {
                    Gnss_data.latitude = convert_coordinates_lat(source_data, i, data_size);
                    data_register |= 0b00000001;
                }
                break;
            case 4:
                if (source_data[i] == 'S')
                    Gnss_data.latitude *= -1;
                break;
            case 5:
                if (!((1 << 1) & data_register))
                {
                    Gnss_data.longitude = convert_coordinates_long(source_data, i, data_size);
                    data_register |= 0b00000010;
                }
                break;
            case 6:
                if (source_data[i] == 'W')
                    Gnss_data.longitude *= -1;
                break;
            case 7:
                if (!((1 << 2) & data_register))
                {
                    Gnss_data.speed = (convert_float(source_data, i, data_size)) * 1.852f;
                    data_register |= 0b00000100;
                }
                break;
            case 8:
                if (!((1 << 3) & data_register))
                {
                    Gnss_data.course = convert_float(source_data, i, data_size);
                    data_register |= 0b00001000;
                }
                start_message = false;
                break;
            }
        }
        if (gga)
        {
            if (source_data[i] == ',')
            {
                num_field++;
                continue;
            }
            switch (num_field)
            {
            case 7:
                if (!((1 << 4) & data_register))
                {
                    Gnss_data.num_sat = convert_int(source_data, i, data_size);
                    data_register |= 0b00010000;
                }
                break;
            case 8:
                if (!((1 << 5) & data_register))
                {
                    Gnss_data.hdop = convert_float(source_data, i, data_size);
                    data_register |= 0b00100000;
                }
                start_message = false;
                break;
            }
        }
    }
    return (data_register == 0b00111111);
}

void uart2_write_uint(uint16_t value)
{
    char buffer[5];
    uint8_t i = 0;

    if (value == 0)
    {
        uart2_write_byte('0');
        return;
    }

    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }

    while (i > 0)
    {
        uart2_write_byte(buffer[--i]);
    }
}
void uart2_write_uint32(uint32_t value)
{
    char buffer[10];
    uint8_t i = 0;

    if (value == 0)
    {
        uart2_write_byte('0');
        return;
    }

    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }

    while (i > 0)
    {
        uart2_write_byte(buffer[--i]);
    }
}
void uart2_write_float5(float value)
{
    if (value < 0)
    {
        uart2_write_byte('-');
        value = -value;
    }

    uint32_t integer = (uint32_t)value;
    uint32_t fraction = (uint32_t)((value - integer) * 100000);

    uart2_write_uint32(integer);
    uart2_write_byte('.');

    uart2_write_byte('0' + (fraction / 10000) % 10);
    uart2_write_byte('0' + (fraction / 1000) % 10);
    uart2_write_byte('0' + (fraction / 100) % 10);
    uart2_write_byte('0' + (fraction / 10) % 10);
    uart2_write_byte('0' + fraction % 10);
}

int main()
{
    // Служебный светодиод
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);
    GPIOA->MODER &= ~GPIO_MODER_MODER5;
    GPIOA->MODER |= (1 << GPIO_MODER_MODER5_Pos);
    GPIOA->BSRR = GPIO_BSRR_BR5;

    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos); // Включение тактирования GPIOB (UART)
    uart1_init();
    uart2_init();
    dma2_init();
    uart2_write_string("AFTER DMA INIT\r\n");

    while (1)
    {
        // GPIOA->ODR ^= GPIO_ODR_OD5;

        if (frame_gps_saved)
        {
            if (upload_gnss_data(size_gps_data, gps_data))
            {
                uart2_write_string("LAT: ");
                uart2_write_float5(Gnss_data.latitude);
                uart2_write_string("\r\n");

                uart2_write_string("LON: ");
                uart2_write_float5(Gnss_data.longitude);
                uart2_write_string("\r\n");

                uart2_write_string("SPEED: ");
                uart2_write_float5(Gnss_data.speed);
                uart2_write_string("\r\n");

                uart2_write_string("COURSE: ");
                uart2_write_float5(Gnss_data.course);
                uart2_write_string("\r\n");

                uart2_write_string("SAT: ");
                uart2_write_uint(Gnss_data.num_sat);
                uart2_write_string("\r\n");

                uart2_write_string("HDOP: ");
                uart2_write_float5(Gnss_data.hdop);
                uart2_write_string("\r\n");

                uart2_write_string("----------------\r\n");
            }

            frame_gps_saved = false;
        }

        // for (int i = 0; i < 100; i++)
        // {
        //     if (frame[i] == '$')
        //     {
        //         uart2_write_string("Found '$' in frame!\r\n");
        //         // Вывести первые 80 символов
        //         for (int j = i; j < i + 80 && j < sizeof(frame); j++)
        //         {
        //             uart2_write_byte(frame[j]);
        //         }
        //         uart2_write_string("\r\n");
        //         break;
        //     }
        // }
        // for (volatile uint32_t i = 0; i < 100; i++)
        // {
        // }
    }
    // while (1)
    // {
    //     // if (DMA2_Stream2->NDTR < sizeof(frame))
    //     // {
    //     //     uart2_write_string("DMA WORKS\r\n");

    //     //     for (int i = 0; i < 100; i++)
    //     //     {
    //     //         uart2_write_byte(frame[i]);
    //     //     }

    //     //     while (1)
    //     //         ;
    //     // }
    //     // if (uart1_available())
    //     // {
    //     //     uint8_t byte = uart1_read_byte();

    //     //     // Ждём начало NMEA-пакета
    //     //     if (byte != '$')
    //     //         continue;

    //     //     char package[256];
    //     //     int i = 0;
    //     //     bool valid = false;
    //     //     // Записываем
    //     //     package[i] = byte;
    //     //     i++;
    //     //     // Читаем пакет
    //     //     while (i < 255)
    //     //     {
    //     //         byte = uart1_read_byte();
    //     //         package[i] = byte;
    //     //         i++;
    //     //         if (byte == '\n')
    //     //             break;
    //     //     }

    //     //     package[i] = '\0';

    //     //     // Проверяем тип пакета
    //     //     if (i >= 6)
    //     //     {
    //     //         if ((package[3] == 'R' &&
    //     //              package[4] == 'M' &&
    //     //              package[5] == 'C') ||
    //     //             (package[3] == 'V' &&
    //     //              package[4] == 'T' &&
    //     //              package[5] == 'G'))
    //     //         {
    //     //             valid = true;
    //     //         }
    //     //     }

    //     //     if (valid)
    //     //     {
    //     //         uart2_write_string(package);
    //     //     }
    //     // }
    // }
}