#include "stm32f10x.h"
#include <stdint.h>

// ========== Configuration ==========
#define LCD_I2C_PORT    I2C1           // LCD on I2C1 (PB6/PB7)
#define SHT45_I2C_PORT  I2C2           // SHT45 on I2C2 (PB10/PB11)
#define LCD_ADDR        0x27
#define SHT45_ADDR      0x44
#define I2C_TIMEOUT     10000

// ========== LCD Commands ==========
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_FUNCTIONSET     0x20
#define LCD_SETDDRAMADDR    0x80

#define LCD_ENTRYLEFT       0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00

#define LCD_DISPLAYON       0x04
#define LCD_CURSORON        0x02
#define LCD_BLINKON         0x01

#define LCD_4BITMODE        0x00
#define LCD_2LINE           0x08
#define LCD_5x8DOTS         0x00

#define BL                  0x08
#define EN                  0x04

// ========== Function Declarations ==========
void system_init(void);
void i2c_init(void);
void i2c2_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint8_t i2c_start_on(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t is_read);
void i2c_stop_on(I2C_TypeDef *I2Cx);
uint8_t i2c_write_byte_on(I2C_TypeDef *I2Cx, uint8_t data);
uint8_t i2c_read_byte_on(I2C_TypeDef *I2Cx, uint8_t ack_flag);
void lcd_send_nibble(uint8_t data);
void lcd_write_byte(uint8_t data, uint8_t mode);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* text, uint8_t row, uint8_t col);
void lcd_print_value(int16_t value_x100, uint8_t row, uint8_t col);  // Updated to x100

uint8_t sht45_start_measurement(void);
uint8_t sht45_read_data(uint16_t *temp_raw, uint16_t *rh_raw);
int16_t sht45_calc_temp_x100(uint16_t raw);  // Updated to x100
int16_t sht45_calc_rh_x100(uint16_t raw);    // Updated to x100

// ========== Main Function ==========
int main(void) {
    system_init();
    i2c_init();       // I2C1 for LCD
    i2c2_init();      // I2C2 for SHT45

    delay_ms(100);

    lcd_init();
    lcd_clear();
    lcd_print("Initializing...", 0, 0);
    delay_ms(500);

    uint16_t temp_raw, rh_raw;
    int16_t temp_x10, rh_x10;  // 名称未改，但实际存储的是 x100 值

    while(1) {
        if (sht45_start_measurement() && sht45_read_data(&temp_raw, &rh_raw)) {
            temp_x10 = sht45_calc_temp_x100(temp_raw);  // 改为 x100 版本
            rh_x10 = sht45_calc_rh_x100(rh_raw);        // 改为 x100 版本

            lcd_clear();
            lcd_print("Temp: ", 0, 0);
            lcd_print_value(temp_x10, 0, 6);
            lcd_print(" C", 0, 12);  // 调整列位置，预留两位小数空间

            lcd_print("RH:   ", 1, 0);
            lcd_print_value(rh_x10, 1, 6);
            lcd_print(" %", 1, 12);  // 同上
        } else {
            lcd_clear();
            lcd_print("SHT45 Error!", 0, 0);
        }

        delay_ms(20000); // 60秒更新一次
    }
}

// ========== Function Implementations ==========

void system_init(void) {
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 只需启用 GPIOB（I2C1 和 I2C2 都用 GPIOB）
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_I2C2EN;
}

void i2c_init(void) {
    // I2C1: PB6=SCL, PB7=SDA
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOB->CRL |= (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0) |
                  (GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0) |
                  (GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0) |
                  (GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0);

    GPIOB->BSRR = GPIO_BSRR_BS6 | GPIO_BSRR_BS7;

    I2C1->CR1 |= I2C_CR1_SWRST;
    delay_us(10);
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 36;
    I2C1->CCR = 180;
    I2C1->TRISE = 37;
    I2C1->CR1 = I2C_CR1_PE;

    delay_ms(10);
}

void i2c2_init(void) {
    // I2C2: PB10=SCL, PB11=SDA
    GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10 | GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= (GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0) |
                  (GPIO_CRH_CNF10_1 | GPIO_CRH_CNF10_0) |
                  (GPIO_CRH_MODE11_1 | GPIO_CRH_MODE11_0) |
                  (GPIO_CRH_CNF11_1 | GPIO_CRH_CNF11_0);

    GPIOB->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BS11;

    I2C2->CR1 |= I2C_CR1_SWRST;
    delay_us(10);
    I2C2->CR1 &= ~I2C_CR1_SWRST;

    I2C2->CR2 = 36;
    I2C2->CCR = 180;
    I2C2->TRISE = 37;
    I2C2->CR1 = I2C_CR1_PE;

    delay_ms(10);
}

void delay_us(uint32_t us) {
    volatile uint32_t count = us * 24;
    while(count--);
}

void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

uint8_t i2c_start_on(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t is_read) {
    uint32_t timeout = I2C_TIMEOUT;
    while(I2Cx->SR2 & I2C_SR2_BUSY) {
        if(--timeout == 0) return 0;
    }

    I2Cx->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & I2C_SR1_SB)) {
        if(--timeout == 0) return 0;
    }

    I2Cx->DR = (addr << 1) | (is_read ? 1 : 0);
    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & I2C_SR1_ADDR)) {
        if(I2Cx->SR1 & I2C_SR1_AF) {
            I2Cx->SR1 &= ~I2C_SR1_AF;
            I2Cx->CR1 |= I2C_CR1_STOP;
            return 0;
        }
        if(--timeout == 0) return 0;
    }

    (void)I2Cx->SR1;
    (void)I2Cx->SR2;
    return 1;
}

void i2c_stop_on(I2C_TypeDef *I2Cx) {
    I2Cx->CR1 |= I2C_CR1_STOP;
    while(I2Cx->CR1 & I2C_CR1_STOP);
}

uint8_t i2c_write_byte_on(I2C_TypeDef *I2Cx, uint8_t data) {
    uint32_t timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & I2C_SR1_TXE)) {
        if(--timeout == 0) return 0;
    }

    I2Cx->DR = data;
    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & I2C_SR1_BTF)) {
        if(I2Cx->SR1 & I2C_SR1_AF) {
            I2Cx->SR1 &= ~I2C_SR1_AF;
            return 0;
        }
        if(--timeout == 0) return 0;
    }
    return 1;
}

uint8_t i2c_read_byte_on(I2C_TypeDef *I2Cx, uint8_t ack_flag) {
    if (ack_flag) {
        I2Cx->CR1 |= I2C_CR1_ACK;
    } else {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
    }

    uint32_t timeout = I2C_TIMEOUT;
    while (!(I2Cx->SR1 & I2C_SR1_RXNE)) {
        if (--timeout == 0) return 0xFF;
    }

    return I2Cx->DR;
}

void lcd_send_nibble(uint8_t data) {
    if(i2c_start_on(LCD_I2C_PORT, LCD_ADDR, 0)) {
        i2c_write_byte_on(LCD_I2C_PORT, data | EN);
        delay_us(1);
        i2c_write_byte_on(LCD_I2C_PORT, data & ~EN);
        i2c_stop_on(LCD_I2C_PORT);
    }
    delay_us(50);
}

void lcd_write_byte(uint8_t data, uint8_t mode) {
    uint8_t high_nibble = (data & 0xF0) | mode;
    uint8_t low_nibble = ((data << 4) & 0xF0) | mode;
    lcd_send_nibble(high_nibble);
    lcd_send_nibble(low_nibble);
}

void lcd_init(void) {
    delay_ms(50);
    lcd_send_nibble(0x30);
    delay_ms(5);
    lcd_send_nibble(0x30);
    delay_us(150);
    lcd_send_nibble(0x30);
    lcd_send_nibble(0x20);

    lcd_write_byte(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE, 0);
    delay_us(50);
    lcd_write_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON, 0);
    delay_us(50);
    lcd_write_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0);
    delay_us(50);
    lcd_clear();
}

void lcd_clear(void) {
    lcd_write_byte(LCD_CLEARDISPLAY, 0);
    delay_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_write_byte(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
    delay_us(50);
}

void lcd_print(const char* text, uint8_t row, uint8_t col) {
    lcd_set_cursor(col, row);
    while (*text) {
        lcd_write_byte(*text++, 1);
        delay_us(50);
    }
}

// ✅ 修改：支持显示小数点后两位
void lcd_print_value(int16_t value_x100, uint8_t row, uint8_t col) {
    char buffer[9];  // 最大格式： -XXX.XX\0
    uint8_t idx = 0;

    if (value_x100 < 0) {
        buffer[idx++] = '-';
        value_x100 = -value_x100;
    }

    int16_t integer = value_x100 / 100;
    uint8_t fraction_tens = (value_x100 % 100) / 10;
    uint8_t fraction_units = value_x100 % 10;

    // 整数部分
    if (integer == 0) {
        buffer[idx++] = '0';
    } else {
        char digits[5];
        uint8_t count = 0;
        int16_t n = integer;
        while (n && count < 5) {
            digits[count++] = (n % 10) + '0';
            n /= 10;
        }
        for (int8_t i = count - 1; i >= 0; i--) {
            buffer[idx++] = digits[i];
        }
    }

    buffer[idx++] = '.';
    buffer[idx++] = fraction_tens + '0';
    buffer[idx++] = fraction_units + '0';
    buffer[idx] = '\0';

    lcd_print(buffer, row, col);
}

uint8_t sht45_start_measurement(void) {
    if (!i2c_start_on(SHT45_I2C_PORT, SHT45_ADDR, 0)) return 0;
    if (!i2c_write_byte_on(SHT45_I2C_PORT, 0xFD)) {  // High precision mode
        i2c_stop_on(SHT45_I2C_PORT);
        return 0;
    }
    i2c_stop_on(SHT45_I2C_PORT);
    delay_ms(20);
    return 1;
}

uint8_t sht45_read_data(uint16_t *temp_raw, uint16_t *rh_raw) {
    uint8_t data[6];

    if (!i2c_start_on(SHT45_I2C_PORT, SHT45_ADDR, 1)) return 0;

    for (int i = 0; i < 6; i++) {
        data[i] = i2c_read_byte_on(SHT45_I2C_PORT, (i < 5));
    }

    i2c_stop_on(SHT45_I2C_PORT);

    *temp_raw = (data[0] << 8) | data[1];
    *rh_raw = (data[3] << 8) | data[4];

    return 1;
}

// ✅ 修改：返回 x100 精度
int16_t sht45_calc_temp_x100(uint16_t raw) {
    return (int16_t)(-4500 + (17500LL * raw) / 65535);
}

// ✅ 修改：返回 x100 精度
int16_t sht45_calc_rh_x100(uint16_t raw) {
    return (int16_t)(-600 + (12500LL * raw) / 65535);
}
