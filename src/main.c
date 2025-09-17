#include "stm32f10x.h"
#include <stdint.h>

// ========== Configuration ==========
#define I2C_PORT        I2C1           // Using I2C1
#define LCD_ADDR        0x27           // Your LCD I2C address (7-bit)
#define I2C_TIMEOUT     50000          // Increased timeout

// Debug LED on PC13 (common on STM32F103 boards)
#define LED_PORT        GPIOC
#define LED_PIN         13

// ========== LCD Commands ==========
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_FUNCTIONSET     0x20
#define LCD_SETDDRAMADDR    0x80

// Entry Mode
#define LCD_ENTRYLEFT       0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Display Control
#define LCD_DISPLAYON       0x04
#define LCD_CURSORON        0x02
#define LCD_BLINKON         0x01

// Function Set
#define LCD_4BITMODE        0x00
#define LCD_2LINE           0x08
#define LCD_5x8DOTS         0x00

// PCF8574 Control Bits (Backlight + Enable)
#define BL                  0x08   // Backlight ON
#define EN                  0x04   // Enable bit

// Global variables for debugging
volatile uint8_t i2c_error = 0;

// ========== Function Declarations ==========
void system_init(void);
void led_init(void);
void led_on(void);
void led_off(void);
void led_blink(uint8_t times);
void i2c_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint8_t i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write_byte(uint8_t data);
uint8_t i2c_scan_device(uint8_t addr);
void lcd_send_nibble(uint8_t data);
void lcd_write_byte(uint8_t data, uint8_t mode);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* text, uint8_t row, uint8_t col);

// ========== Main Function ==========
int main(void) {
    system_init();
    led_init();
    i2c_init();
    
    // LED indicates system is running
    led_blink(3);
    delay_ms(500);
    
    // Scan for I2C device
    if(i2c_scan_device(LCD_ADDR)) {
        led_blink(1); // 1 blink = device found
    } else {
        // 5 rapid blinks = device not found
        for(int i = 0; i < 10; i++) {
            led_on();
            delay_ms(100);
            led_off();
            delay_ms(100);
        }
        // Continue anyway, might be address issue
    }
    
    delay_ms(100);
    
    // Try different addresses if 0x27 doesn't work
    uint8_t test_addresses[] = {0x27, 0x3F, 0x26, 0x20};
    uint8_t found_addr = 0;
    
    for(int i = 0; i < 4; i++) {
        if(i2c_scan_device(test_addresses[i])) {
            found_addr = test_addresses[i];
            break;
        }
    }
    
    if(found_addr) {
        led_blink(2); // 2 blinks = found working address
        // Update LCD address
        // You might need to modify LCD_ADDR define or make it variable
    }
    
    lcd_init();
    
    // Display content
    lcd_clear();
    lcd_print("Hello, STM32!", 0, 0);
    lcd_print("Debug Mode", 1, 0);
    
    led_on(); // Solid LED = success
    
    while(1) {
        // Main loop
        delay_ms(1000);
    }
}

// ========== Function Implementations ==========

// Initialize system clock and GPIO
void system_init(void) {
    // Enable HSI for now (internal 8MHz)
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    
    // Use HSI with PLL * 9 = 72MHz
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    
    // Enable PLL and wait for it to be ready
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    
    // Configure Flash latency
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    
    // Set APB1 prescaler to /2 (36MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    
    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

// Initialize debug LED
void led_init(void) {
    // Configure PC13 as output push-pull
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_1; // 2MHz output
    led_off();
}

void led_on(void) {
    GPIOC->BRR = GPIO_BRR_BR13; // LED is active low
}

void led_off(void) {
    GPIOC->BSRR = GPIO_BSRR_BS13;
}

void led_blink(uint8_t times) {
    for(uint8_t i = 0; i < times; i++) {
        led_on();
        delay_ms(200);
        led_off();
        delay_ms(200);
    }
}

// Initialize I2C1 (PB6=SCL, PB7=SDA)
void i2c_init(void) {
    // Configure PB6 (SCL) and PB7 (SDA) as alternate function open-drain
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOB->CRL |= (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0) |  // 50MHz
                  (GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0) |    // AF open-drain
                  (GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0) |  // 50MHz
                  (GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0);     // AF open-drain
    
    // Reset I2C1
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    
    // Configure I2C for 100kHz (PCLK1 = 36MHz)
    I2C1->CR2 = 36;     // PCLK1 frequency in MHz
    I2C1->CCR = 180;    // 36MHz / (2 * 100kHz) = 180
    I2C1->TRISE = 37;   // (36 * 1000ns) + 1 = 37
    
    // Enable I2C
    I2C1->CR1 = I2C_CR1_PE;
    delay_ms(10);
}

// Microsecond delay
void delay_us(uint32_t us) {
    volatile uint32_t count = us * 18; // Adjusted for 72MHz
    while(count--);
}

// Millisecond delay
void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

// Scan for I2C device
uint8_t i2c_scan_device(uint8_t addr) {
    uint32_t timeout = I2C_TIMEOUT;
    
    // Wait until bus is not busy
    while(I2C1->SR2 & I2C_SR2_BUSY) {
        if(--timeout == 0) return 0;
    }
    
    // Generate start condition
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_SB)) {
        if(--timeout == 0) return 0;
    }
    
    // Send address
    I2C1->DR = (addr << 1) | 0;
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) {
        if(--timeout == 0) break;
    }
    
    if(I2C1->SR1 & I2C_SR1_AF) {
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;
        return 0; // Device not found
    }
    
    // Clear ADDR
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    
    // Generate stop
    I2C1->CR1 |= I2C_CR1_STOP;
    return 1; // Device found
}

// Generate I2C start condition and send address
uint8_t i2c_start(void) {
    uint32_t timeout = I2C_TIMEOUT;
    
    // Wait until bus not busy
    while(I2C1->SR2 & I2C_SR2_BUSY) {
        if(--timeout == 0) {
            i2c_error = 1;
            return 0;
        }
    }
    
    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_SB)) {
        if(--timeout == 0) {
            i2c_error = 2;
            return 0;
        }
    }
    
    // Send address
    I2C1->DR = (LCD_ADDR << 1) | 0;
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if(I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            i2c_stop();
            i2c_error = 3;
            return 0;
        }
        if(--timeout == 0) {
            i2c_error = 4;
            return 0;
        }
    }
    
    // Clear ADDR
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    return 1;
}

// Generate stop condition
void i2c_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
    delay_us(10);
}

// Write byte via I2C
uint8_t i2c_write_byte(uint8_t data) {
    uint32_t timeout = I2C_TIMEOUT;
    
    while(!(I2C1->SR1 & I2C_SR1_TXE)) {
        if(--timeout == 0) {
            i2c_error = 5;
            return 0;
        }
    }
    
    I2C1->DR = data;
    
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_BTF)) {
        if(I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            i2c_error = 6;
            return 0;
        }
        if(--timeout == 0) {
            i2c_error = 7;
            return 0;
        }
    }
    
    return 1;
}

// Send nibble to LCD
void lcd_send_nibble(uint8_t data) {
    data |= BL; // Backlight on
    
    if(i2c_start()) {
        if(i2c_write_byte(data | EN)) {  // EN high
            delay_us(1);
            i2c_write_byte(data & ~EN);  // EN low
        }
        i2c_stop();
    }
    delay_us(50);
}

// Write byte to LCD
void lcd_write_byte(uint8_t data, uint8_t mode) {
    uint8_t high = (data & 0xF0) | mode;
    uint8_t low = ((data << 4) & 0xF0) | mode;
    lcd_send_nibble(high);
    lcd_send_nibble(low);
}

// Initialize LCD
void lcd_init(void) {
    delay_ms(50);
    
    // Initialization sequence
    lcd_send_nibble(0x30);
    delay_ms(5);
    lcd_send_nibble(0x30);
    delay_us(150);
    lcd_send_nibble(0x30);
    lcd_send_nibble(0x20); // 4-bit mode
    
    lcd_write_byte(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS, 0);
    delay_us(50);
    
    lcd_write_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, 0);
    delay_us(50);
    
    lcd_write_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0);
    delay_us(50);
    
    lcd_clear();
}

// Clear LCD
void lcd_clear(void) {
    lcd_write_byte(LCD_CLEARDISPLAY, 0);
    delay_ms(2);
}

// Set cursor position
void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t offsets[] = {0x00, 0x40};
    if(row > 1) row = 1;
    lcd_write_byte(LCD_SETDDRAMADDR | (col + offsets[row]), 0);
    delay_us(50);
}

// Print text on LCD
void lcd_print(const char* text, uint8_t row, uint8_t col) {
    lcd_set_cursor(col, row);
    while(*text) {
        lcd_write_byte(*text++, 1);
        delay_us(50);
    }
}
