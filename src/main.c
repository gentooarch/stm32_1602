#include "stm32f10x.h"
#include <stdint.h>

// ========== Configuration ==========
#define I2C_PORT        I2C1           // Using I2C1
#define LCD_ADDR        0x27           // Your LCD I2C address (7-bit)
#define I2C_TIMEOUT     10000          // Timeout counter

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

// ========== Function Declarations ==========
void system_init(void);
void i2c_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint8_t i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write_byte(uint8_t data);
void lcd_send_nibble(uint8_t data);
void lcd_write_byte(uint8_t data, uint8_t mode);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* text, uint8_t row, uint8_t col);

// ========== Main Function ==========
int main(void) {
    system_init();
    i2c_init();
    
    delay_ms(100); // Allow system to stabilize
    
    lcd_init();
    
    // Display content
    lcd_clear();
    lcd_print("Hello, STM32!", 0, 0);
    lcd_print("No HAL Used!", 1, 0);
    
    while(1) {
        // Main loop - LCD content is displayed
        delay_ms(1000);
    }
}

// ========== Function Implementations ==========

// Initialize system clock and GPIO
void system_init(void) {
    // Enable HSE and wait for it to be ready
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));
    
    // Configure Flash latency for 72MHz
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    
    // Configure PLL: HSE * 9 = 72MHz
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    
    // Enable PLL and wait for it to be ready
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    
    // Set APB1 prescaler to /2 (36MHz) and APB2 to /1 (72MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    
    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    // Enable GPIOB and I2C1 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

// Initialize I2C1 (PB6=SCL, PB7=SDA)
void i2c_init(void) {
    // Enable alternate function clock
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    
    // Configure PB6 (SCL) and PB7 (SDA) as alternate function open-drain
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOB->CRL |= (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0) |  // 50MHz output
                  (GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0) |    // AF open-drain
                  (GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0) |  // 50MHz output
                  (GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0);     // AF open-drain
    
    // Set pins high initially
    GPIOB->BSRR = GPIO_BSRR_BS6 | GPIO_BSRR_BS7;
    
    // Reset I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;
    delay_us(10);
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    
    // Configure I2C timing for 100kHz (PCLK1 = 36MHz after APB1 prescaler /2)
    I2C1->CR2 = 36; // APB1 frequency in MHz
    I2C1->CCR = 180; // 36MHz / (2 * 100kHz) = 180
    I2C1->TRISE = 37; // (36MHz * 1000ns) + 1 = 37
    
    // Enable I2C1
    I2C1->CR1 = I2C_CR1_PE;
    
    delay_ms(10); // Allow I2C to stabilize
}

// Microsecond delay (approximate, depends on optimization)
void delay_us(uint32_t us) {
    volatile uint32_t count = us * 24; // Approximate for 72MHz
    while(count--);
}

// Millisecond delay
void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

// Generate I2C start condition and send device address
uint8_t i2c_start(void) {
    uint32_t timeout = I2C_TIMEOUT;
    
    // Wait until bus is not busy
    while(I2C1->SR2 & I2C_SR2_BUSY) {
        if(--timeout == 0) return 0;
    }
    
    // Generate start condition
    I2C1->CR1 |= I2C_CR1_START;
    
    // Wait for start condition generated
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_SB)) {
        if(--timeout == 0) return 0;
    }
    
    // Send device address with write bit
    I2C1->DR = (LCD_ADDR << 1) | 0; // Write operation
    
    // Wait for address sent
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if(I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF; // Clear ACK failure
            i2c_stop();
            return 0;
        }
        if(--timeout == 0) return 0;
    }
    
    // Clear ADDR flag by reading SR1 and SR2
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    
    return 1;
}

// Generate I2C stop condition
void i2c_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP); // Wait for stop condition to complete
}

// Write one byte via I2C
uint8_t i2c_write_byte(uint8_t data) {
    uint32_t timeout = I2C_TIMEOUT;
    
    // Wait until data register is empty
    while(!(I2C1->SR1 & I2C_SR1_TXE)) {
        if(--timeout == 0) return 0;
    }
    
    // Send data
    I2C1->DR = data;
    
    // Wait for byte transfer finished
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_BTF)) {
        if(I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            return 0;
        }
        if(--timeout == 0) return 0;
    }
    
    return 1;
}

// Send high 4 bits (nibble) to LCD with enable pulse
void lcd_send_nibble(uint8_t data) {
    data |= BL; // Turn on backlight
    
    // Send data with EN=1, then EN=0 in single I2C transaction
    if(i2c_start()) {
        i2c_write_byte(data | EN);   // EN=1
        delay_us(1);                 // Hold enable high
        i2c_write_byte(data & ~EN);  // EN=0
        i2c_stop();
    }
    delay_us(50);
}

// Write one byte to LCD (command or data)
void lcd_write_byte(uint8_t data, uint8_t mode) {
    uint8_t high_nibble = (data & 0xF0) | mode;
    uint8_t low_nibble = ((data << 4) & 0xF0) | mode;
    lcd_send_nibble(high_nibble);
    lcd_send_nibble(low_nibble);
}

// Initialize LCD (4-bit mode)
void lcd_init(void) {
    delay_ms(50); // Wait for LCD power-up stability

    // Triple initialization sequence (for compatibility)
    lcd_send_nibble(0x30);
    delay_ms(5);
    lcd_send_nibble(0x30);
    delay_us(150);
    lcd_send_nibble(0x30);
    lcd_send_nibble(0x20); // Switch to 4-bit mode

    // Set display mode
    lcd_write_byte(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE, 0);
    delay_us(50);

    // Turn on display, cursor optional
    lcd_write_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON, 0);
    delay_us(50);

    // Set text direction
    lcd_write_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0);
    delay_us(50);

    lcd_clear();
}

// Clear screen
void lcd_clear(void) {
    lcd_write_byte(LCD_CLEARDISPLAY, 0);
    delay_ms(2); // Clearing screen takes longer
}

// Set cursor position (col=0~15, row=0~1)
void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_write_byte(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
    delay_us(50);
}

// Display string at specified row and column
void lcd_print(const char* text, uint8_t row, uint8_t col) {
    lcd_set_cursor(col, row);
    while (*text) {
        lcd_write_byte(*text++, 1); // 1 = data mode
        delay_us(50);
    }
}
