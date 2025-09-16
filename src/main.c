#include <stm32f10x.h>

void delay_us(volatile uint32_t us) {
    volatile uint32_t i;
    for (i = 0; i < us * 10; i++);
}

void delay_ms(volatile uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

int main(void) {
    // ========== 1. 开启 GPIOC 时钟 ==========
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(0xF << (13 - 8) * 4);
    GPIOC->CRH |= (0b0010 << (13 - 8) * 4);

    // ========== 2. 开启 GPIOB 时钟 ==========
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRL &= ~(0xFF << (6*4));
    GPIOB->CRL |= (GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1);
    GPIOB->CRL |= (GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1);

    // ========== 3. 初始化 I2C 总线状态 ==========
    GPIOB->BSRR = GPIO_BSRR_BS6 | GPIO_BSRR_BS7;
    delay_us(100);

    // ========== 4. 发送 I2C 起始条件 ==========
    GPIOB->BSRR = GPIO_BSRR_BS7;   // SDA=1
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BS6;   // SCL=1
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BR7;   // SDA=0
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BR6;   // SCL=0
    delay_us(100);

    // ========== 5. 发送设备地址（写模式） ==========
    uint8_t addr = 0x27;  // 👈 尝试 0x27, 0x3F, 0x20, 0x23
    uint8_t addr_byte = (addr << 1) | 0; // 写模式
    for (int i = 0; i < 8; i++) {
        if (addr_byte & 0x80) {
            GPIOB->BSRR = GPIO_BSRR_BS7;
        } else {
            GPIOB->BSRR = GPIO_BSRR_BR7;
        }
        addr_byte <<= 1;
        delay_us(100);
        GPIOB->BSRR = GPIO_BSRR_BS6;
        delay_us(100);
        GPIOB->BSRR = GPIO_BSRR_BR6;
        delay_us(100);
    }

    // 忽略 ACK
    GPIOB->BSRR = GPIO_BSRR_BS7;
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BS6;
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BR6;
    delay_us(100);

    // ========== 6. 发送数据字节 0x00（关闭背光） ==========
    uint8_t data = 0x00;
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) {
            GPIOB->BSRR = GPIO_BSRR_BS7;
        } else {
            GPIOB->BSRR = GPIO_BSRR_BR7;
        }
        data <<= 1;
        delay_us(100);
        GPIOB->BSRR = GPIO_BSRR_BS6;
        delay_us(100);
        GPIOB->BSRR = GPIO_BSRR_BR6;
        delay_us(100);
    }

    // 忽略 ACK
    GPIOB->BSRR = GPIO_BSRR_BS7;
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BS6;
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BR6;
    delay_us(100);

    // ========== 7. 发送 STOP 条件 ==========
    GPIOB->BSRR = GPIO_BSRR_BR7;
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BS6;
    delay_us(100);
    GPIOB->BSRR = GPIO_BSRR_BS7;
    delay_us(100);

    // ========== 8. 主循环 ==========
    while (1) {
        GPIOC->BSRR = GPIO_BSRR_BR13; // LED 亮
        delay_ms(500);
        GPIOC->BSRR = GPIO_BSRR_BS13; // LED 灭
        delay_ms(500);
    }
}
