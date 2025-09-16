#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>

// ========== 配置 ==========
#define I2C_BUS         "/dev/i2c-1"   // 你的 I2C 总线
#define LCD_ADDR        0x27           // 你的 LCD 地址

// ========== LCD 指令 ==========
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_FUNCTIONSET     0x20
#define LCD_SETDDRAMADDR    0x80

// 入口模式
#define LCD_ENTRYLEFT       0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00

// 显示控制
#define LCD_DISPLAYON       0x04
#define LCD_CURSORON        0x02
#define LCD_BLINKON         0x01

// 功能设置
#define LCD_4BITMODE        0x00
#define LCD_2LINE           0x08
#define LCD_5x8DOTS         0x00

// PCF8574 控制位（背光 + 使能）
#define BL                  0x08   // 背光开
#define EN                  0x04   // 使能位

// ========== 全局变量 ==========
int i2c_fd;

// ========== 函数声明 ==========
void i2c_write_byte(unsigned char data);
void lcd_send_nibble(unsigned char data);
void lcd_write_byte(unsigned char data, unsigned char mode);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(unsigned char col, unsigned char row);
void lcd_print(const char* text, unsigned char row, unsigned char col);

// ========== 主函数 ==========
int main() {
    // 打开 I2C 总线
    if ((i2c_fd = open(I2C_BUS, O_RDWR)) < 0) {
        perror("❌ 无法打开 I2C 总线");
        exit(1);
    }

    // 设置从设备地址
    if (ioctl(i2c_fd, I2C_SLAVE, LCD_ADDR) < 0) {
        perror("❌ 无法设置 I2C 从设备地址");
        close(i2c_fd);
        exit(1);
    }

    printf("🔧 正在初始化 LCD...\n");
    lcd_init();

    printf("✅ LCD 初始化成功！\n");

    // 显示内容
    lcd_clear();
    lcd_print("Hello, Orange Pi!", 0, 0);
    lcd_print("Address: 0x27", 1, 0);

    printf("🎉 文字已显示到 LCD！\n");

    close(i2c_fd);
    return 0;
}

// ========== 函数实现 ==========

// 向 I2C 设备写入一个字节（PCF8574）
void i2c_write_byte(unsigned char data) {
    if (write(i2c_fd, &data, 1) != 1) {
        fprintf(stderr, "⚠️  I2C 写入失败: %s\n", strerror(errno));
    }
    usleep(50); // 短暂延时确保稳定
}

// 发送半字节（高4位）到 LCD，带使能脉冲
void lcd_send_nibble(unsigned char data) {
    data |= BL; // 开背光
    i2c_write_byte(data | EN);   // EN=1
    usleep(1);                   // 保持使能
    i2c_write_byte(data & ~EN);  // EN=0
    usleep(50);
}

// 写入一个字节到 LCD（命令或数据）
void lcd_write_byte(unsigned char data, unsigned char mode) {
    unsigned char high_nibble = (data & 0xF0) | mode;
    unsigned char low_nibble = ((data << 4) & 0xF0) | mode;
    lcd_send_nibble(high_nibble);
    lcd_send_nibble(low_nibble);
}

// 初始化 LCD（4-bit 模式）
void lcd_init(void) {
    usleep(50000); // 等待 LCD 上电稳定

    // 三次初始化序列（兼容性）
    lcd_send_nibble(0x30);
    usleep(5000);
    lcd_send_nibble(0x30);
    usleep(150);
    lcd_send_nibble(0x30);
    lcd_send_nibble(0x20); // 切换到 4-bit 模式

    // 设置显示模式
    lcd_write_byte(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE, 0);
    usleep(50);

    // 开启显示，光标可选
    lcd_write_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON, 0);
    usleep(50);

    // 设置文本方向
    lcd_write_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0);
    usleep(50);

    lcd_clear();
}

// 清屏
void lcd_clear(void) {
    lcd_write_byte(LCD_CLEARDISPLAY, 0);
    usleep(2000); // 清屏需要较长时间
}

// 设置光标位置（col=0~15, row=0~1）
void lcd_set_cursor(unsigned char col, unsigned char row) {
    unsigned char row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_write_byte(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
    usleep(50);
}

// 在指定行列显示字符串
void lcd_print(const char* text, unsigned char row, unsigned char col) {
    lcd_set_cursor(col, row);
    while (*text) {
        lcd_write_byte(*text++, 1); // 1=数据模式
        usleep(50);
    }
}
