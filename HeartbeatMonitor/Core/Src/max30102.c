#include "max30102.h"

static I2C_HandleTypeDef *msg_i2c;

static void MAX30102_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    HAL_I2C_Master_Transmit(msg_i2c, MAX30102_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

static void MAX30102_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_I2C_Master_Transmit(msg_i2c, MAX30102_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(msg_i2c, MAX30102_I2C_ADDR, data, len, HAL_MAX_DELAY);
}

void MAX30102_Init(I2C_HandleTypeDef *hi2c) {
    msg_i2c = hi2c;

    // Reset
    MAX30102_WriteReg(REG_MODE_CONFIG, 0x40);
    HAL_Delay(100);

    // Interrupt enable (FIFO almost full)
    MAX30102_WriteReg(REG_INTR_ENABLE_1, 0xC0); // A_FULL_EN, PPG_RDY_EN
    MAX30102_WriteReg(REG_INTR_ENABLE_2, 0x00);

    // FIFO Config
    // SMP_AVE = 010 (4 samples avg), FIRO_ROLLOVER_EN = 1, FIFO_A_FULL = 15
    MAX30102_WriteReg(REG_FIFO_CONFIG, 0x4F);

    // Mode Config (SpO2 mode)
    MAX30102_WriteReg(REG_MODE_CONFIG, 0x03);

    // SpO2 Config
    // ADC Range = 4096nA, Sample Rate = 100Hz, LED Pulse Width = 411us
    MAX30102_WriteReg(REG_SPO2_CONFIG, 0x27);

    // LED Pulse Amplitude
    MAX30102_WriteReg(REG_LED1_PA, 0x24); // Red ~7mA
    MAX30102_WriteReg(REG_LED2_PA, 0x24); // IR ~7mA
    MAX30102_WriteReg(REG_PILOT_PA, 0x7F);
}

void MAX30102_ReadFIFO(uint32_t *red_led, uint32_t *ir_led) {
    uint8_t data[6];
    
    // Check write/read pointers to see if we have data?
    // For simplicity, we just read one sample if called.
    // In a real app, you might check interrupt or pointers.
    
    MAX30102_ReadRegs(REG_FIFO_DATA, data, 6);

    // Combine bytes
    // Data is 3 bytes for Red, 3 bytes for IR
    /*
      FIFO Data Format:
      [0] Red[17:16]
      [1] Red[15:8]
      [2] Red[7:0]
      [3] IR[17:16]
      [4] IR[15:8]
      [5] IR[7:0]
    */

    *red_led = ((data[0] & 0x03) << 16) | (data[1] << 8) | data[2];
    *ir_led  = ((data[3] & 0x03) << 16) | (data[4] << 8) | data[5];
}

float MAX30102_ReadTemperature() {
    uint8_t temp_int, temp_frac;
    MAX30102_WriteReg(REG_TEMP_CONFIG, 0x01); // Enable temp
    
    // Wait for temp ready? Polling for now.
    HAL_Delay(10); 
    
    MAX30102_ReadRegs(REG_TEMP_INTR, &temp_int, 1);
    MAX30102_ReadRegs(REG_TEMP_FRAC, &temp_frac, 1);
    
    return (float)temp_int + ((float)temp_frac * 0.0625f);
}
