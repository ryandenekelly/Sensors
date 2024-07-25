#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include "i2c.h"

class MS5637
{
    public:
	    MS5637();
	    MS5637(I2C_HandleTypeDef *i2c);

	    static constexpr std::uint8_t I2C_ADDR = 0x76;
	    static constexpr std::uint8_t READ_ADDR = (I2C_ADDR << 1) | 0x01;
	    static constexpr std::uint8_t WRITE_ADDR = I2C_ADDR << 1;
	    static constexpr std::uint8_t CMD_RESET = 0x1E;
	    static constexpr std::uint8_t CMD_CONVERT_D1 = 0x40;
	    static constexpr std::uint8_t CMD_CONVERT_D2 = 0x50;
	    static constexpr std::uint8_t OSR_256 = 0x00;
	    static constexpr std::uint8_t OSR_512 = 0x02;
	    static constexpr std::uint8_t OSR_1024 = 0x04;
	    static constexpr std::uint8_t OSR_2048 = 0x06;
	    static constexpr std::uint8_t OSR_4096 = 0x08;
	    static constexpr std::uint8_t OSR_8132 = 0x0A;
	    static constexpr std::uint8_t CMD_ADC_READ = 0x00;
	    static constexpr std::uint8_t CMD_PROM_READ = 0xA0;

	    bool reset();
	    bool init();
	    std::array<float, 2> getTempAndPressure(bool compensating = false);

	    std::vector<std::uint16_t> getPROM(std::uint8_t startAddress, std::uint8_t size);
	    std::uint32_t convertAndRead(std::uint8_t dX, std::uint8_t OSR);
	    void setI2C(I2C_HandleTypeDef *i2c);
	    bool getInitialized();
	    void getBaselinePressure(std::uint8_t n);
	    float getTemp();
	    float getPressure();
	    float getAltitude();
	    float getAltitudeFiltered();

    private:
	    I2C_HandleTypeDef *m_i2c;
	    std::uint16_t m_i2cTimeout;
	    bool m_initialized;
	    float m_startingPressure;
	    float m_pressureValue;
	    float m_tempValue;
	    std::uint32_t m_dataRefresh;
	    std::uint32_t m_lastPressureRead;
	    std::uint32_t m_lastTempRead;
	    float m_filtered;

	    bool writeCommand(std::uint8_t command);
	    std::vector<std::uint8_t> readCommand(std::uint8_t command, std::uint8_t size);
	    std::array<std::int64_t, 3> compensate(std::int64_t dt, float temp);
	    bool convertD1(std::uint8_t OSR);
	    bool convertD2(std::uint8_t OSR);
	    std::uint32_t readADC();

};
