#include "MS5637.hpp"
#include <cmath>

MS5637::MS5637()
{
    m_i2c = nullptr;
    m_i2cTimeout = 100;
    m_initialized = false;
    m_baselinePressure = 1013.25;
    m_pressureValue = NAN;
    m_tempValue = NAN;
    m_dataRefresh = 100;
    m_lastPressureRead = 0;
    m_lastTempRead = 0;
    m_baselineAltitude = 0.0;
}

MS5637::MS5637(I2C_HandleTypeDef *i2c)
{
    m_i2c = i2c;
    m_i2cTimeout = 100;
    m_initialized = false;
    m_baselinePressure = 1013.25;
    m_pressureValue = NAN;
    m_tempValue = NAN;
    m_dataRefresh = 100;
    m_lastPressureRead = 0;
    m_lastTempRead = 0;
    m_baselineAltitude = 0.0;
}

bool MS5637::getInitialized()
{
    return m_initialized;
}

bool MS5637::writeCommand(std::uint8_t command)
{
    std::uint8_t data[] = {command};
    HAL_StatusTypeDef i2c_res = HAL_I2C_Master_Transmit(m_i2c, MS5637::WRITE_ADDR, data, 1, m_i2cTimeout);
    return i2c_res == HAL_OK;
}

std::vector<std::uint8_t> MS5637::readCommand(std::uint8_t command, std::uint8_t size)
{
    std::uint8_t readAddress[] = {command};
    std::uint8_t readData[size] = {0};
    HAL_StatusTypeDef tx_res = HAL_I2C_Master_Transmit(m_i2c, MS5637::WRITE_ADDR, readAddress, 1, m_i2cTimeout);
    HAL_StatusTypeDef rx_res = HAL_I2C_Master_Receive(m_i2c, MS5637::READ_ADDR, readData, size, m_i2cTimeout);

    return std::vector<std::uint8_t>(readData, readData + sizeof readData / sizeof readData[0]);
    UNUSED(tx_res);
    UNUSED(rx_res);
}

bool MS5637::reset()
{
    return writeCommand((std::uint8_t)MS5637::CMD_RESET);
}

bool MS5637::init()
{
    if(m_i2c)
    {
	HAL_Delay(200);
	m_initialized = true;
	return reset();

    }
    return false;

}

void MS5637::setI2C(I2C_HandleTypeDef *i2c)
{
    m_i2c = i2c;
}

std::vector<std::uint16_t> MS5637::getPROM(std::uint8_t startAddress, std::uint8_t size)
{
    std::vector<std::uint16_t> prom;

    /* Read PROM */
    for(int i=0; i<size; i++)
    {
	    std::vector<std::uint8_t> data = readCommand(startAddress, 2);

	    startAddress = startAddress + 2;
	    prom.push_back((data[0] << 8) | data[1]);
    }

    return prom;
}

bool MS5637::convertD1(std::uint8_t OSR)
{
    return writeCommand(MS5637::CMD_CONVERT_D1 | OSR);
}

bool MS5637::convertD2(std::uint8_t OSR)
{
    return writeCommand(MS5637::CMD_CONVERT_D2 | OSR);
}

std::uint32_t MS5637::readADC()
{
    std::vector<std::uint8_t> data = readCommand(MS5637::CMD_ADC_READ, 3);
    std::uint32_t adcVal = (data[0] << 16) | (data[1] << 8) | (data[2] & 0xF);

    return adcVal;
}

std::uint32_t MS5637::convertAndRead(std::uint8_t dX, std::uint8_t OSR)
{
    writeCommand(dX | OSR);
    HAL_Delay(20);
    return readADC();
}

/* TODO: make this work!! */
std::array<std::int64_t, 3> MS5637::compensate(std::int64_t dt, float temp)
{
    // compensation
    std::int64_t t2 = 0;
    std::int64_t off2 = 0;
    std::int64_t sens2 = 0;
    if(temp < 2000.0)
    {
	t2 = (3 * ((std::int64_t)dt * (std::int64_t)dt) >> 33);
	off2 = 61 * (temp - 2000 * temp - 2000) / 16;
	sens2 = 29 * (temp - 2000) * (temp - 2000) / 16;

	if(temp < -1500.0)
	{
	    off2 += 17 * ((temp + 1500) * (temp + 1500));
	    sens2 += 9 * ((temp + 1500) * (temp + 1500));
	}
    }
    else
    {
	t2 = (5 * ((std::int64_t)dt * (std::int64_t)dt)) >> 38;
	off2 = 0;
	sens2 = 0;
    }

    return {t2, off2, sens2};
}

std::array<float, 2> MS5637::getTempAndPressure(bool compensating)
{
    // TODO: only need to read this once!
    std::vector<std::uint16_t> C = getPROM(0xA0, 7);


    std::uint32_t d1 = convertAndRead(MS5637::CMD_CONVERT_D1, MS5637::OSR_8132);
    std::uint32_t d2 = convertAndRead(MS5637::CMD_CONVERT_D2, MS5637::OSR_8132);

    std::int32_t dt = (d2) - ((std::uint32_t)C[5] << 8);
    float temp = 2000.0 + ((dt * C[5]) * std::exp2(-23));

    // compensation
    std::int64_t t2 = 0;
    std::int64_t off2 = 0;
    std::int64_t sens2 = 0;
    if(compensating)
    {
	// NB: this doesn't work!
	std::array<std::int64_t, 3> correction = compensate(dt, temp);
	t2 = correction[0];
	off2 = correction[1];
	sens2 = correction[2];
    }


    temp = temp - t2;

    std::int64_t off = (C[2] * std::exp2(17)) + ((C[4] * dt) * std::exp2(-6));
    off -= off2;

    std::int64_t sens = (C[1] * std::exp2(16)) + ((C[3]*dt) * std::exp2(-7));
    sens -= sens2;

    float p = ((d1) * (sens >> 21) - (off)) * std::exp2(-15);



    float pressureActual = p / 100;
    float tempActual = temp / 100;


    return {tempActual, pressureActual};
}

void MS5637::getBaselinePressure(std::uint16_t n)
{
    float baseline = 0.0;
    for(std::uint32_t i=0; i<n; i++)
    {
	baseline += getTempAndPressure()[1];
    }
    baseline /= (float)n;
    m_baselinePressure = baseline;
}

void MS5637::getBaselineAltitude(std::uint32_t n)
{
    float baseline = 0.0;
    for(std::uint32_t i=0; i<n; i++)
    {
	baseline += getAltitude();
    }
    baseline /= (float)n;
    m_baselineAltitude = baseline;
}

float MS5637::getAltitude()
{
    float calculatedAlt = 44330.0 * (1.0 - pow(getPressure()/m_baselinePressure, 1.0/5.255));
    return calculatedAlt - m_baselineAltitude;
}

float MS5637::getAltitude(float pressure)
{
    float calculatedAlt = 44330.0 * (1.0 - pow(pressure/m_baselinePressure, 1.0/5.255));
    return calculatedAlt - m_baselineAltitude;
}

float MS5637::getPressure()
{
    if((HAL_GetTick() - m_lastPressureRead) > m_dataRefresh)
    {
	m_pressureValue = getTempAndPressure()[1];
    }
    return m_pressureValue;
}

float MS5637::getTemp()
{
    if((HAL_GetTick() - m_lastTempRead) > m_dataRefresh)
    {
	m_tempValue = getTempAndPressure()[0];
    }
    return m_tempValue;
}

