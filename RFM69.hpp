/* derived from https://github.com/LowPowerLab/RFM69 */

#pragma once

#include "main.h"

#include <cstdint>
#include <vector>
#include <array>

#ifdef SPI
#include "spi.h"
#endif

#include "RFM.hpp"
class RFM69
{
    public:

	RFM69();
	RFM69(SPI_HandleTypeDef * spiHandler, GPIO_TypeDef * csPort,
	      std::uint16_t csPin, GPIO_TypeDef * resetPort,
	      std::uint16_t resetPin, std::uint8_t networkID);

	void reset();
	bool init();
	bool readReg(std::uint8_t address, std::uint8_t* readData);
	bool writeReg(std::uint8_t address, std::uint8_t writeData);
	bool writeRegBurst(std::uint8_t addressStart, std::vector<std::uint8_t> writeData,
			   std::uint8_t burstLength);
	bool writeRegN(std::uint8_t startAddress, std::vector<std::uint8_t> data, std::uint8_t length);
	bool verifyWrite(std::uint8_t address, std::uint8_t compVal);

	bool changeMode(RFM::Mode mode);
	RFM::Mode getMode();
	bool sendPacket();
	bool recievePacket();
	bool recievePacketTimeout();\
	bool recieve();
	bool available();

	void readFIFO();
	std::vector<std::uint8_t> getFIFO();
	bool init2();
	bool setSyncWords(std::vector<std::uint8_t> syncWords, std::uint8_t length);
	bool setMod();
	bool setPreambleLength(std::uint8_t bytes);
	bool setFrequency(float freq);
	bool setTxPower(std::int8_t power, bool isHighPower);
	void encrypt(std::vector<std::uint8_t> key);
	bool send(std::uint8_t* data, std::uint8_t length);



    private:
	SPI_HandleTypeDef *m_spiHandler;
	GPIO_TypeDef * m_csPort;
	std::uint16_t m_csPin;
	GPIO_TypeDef * m_resetPort;
	std::uint16_t m_resetPin;
	std::uint8_t m_networkID;
	std::vector<std::uint8_t> m_fifo;

	const std::uint32_t m_spiTimeout = 100;
};

