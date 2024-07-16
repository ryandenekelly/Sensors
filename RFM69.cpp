#include "RFM69.hpp"
#include <string.h>


RFM69::RFM69(SPI_HandleTypeDef * spiHandler, GPIO_TypeDef * csPort,
	     std::uint16_t csPin, GPIO_TypeDef * resetPort,
	     std::uint16_t resetPin, std::uint8_t networkID)
{
    m_spiHandler = spiHandler;
    m_csPort = csPort;
    m_csPin = csPin;
    m_resetPort = resetPort;
    m_resetPin = resetPin;
    m_networkID = networkID;
    //packetSent=0;

}

void RFM69::reset()
{
    HAL_GPIO_WritePin(m_resetPort, m_resetPin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(m_resetPort, m_resetPin, GPIO_PIN_RESET);
    HAL_Delay(5);
}

bool RFM69::readReg(std::uint8_t address, std::uint8_t* readData)
{
    std::uint8_t addressBuffer[] = {address};
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status_tx = HAL_SPI_Transmit(m_spiHandler, addressBuffer, 1, m_spiTimeout);
    HAL_StatusTypeDef hal_status_rx = HAL_SPI_Receive(m_spiHandler, readData, 1, m_spiTimeout);
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);


    return (hal_status_tx == HAL_OK) && (hal_status_rx == HAL_OK);
}

bool RFM69::writeReg(std::uint8_t address, std::uint8_t writeData)
{
    std::uint8_t txBuffer[] = {std::uint8_t(address | 0x80), writeData};
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(m_spiHandler, txBuffer, 2, m_spiTimeout);
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);


    return hal_status == HAL_OK;
}

bool RFM69::writeRegBurst(std::uint8_t addressStart, std::vector<std::uint8_t> writeData,
			  std::uint8_t burstLength)
{
    writeData.insert(writeData.begin(), addressStart);
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(m_spiHandler, writeData.data(), burstLength+1, 500);
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);

    return hal_status == HAL_OK;
}

/* NB: not burst, just repeated single access (i.e. slow!)*/
bool RFM69::writeRegN(std::uint8_t startAddress, std::vector<std::uint8_t> data, std::uint8_t length)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    for(std::uint8_t i=0; i<length; i++)
    {
	std::uint8_t txData[] = {std::uint8_t((startAddress + i) | 0x80), data[i]};
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
	hal_status = HAL_SPI_Transmit(m_spiHandler, txData, 2, m_spiTimeout);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
	if(hal_status != HAL_OK)
	{
	    return false;
	}
    }

    return true;
}

bool RFM69::verifyWrite(std::uint8_t address, std::uint8_t compVal)
{
    std::uint8_t readData[] = {0};
    readReg(address, readData);
    if(((compVal & readData[0]) == compVal) || compVal == readData[0])
    {
	return true;
    }
    else
    {
	return false;
    }
}

bool RFM69::init2()
{
    writeReg(RFM::RegFifoThresh , RFM::TxStartConditionFifoNotEmpty | 0x0F);
    // Default RSSI Thresh
    // Default Sync Config
    // Default Payload Length ()
    // Default Packet Config2
    writeReg(RFM::RegTestDagc, RFM::ContinuousDagcImproved); // NB: lowbeta off

    writeReg(RFM::RegTestPa1, RFM::TestPa1Normal);
    writeReg(RFM::RegTestPa1, RFM::TestPa2Normal);

    std::vector<std::uint8_t> syncWords = {0x2D, 0x4D};
    setSyncWords(syncWords, syncWords.size());

    setMod();

    setPreambleLength(4u);

    // setFrequency(915.0); // default is already 915MHz

    encrypt(std::vector<std::uint8_t>());

    setTxPower(13, true);

    return true;
}

bool RFM69::setSyncWords(std::vector<std::uint8_t> syncWords, std::uint8_t length)
{
    std::uint8_t syncConfigVal[] = {0};
    readReg(RFM::RegSyncConfig, syncConfigVal);

    if(syncWords.size() <= length)
    {
	writeRegN(RFM::RegSyncValue1, syncWords, length);
	syncConfigVal[0] |= RFM::SyncOn;
    }
    else
    {
	syncConfigVal[0] &= ~RFM::SyncOn;
    }

    syncConfigVal[0] &= ~RFM::SyncSize;
    syncConfigVal[0] |= (length-1) << 3;

    writeReg(RFM::RegSyncConfig, syncConfigVal[0]);

    return true;
}
bool RFM69::setMod()
{
    std::uint8_t reg_02 = 0x00 | 0x00 | 0x01; // Packet, FSK, BT1
    std::uint8_t reg_03 = 0x00; //br MSB
    std::uint8_t reg_04 = 0x80; //br LSB
    std::uint8_t reg_05 = 0x10; // fdev msb
    std::uint8_t reg_06 = 0x00; // fdev lsb
    std::uint8_t reg_19 = 0xE0; // dccfreq
    std::uint8_t reg_1A = 0xE0; // dccfreqafc
    std::uint8_t reg_37 = 0x80 | 0x40 | 0x10 | 0x00; // Variable packet, DCFREE white, crc on, address filtering none

    writeReg(RFM::RegDataModul, reg_02);
    writeReg(RFM::RegBitRateMSB, reg_03);
    writeReg(RFM::RegBitRateLSB, reg_04);
    writeReg(RFM::RegFdevMSB, reg_05);
    writeReg(RFM::RegFdevLSB, reg_06);
    writeReg(RFM::RegRxBw, reg_19);
    writeReg(RFM::RegAfcBw, reg_1A);
    writeReg(RFM::RegPacketConfig1, reg_37);

    return true;
}

bool RFM69::setPreambleLength(std::uint8_t bytes)
{
    writeReg(RFM::RegPreambleMsb, bytes >> 8);
    writeReg(RFM::RegPreambleLsb, bytes & 0xFF);

    return true;
}

#define RH_RF69_FXOSC 32000000.0
#define RFM69_FSTEP  (RH_RF69_FXOSC / 524288)

bool RFM69::setFrequency(float freq)
{
    std::uint32_t frf = (std::uint8_t)((freq * 1000000.0) / RFM69_FSTEP);

    writeReg(RFM::RegFrfMSB, (frf >> 16) & 0xFF);
    writeReg(RFM::RegFrfMid, (frf >> 8) & 0xFF);
    writeReg(RFM::RegFrfLSB, frf & 0xFF);

    return true;
}

bool RFM69::send(std::uint8_t* data, std::uint8_t length)
{
    while(!changeMode(RFM::Mode::Standby)){;}

    writeReg(RFM::RegFifo, length + 4); // Send message length (payload + headers)
    writeReg(RFM::RegFifo, 0xFF); // header to
    writeReg(RFM::RegFifo, 0xFF); // header from
    writeReg(RFM::RegFifo, 0x0);  // header id
    writeReg(RFM::RegFifo, 0x0);  // header flags

    while(length--)
    {
	writeReg(RFM::RegFifo, *data++);
    }

    changeMode(RFM::Mode::Tx);

}

bool RFM69::init()
{
    bool writeSuccess = true;
    bool verified = true;

    // OpMode (0x01)

    std::uint8_t opModeBitField = RFM::SequencerOn | RFM::ListenOff | (std::uint8_t)RFM::Mode::Standby;
    writeSuccess &= writeReg(RFM::RegOpMode, opModeBitField);
    verified &= verifyWrite(RFM::RegOpMode, opModeBitField);


    // DataModul (0x02)
    std::uint8_t dataModulBitField = RFM::DataModePacket | RFM::ModFSK | RFM::ModNoShaping;
    writeSuccess &= writeReg(RFM::RegDataModul, dataModulBitField);
    verified &= verifyWrite(RFM::RegDataModul, dataModulBitField);

    // BitRate LSB (0x03)
    writeSuccess &= writeReg(RFM::RegBitRateMSB, 0x02);
    verified &= verifyWrite(RFM::RegBitRateMSB, 0x02);


    // BitRate MSB (0x04)
    writeSuccess &= writeReg(RFM::RegBitRateLSB, 0x40);
    verified &= verifyWrite(RFM::RegBitRateLSB, 0x40);


    // FdevMSb (0x05)
    writeSuccess &= writeReg(RFM::RegFdevMSB, 0x00);
    verified &= verifyWrite(RFM::RegFdevMSB, 0x00);


    // FdevLSB (0x06
    writeSuccess &= writeReg(RFM::RegFdevLSB, 0x52);
    verified &= verifyWrite(RFM::RegFdevLSB, 0x52);


    // FrfMSB (0x07) - 915MHz
    writeSuccess &= writeReg(RFM::RegFrfMSB, 0xE4);
    verified &= verifyWrite(RFM::RegFrfMSB, 0xE4);

    // FrfMid (0x08)
    writeSuccess &= writeReg(RFM::RegFrfMid, 0xC0);
    verified &= verifyWrite(RFM::RegFrfMid, 0xC0);

    // FrfLSB (0x09)
    writeSuccess &= writeReg(RFM::RegFrfLSB, 0x00);
    verified &= verifyWrite(RFM::RegFrfLSB, 0x00);


    // RxBw (0x19)
    std::uint8_t rxBwBitField = 0x02 | RFM::RxBwMant16 | 0x06;
    writeSuccess &= writeReg(RFM::RegRxBw, rxBwBitField);
    verified &= verifyWrite(RFM::RegRxBw, rxBwBitField);

    // RegDioMapping1 (0x25)
    writeSuccess &= writeReg(RFM::RegDioMapping1, 00);
    verified &= verifyWrite(RFM::RegDioMapping1, 00);

    // RegDioMapping1 (0x26)
    writeSuccess &= writeReg(RFM::RegDioMapping2, RFM::ClkOutOff);
    verified &= verifyWrite(RFM::RegDioMapping2, RFM::ClkOutOff);

    // IrqFlags2 (0x28)
    writeSuccess &= writeReg(RFM::RegIrqFlags2, RFM::FifoOverrun);

    // RssiThresh (0x29)
    writeSuccess &= writeReg(RFM::RegRssiThresh, 0x8F); // or 220? // TODO: Check
    verified &= verifyWrite(RFM::RegRssiThresh, 0x8F);

    // SyncConfig (0x2E)
    std::uint8_t syncConfigBitField = 0x80 | 0x00 | 0x08 | 0x00;
    writeSuccess &= writeReg(RFM::RegSyncConfig, syncConfigBitField);
    verified &= verifyWrite(RFM::RegSyncConfig, syncConfigBitField);

    // RegSyncValue1 (0x2F)
    writeSuccess &= writeReg(RFM::RegSyncValue1, 0x2D); // TODO: Check
    verified &= verifyWrite(RFM::RegSyncValue1, 0x2D);

    // RegSyncValue2 (0x30) - network id
    writeSuccess &= writeReg(RFM::RegSyncValue2, m_networkID);
    verified &= verifyWrite(RFM::RegSyncValue2, m_networkID);

    // PacketConfig1 (0x37)
    std::uint8_t packetConfig1BitField = RFM::PacketFormat | RFM::DcFreeNone | RFM::CrcOn;
    writeSuccess &= writeReg(RFM::RegPacketConfig1, packetConfig1BitField);
    verified &= verifyWrite(RFM::RegPacketConfig1, packetConfig1BitField);

    // FifoThresh (0x3C)
    std::uint8_t fifoThreshBitField = 0x02;//RFM::TxStartConditionFifoNotEmpty | RFM::FifoThresholdValue;
    writeSuccess &= writeReg(RFM::RegFifoThresh, fifoThreshBitField);
    verified &= verifyWrite(RFM::RegFifoThresh, fifoThreshBitField);

    // PacketConfig2 (0x3D)
    std::uint8_t packetConfig2BitField = RFM::InterPacketRxDelay2Bits | RFM::RestartRxOff | RFM::AutoRxRestartOff | RFM::AesOff;
    writeSuccess &= writeReg(RFM::RegPacketConfig2, packetConfig2BitField);
    verified &= verifyWrite(RFM::RegPacketConfig2, packetConfig2BitField & ~RFM::RestartRxOff);


    // TestDagc (0x6f)
    writeSuccess &= writeReg(RFM::RegTestDagc, RFM::ContinuousDagcImproved);
    verified &= verifyWrite(RFM::RegTestDagc, RFM::ContinuousDagcImproved);

    return true && (writeSuccess && verified);
}

bool RFM69::sendPacket()
{
    // dio 00
    writeReg(RFM::RegDioMapping1, 0x00);
    bool dioM1 = verifyWrite(RFM::RegDioMapping1, 0x00);
    writeReg(RFM::RegDioMapping2, RFM::ClkOutOff);
    bool dioM2 = verifyWrite(RFM::RegDioMapping2, RFM::ClkOutOff);
    if(dioM1 == false || dioM2 == false){ return false;}


    // set mode to standby (to turn rx off)
    if(!changeMode(RFM::Mode::Standby)){ return false;}

    std::uint8_t irqFlags1[] = {0};
    while((irqFlags1[0] & RFM::ModeReady) == 0x00)
    {
	readReg(RFM::RegIrqFlags1, irqFlags1);
    }

    // write to fifo
    writeReg(0x00, 0x48);
    writeReg(0x00, 0x49);
    writeReg(0x00, 0x21);


    // set to tx mode
    if(!changeMode(RFM::Mode::Tx)){ return false;}

    std::uint8_t irqFlags2[] = {0};
    while((irqFlags2[0] & RFM::PacketSent) == 0x00)
    {
	readReg(RFM::RegIrqFlags2, irqFlags2);
    }

    // set mode to standby (to turn rx off)
    if(!changeMode(RFM::Mode::Standby)){ return false;}

    return true;
}
bool RFM69::changeMode(RFM::Mode mode)
{
    // get current value to avoid overwriting other settings.
    std::uint8_t opModeRegVal[] = {0};
    readReg(RFM::RegOpMode, opModeRegVal);
    switch(mode)
    {
	case RFM::Mode::Sleep:
	    writeReg(RFM::RegOpMode, opModeRegVal[0] | (std::uint8_t)mode);
	    break;
	case RFM::Mode::Standby:
	    writeReg(RFM::RegOpMode, opModeRegVal[0] | (std::uint8_t)mode);
	    break;
	case RFM::Mode::Freq:
	    writeReg(RFM::RegOpMode, opModeRegVal[0] | (std::uint8_t)mode);
	    break;
	case RFM::Mode::Tx:
	    writeReg(RFM::RegOpMode, opModeRegVal[0] | (std::uint8_t)mode);
	    break;
	case RFM::Mode::Rx:
	    writeReg(RFM::RegOpMode, opModeRegVal[0] | (std::uint8_t)mode);
	    break;
	default:
	    return false;
    }
    // verify that we did enter the intended mode.
    return verifyWrite(RFM::RegOpMode, opModeRegVal[0] | (std::uint8_t)mode);
}

RFM::Mode RFM69::getMode()
{
    std::uint8_t opModeRegVal[] = {0};
    readReg(RFM::RegOpMode, opModeRegVal);
    return (RFM::Mode)opModeRegVal[0];
}

bool RFM69::recievePacket()
{
    changeMode(RFM::Mode::Rx);

    std::uint8_t irqFlags2[] = {0};
    while((irqFlags2[0] & RFM::PayloadReady) == 0x00)
    {
	readReg(RFM::RegIrqFlags2, irqFlags2);
    }
    return false;
}

bool RFM69::recievePacketTimeout()
{
    changeMode(RFM::Mode::Rx);

    std::uint8_t irqFlags2[] = {0};
    std::uint32_t time_t = 0;
    const std::uint32_t TIMEOUT = 80;
    while(time_t < TIMEOUT)
    {
	time_t = HAL_GetTick();
	readReg(RFM::RegIrqFlags2, irqFlags2);
	if((irqFlags2[0] & RFM::PayloadReady) == 0x00)
	{
	    return true;
	}
    }
    return false;
}


void RFM69::readFIFO()
{
    m_fifo.clear();
    std::uint8_t fifo[] = {0};
    while((fifo && RFM::FifoNotEmpty) == 0x00)
    {
	readReg(RFM::RegFifo, fifo);
	m_fifo.push_back(fifo[0]);
    }

}

std::vector<uint8_t> RFM69::getFIFO()
{
    return m_fifo;
}

bool RFM69::setTxPower(std::int8_t power, bool isHighPower)
{
    std::int8_t power_temp = power;
    std::uint8_t paLevel;
    if(isHighPower)
    {
	if(power_temp < -2)
	{
	    power_temp = -2;
	}
	if( power_temp <= 13)
	{
	    paLevel = RFM::Pa1On | ((power_temp + 18) & RFM::OutputPower);
	}
	else if( power_temp >= 18)
	{
	    paLevel = RFM::Pa1On | RFM::Pa2On | ((power_temp + 11) & RFM::OutputPower);
	}
	else
	{
	    paLevel = RFM::Pa1On | RFM::Pa2On | ((power_temp + 14) & RFM::OutputPower);
	}

    }
    else
    {
	return false;
    }

    writeReg(RFM::RegPaLevel, paLevel);
    return true;
}


void RFM69::encrypt(std::vector<std::uint8_t> key)
{
    changeMode(RFM::Mode::Standby);
    if(key.size() > 0)
    {
	writeRegN(RFM::RegAesKey1, key, key.size());

	std::uint8_t packetConfig2Value[] = {0};
	readReg(RFM::RegPacketConfig2, packetConfig2Value);
	writeReg(RFM::RegPacketConfig2, (packetConfig2Value[0] | RFM::AesOn));
    }
    else
    {
	std::uint8_t packetConfig2Value[] = {0};
	readReg(RFM::RegPacketConfig2, packetConfig2Value);
	writeReg(RFM::RegPacketConfig2, (packetConfig2Value[0] & ~RFM::AesOn));
    }

}

bool RFM69::available()
{
    // get current mode
    RFM::Mode mode = getMode();
    if(mode == RFM::Mode::Tx)
    {
	// if we are transmitting then we can't receive!
	return false;
    }
    changeMode(RFM::Mode::Rx);
    return true;

}
bool RFM69::recieve()
{
    changeMode(RFM::Mode::Rx);

    std::uint8_t irqFlags2[] = {0};
    if(irqFlags2[0] & RFM::PayloadReady)
    {
	readReg(RFM::RegIrqFlags2, irqFlags2);
    }
    return false;
}

