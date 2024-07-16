#pragma once
class RFM
{
    public:
	/* config registers */
	static constexpr std::uint8_t RegFifo       = 0x00;
	static constexpr std::uint8_t RegOpMode     = 0x01;
	static constexpr std::uint8_t RegDataModul  = 0x02;
	static constexpr std::uint8_t RegBitRateMSB = 0x03;
	static constexpr std::uint8_t RegBitRateLSB = 0x04;
	static constexpr std::uint8_t RegFdevMSB    = 0x05;
	static constexpr std::uint8_t RegFdevLSB    = 0x06;
	static constexpr std::uint8_t RegFrfMSB     = 0x07;
	static constexpr std::uint8_t RegFrfMid     = 0x08;
	static constexpr std::uint8_t RegFrfLSB     = 0x09;
	static constexpr std::uint8_t RegOsc1       = 0x0A;
	static constexpr std::uint8_t RegAfcCtrl    = 0x0B;
	static constexpr std::uint8_t Reserved0C    = 0x0C;
	static constexpr std::uint8_t RegListen1    = 0x0D;
	static constexpr std::uint8_t RegListen2    = 0x0E;
	static constexpr std::uint8_t RegListen3    = 0x0F;
	static constexpr std::uint8_t RegVersion    = 0x10;

	/* Transmitter Registers */
	static constexpr std::uint8_t RegPaLevel = 0x11;
	static constexpr std::uint8_t RegPaRamp  = 0x12;
	static constexpr std::uint8_t RegOcp     = 0x13;

	/* Receiver Registers */
	static constexpr std::uint8_t Reserved14    = 0x14;
	static constexpr std::uint8_t Reserved15    = 0x15;
	static constexpr std::uint8_t Reserved16    = 0x16;
	static constexpr std::uint8_t Reserved17    = 0x17;
	static constexpr std::uint8_t RegLna        = 0x18;
	static constexpr std::uint8_t RegRxBw       = 0x19;
	static constexpr std::uint8_t RegAfcBw      = 0x1A;
	static constexpr std::uint8_t RegOokPeak    = 0x1B;
	static constexpr std::uint8_t RegOokAvg     = 0x1C;
	static constexpr std::uint8_t RegOokFix     = 0x1D;
	static constexpr std::uint8_t RegAfcFei     = 0x1E;
	static constexpr std::uint8_t RegAfcMsb     = 0x1F;
	static constexpr std::uint8_t RegAfcLsb     = 0x20;
	static constexpr std::uint8_t RegFeiMsb     = 0x21;
	static constexpr std::uint8_t RegFeiLsb     = 0x22;
	static constexpr std::uint8_t RegRssiConfig = 0x23;
	static constexpr std::uint8_t RegRssiValue  = 0x24;

	/* IRQ and Pin Mapping */
	static constexpr std::uint8_t RegDioMapping1 = 0x25;
	static constexpr std::uint8_t RegDioMapping2 = 0x26;
	static constexpr std::uint8_t RegIrqFlags1   = 0x27;
	static constexpr std::uint8_t RegIrqFlags2   = 0x28;
	static constexpr std::uint8_t RegRssiThresh  = 0x29;
	static constexpr std::uint8_t RegRxTimeout1  = 0x2A;
	static constexpr std::uint8_t RegRxTimeout2  = 0x2B;

	/* Packet Engine Registers */
	static constexpr std::uint8_t RegPreambleMsb   = 0x2C;
	static constexpr std::uint8_t RegPreambleLsb   = 0x2D;
	static constexpr std::uint8_t RegSyncConfig    = 0x2E;
	static constexpr std::uint8_t RegSyncValue1    = 0x2F;
	static constexpr std::uint8_t RegSyncValue2    = 0x30;
	static constexpr std::uint8_t RegSyncValue3    = 0x31;
	static constexpr std::uint8_t RegSyncValue4    = 0x32;
	static constexpr std::uint8_t RegSyncValue5    = 0x33;
	static constexpr std::uint8_t RegSyncValue6    = 0x34;
	static constexpr std::uint8_t RegSyncValue7    = 0x35;
	static constexpr std::uint8_t RegSyncValue8    = 0x36;
	static constexpr std::uint8_t RegPacketConfig1 = 0x37;
	static constexpr std::uint8_t RegPayloadLength = 0x38;
	static constexpr std::uint8_t RegNodeAdrs      = 0x39;
	static constexpr std::uint8_t RegBroadcastAdrs = 0x3A;
	static constexpr std::uint8_t RegAutoModes     = 0x3B;
	static constexpr std::uint8_t RegFifoThresh    = 0x3C;
	static constexpr std::uint8_t RegPacketConfig2 = 0x3D;
	static constexpr std::uint8_t RegAesKey1       = 0x3E;
	static constexpr std::uint8_t RegAesKey2       = 0x3F;
	static constexpr std::uint8_t RegAesKey3       = 0x40;
	static constexpr std::uint8_t RegAesKey4       = 0x41;
	static constexpr std::uint8_t RegAesKey5       = 0x42;
	static constexpr std::uint8_t RegAesKey6       = 0x43;
	static constexpr std::uint8_t RegAesKey7       = 0x44;
	static constexpr std::uint8_t RegAesKey8       = 0x45;
	static constexpr std::uint8_t RegAesKey9       = 0x46;
	static constexpr std::uint8_t RegAesKey10      = 0x47;
	static constexpr std::uint8_t RegAesKey11      = 0x48;
	static constexpr std::uint8_t RegAesKey12      = 0x49;
	static constexpr std::uint8_t RegAesKey13      = 0x4A;
	static constexpr std::uint8_t RegAesKey14      = 0x4B;
	static constexpr std::uint8_t RegAesKey15      = 0x4C;
	static constexpr std::uint8_t RegAesKey16      = 0x4D;

	/* Temp Sensor Registers */
	static constexpr std::uint8_t RegTemp1 = 0x4E;
	static constexpr std::uint8_t RegTemp2 = 0x4F;
	/* Test Registers*/
	static constexpr std::uint8_t RegTestLna  = 0x58;
	static constexpr std::uint8_t RegTestPa1  = 0x5A;
	static constexpr std::uint8_t RegTestPa2  = 0x5C;
	static constexpr std::uint8_t RegTestDagc = 0x6F;
	static constexpr std::uint8_t RegTestAfc =  0x71;


	/*RegOpMode*/
	static constexpr std::uint8_t SequencerOff = 0x00;
	static constexpr std::uint8_t SequencerOn = 1 << 7;
	static constexpr std::uint8_t ListenOff = 0x00;
	static constexpr std::uint8_t ListenOn = 1 << 6;
	static constexpr std::uint8_t ListenAbort = 1 << 5;
	/* changed to enums */
	static constexpr std::uint8_t _ModeSleep = 0b00000;
	static constexpr std::uint8_t _ModeStandby = 1<<2;
	static constexpr std::uint8_t _ModeFreq = 2 << 2;
	static constexpr std::uint8_t _ModeTx = 3 << 2;
	static constexpr std::uint8_t _ModeRx = 4 << 2;
	/* ---------------- */

	enum class Mode
	{
	    Sleep   = 0x00,
	    Standby = 0x04,
	    Freq    = 0x08,
	    Tx      = 0x0C,
	    Rx      = 0x10
	};

	/*RegDataModul*/
	static constexpr std::uint8_t DataModePacket = 0 << 5;
	static constexpr std::uint8_t DataModeContSync = 2 << 5;
	static constexpr std::uint8_t DataModeCont = 3 << 5;
	static constexpr std::uint8_t ModFSK = 0x00;
	static constexpr std::uint8_t ModOOK = 1 << 3;
	static constexpr std::uint8_t ModNoShaping = 0x00;
	static constexpr std::uint8_t ModShapingBT1 = 1;
	static constexpr std::uint8_t ModShapingBT05 = 2;
	static constexpr std::uint8_t ModShapingBT03 = 3;
	static constexpr std::uint8_t ModShapingBR = 1;
	static constexpr std::uint8_t ModShaping2BR = 2;

	/* RegOsc1 */
	static constexpr std::uint8_t RcCalStart = 1 << 7;
	static constexpr std::uint8_t RcCalDone = 1 << 6;

	/* RegAfcCtrl*/
	static constexpr std::uint8_t AfcLowBetaOn = 1 << 5;

	/* RegListen1*/
	static constexpr std::uint8_t ListenResolIdle64u = 1 << 6;
	static constexpr std::uint8_t ListenResolIdle41ms = 2 << 6;
	static constexpr std::uint8_t ListenResolIdle262ms = 3 << 6;
	static constexpr std::uint8_t ListenResolRx64u = 1 << 4;
	static constexpr std::uint8_t ListenResolRx41ms = 2 << 4;
	static constexpr std::uint8_t ListenResolRx262ms = 3 << 4;
	static constexpr std::uint8_t ListenCriteriaRssi = 0x00;
	static constexpr std::uint8_t ListenCriteriaRssiSyncAddr = 1<<3;
	static constexpr std::uint8_t ListenEndStop = 0x00;
	static constexpr std::uint8_t ListenEndMode = 1 << 1;
	static constexpr std::uint8_t ListenEndRx = 2 << 1;

	/* RegPaLevel */
	static constexpr std::uint8_t Pa0On       = 1<<7;
	static constexpr std::uint8_t Pa1On       = 1<<6;
	static constexpr std::uint8_t Pa2On 	  = 1<<5;
	static constexpr std::uint8_t OutputPower = 0x1F;

	/*RegPaRamp */
	static constexpr std::uint8_t PaRamp3_4ms = 0x00;
	static constexpr std::uint8_t PaRamp2ms = 0x01;
	static constexpr std::uint8_t PaRamp1ms = 0x02;
	static constexpr std::uint8_t PaRamp500us = 0x03;
	static constexpr std::uint8_t PaRamp250us = 0x04;
	static constexpr std::uint8_t PaRamp125us = 0x05;
	static constexpr std::uint8_t PaRamp100us = 0x06;
	static constexpr std::uint8_t PaRamp62us = 0x07;
	static constexpr std::uint8_t PaRamp50us = 0x08;
	static constexpr std::uint8_t PaRamp40us = 0x09;
	static constexpr std::uint8_t PaRamp31us = 0x0A;
	static constexpr std::uint8_t PaRamp25us = 0x0B;
	static constexpr std::uint8_t PaRamp20us = 0x0C;
	static constexpr std::uint8_t PaRamp15us = 0x0D;
	static constexpr std::uint8_t PaRamp12us = 0x0E;
	static constexpr std::uint8_t PaRamp10us = 0x0F;

	/* RegOcp */
	static constexpr std::uint8_t OcpOn = 1 << 4;
	static constexpr std::uint8_t OcpOff = 1 << 4;

	/*RegLna*/
	static constexpr std::uint8_t LnaZin50 = 0x00;
	static constexpr std::uint8_t LnaZin200 = 1<<7;
	static constexpr std::uint8_t LnaCurrentGain = 1 << 3;
	static constexpr std::uint8_t LnaGainSelectG1 = 0x00;
	static constexpr std::uint8_t LnaGainSelectG2 = 0x01;
	static constexpr std::uint8_t LnaGainSelectG3 = 0x03;
	static constexpr std::uint8_t LnaGainSelectG4 = 0x04;
	static constexpr std::uint8_t LnaGainSelectG5 = 0x05;
	static constexpr std::uint8_t LnaGainSelectG6 = 0x06;

	/* RegRxBw */
	static constexpr std::uint8_t RxBwMant16 = 0x00;
	static constexpr std::uint8_t RxBwMant20 = 0x01 << 3;
	static constexpr std::uint8_t RxBwMant24 = 0x02 << 3;

	/*RegOokPeak*/
	static constexpr std::uint8_t OokThreshFixed = 0x00;
	static constexpr std::uint8_t OokThreshPeak = 1 << 6;
	static constexpr std::uint8_t OokThreshAve = 2 << 6;
	static constexpr std::uint8_t OokPeakTheshStep0_5db = 0x00;
	static constexpr std::uint8_t OokPeakTheshStep1db = 1 << 3;
	static constexpr std::uint8_t OokPeakTheshStep1_5db = 2 << 3;
	static constexpr std::uint8_t OokPeakTheshStep2db = 3 << 3;
	static constexpr std::uint8_t OokPeakTheshStep3db = 4 << 3;
	static constexpr std::uint8_t OokPeakTheshStep4db = 5 << 3;
	static constexpr std::uint8_t OokPeakTheshStep5db = 6 << 3;
	static constexpr std::uint8_t OokPeakTheshStep6db = 7 << 3;
	static constexpr std::uint8_t OokPeakThreshDec1Chip = 0x00;
	static constexpr std::uint8_t OokPeakThreshDec2Chip = 1;
	static constexpr std::uint8_t OokPeakThreshDec4Chip = 2;
	static constexpr std::uint8_t OokPeakThreshDec8Chip = 3;
	static constexpr std::uint8_t OokPeakThreshDecHalfChip = 4;
	static constexpr std::uint8_t OokPeakThreshDecQuartChip = 5;
	static constexpr std::uint8_t OokPeakThreshDecEighthChip = 6;
	static constexpr std::uint8_t OokPeakThreshDecSixTeenthChip = 7;
	static constexpr std::uint8_t OokAverageThreshFilt = 8;

	/*RegAfcFei*/
	static constexpr std::uint8_t FeiDone = 1<<6;
	static constexpr std::uint8_t FeiStart = 1<<5;
	static constexpr std::uint8_t AfcDone = 1 << 4;
	static constexpr std::uint8_t AfcAutoclearOn = 0x00;
	static constexpr std::uint8_t AfcAutoclearOff = 1 << 3;
	static constexpr std::uint8_t fcAutoOn = 1 << 2;
	static constexpr std::uint8_t fcAutoOff = 0x00;
	static constexpr std::uint8_t AfcClear = 1 << 1;
	static constexpr std::uint8_t AfcStart = 1;

	/*RegRssiConfig*/
	static constexpr std::uint8_t RssiDone = 1 << 1;
	static constexpr std::uint8_t RssiStart = 1;

	/* RegDioMapping1 */
	static constexpr std::uint8_t Dio0Mapping = 0x01 << 6;
	static constexpr std::uint8_t Dio1Mapping = 0x01 << 4;
	static constexpr std::uint8_t Dio2Mapping = 0x01 << 2;
	static constexpr std::uint8_t Dio3Mapping = 0x01;

	/* RegDioMapping1 */
	static constexpr std::uint8_t Dio4Mapping = 0x01 << 6;
	static constexpr std::uint8_t Dio5Mapping = 0x01 << 4;
	static constexpr std::uint8_t ClkOut = 0x00;
	static constexpr std::uint8_t ClkOutDiv2 = 0x01;
	static constexpr std::uint8_t ClkOutDiv4 = 0x02;
	static constexpr std::uint8_t ClkOutDiv8 = 0x03;
	static constexpr std::uint8_t ClkOutDiv16 = 0x04;
	static constexpr std::uint8_t ClkOutDiv32 = 0x05;
	static constexpr std::uint8_t ClkOutRC = 0x06;
	static constexpr std::uint8_t ClkOutOff = 0x07;

	/*RegIrqFlags1*/
	static constexpr std::uint8_t ModeReady = 1 << 7;
	static constexpr std::uint8_t RxReady = 1 << 6;
	static constexpr std::uint8_t TxReady = 1 << 5;
	static constexpr std::uint8_t PllLock = 1 << 4;
	static constexpr std::uint8_t Rssi = 1 << 3;
	static constexpr std::uint8_t Timeout = 1 << 2;
	static constexpr std::uint8_t AutoMode = 1 << 1;
	static constexpr std::uint8_t SyncAddressMatch = 1;

	/*RegIrqFlags2*/
	static constexpr std::uint8_t FifoFull = 1 << 7;
	static constexpr std::uint8_t FifoNotEmpty = 1 << 6;
	static constexpr std::uint8_t FifoLevel = 1 << 5;
	static constexpr std::uint8_t FifoOverrun = 1 << 4;
	static constexpr std::uint8_t PacketSent = 1 << 3;
	static constexpr std::uint8_t PayloadReady = 1 << 2;
	static constexpr std::uint8_t CrcOk = 1 << 1;

	/* RegSyncConfig */
	static constexpr std::uint8_t SyncOn = 1 << 7;
	static constexpr std::uint8_t FifoFillConditionAuto = 1 << 6;
	static constexpr std::uint8_t SyncSize = 0x38;
	static constexpr std::uint8_t SyncSize2 = 0x02 << 3;
	static constexpr std::uint8_t SyncTol0 = 0;

	/* RegPacketConfig1*/
	static constexpr std::uint8_t PacketFormat = 1 << 7;
	static constexpr std::uint8_t DcFreeNone = 0x00 << 5;
	static constexpr std::uint8_t DcFreeManc = 0x01 << 5;
	static constexpr std::uint8_t DcFreeWhite = 2 << 5;
	static constexpr std::uint8_t CrcOn = 1 << 4;
	static constexpr std::uint8_t CrcAutoClearOff = 1 << 3;
	static constexpr std::uint8_t AddressFilteringMatchNode = 1 << 1;
	static constexpr std::uint8_t AddressFilteringMatchBoth = 2 << 1;


	/* RegAutoModes */
	static constexpr std::uint8_t EnterConditionNone = 0x00;
	static constexpr std::uint8_t EnterConditionFifoNotEmpty = 1<< 5;
	static constexpr std::uint8_t EnterConditionFifoLevel = 2<< 5;
	static constexpr std::uint8_t EnterConditionCrcOk = 3<< 5;
	static constexpr std::uint8_t EnterConditionPayloadReady = 4<< 5;
	static constexpr std::uint8_t EnterConditionSyncAddress = 5<< 5;
	static constexpr std::uint8_t EnterConditionPacketSent = 6<< 5;
	static constexpr std::uint8_t EnterConditionFifoEmpty = 7 << 5;
	static constexpr std::uint8_t ExitConditionNone = 0x00;
	static constexpr std::uint8_t ExitConditionFifoEmpty = 1 << 2;
	static constexpr std::uint8_t ExitConditionFifoLevel_TO = 2 << 2;
	static constexpr std::uint8_t ExitConditionCrcOk_TO = 3 << 2;
	static constexpr std::uint8_t ExitConditionPayloadReady_TO = 4 << 2;
	static constexpr std::uint8_t ExitConditionSyncAddress_TO = 5 << 2;
	static constexpr std::uint8_t ExitConditionPacketSent = 6 << 2;
	static constexpr std::uint8_t ExitConditionTO = 7 << 2;
	static constexpr std::uint8_t IntermediateModeSleep = 0x00;
	static constexpr std::uint8_t IntermediateModeStndby = 1;
	static constexpr std::uint8_t IntermediateModeRx = 2;
	static constexpr std::uint8_t IntermediateModeTx = 3;

	/* RegFifoThresh */
	static constexpr std::uint8_t TxStartConditionFifoLevel = 0x00;
	static constexpr std::uint8_t TxStartConditionFifoNotEmpty = 1 << 7;
	static constexpr std::uint8_t FifoThresholdValue = 0x0f;

	/* RegPacketConfig2 */
	static constexpr std::uint8_t InterPacketRxDelay2Bits = 0b0011 << 4;
	static constexpr std::uint8_t RestartRxOff = 1 << 2;
	static constexpr std::uint8_t AutoRxRestartOn = 1 << 1;
	static constexpr std::uint8_t AutoRxRestartOff = 0x00;
	static constexpr std::uint8_t AesOn = 1;
	static constexpr std::uint8_t AesOff = 0x00;
	/*RegTemp1*/
	static constexpr std::uint8_t TempMeasStart = 1 << 3;
	static constexpr std::uint8_t TempMeasRunning = 1 <<2;

	/*RegTestLna*/
	static constexpr std::uint8_t SensitivityBoostNormal = 0x1B;
	static constexpr std::uint8_t SensitivityBoostHighSens = 0x2D;

	/*RegTestPa1*/
	static constexpr std::uint8_t TestPa1Normal = 0x55;
	static constexpr std::uint8_t Pa20dBm120dBm = 0x5D;

	/*RegTestPa2*/
	static constexpr std::uint8_t TestPa2Normal = 0x70;
	static constexpr std::uint8_t Pa20dBm220dBm = 0x7C;

	/* RegTestDagc */
	static constexpr std::uint8_t ContinuousDagcNormal = 0x00;
	static constexpr std::uint8_t ContinuousDagcImprovedBeta = 0x20;
	static constexpr std::uint8_t ContinuousDagcImproved = 0x30;

};
