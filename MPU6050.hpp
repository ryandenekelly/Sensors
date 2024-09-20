/* some parts derived from https://www.arduino.cc/reference/en/libraries/mpu6050/ */

#include "MPU6050_REG.hpp"
#include <cstdint>
#include <array>
#include <math.h>

#include "i2c.h"


class MPU6050
{
    public:
	MPU6050(I2C_HandleTypeDef* i2c, std::uint8_t device_address);
	MPU6050();
	enum Offset
	{
	    Ax = 0,
	    Ay,
	    Az,
	    Gx,
	    Gy,
	    Gz
	};

	bool init();
	void setI2C(I2C_HandleTypeDef*);
	void setDeviceAddress(std::uint8_t);
	std::array<float, 3> getAccel();
	std::array<float, 2> getAccelAngle();
	bool calibrateAccelAngle();
	std::array<float, 3> getGyro();
	bool calibrateGyro();
	std::array<float,2> kalmanFilter(float state, float uncertainty, float input, float measurement);
	float getTemp();
	float getOffset(Offset offset);
	void setOffset(Offset offset, float value);
	std::array<float, 3> getAngle();
	std::array<float, 3> getVel();

	bool getInitDone();
	std::array<float,2> getKalmanOutput();

	static constexpr float GYRO_LSB = 16.38;
	static constexpr float ACCEL_LSB = 16384.0;





    private:
	I2C_HandleTypeDef* m_i2c;
	std::uint8_t m_device_address;
	bool m_initDone;
	float m_OffsetAx;
	float m_OffsetAy;
	float m_OffsetAz;
	float m_OffsetAngleX;
	float m_OffsetAngleY;
	float m_OffsetGx;
	float m_OffsetGy;
	float m_OffsetGz;

	float m_kalmanAngleRoll;
	float m_kalmanUncertaintyAngleRoll;
	float m_kalmanAnglePitch;
	float m_kalmanUncertaintyAnglePitch;











};
