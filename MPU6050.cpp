#include "MPU6050.hpp"




MPU6050::MPU6050(I2C_HandleTypeDef* i2c, std::uint8_t device_address)
{
    m_i2c = i2c;
    m_device_address = device_address << 1;
    m_initDone = false;
    m_OffsetAx = 0;
    m_OffsetAy = 0;
    m_OffsetAz = 0;
    m_OffsetGx = 0;
    m_OffsetGy = 0;
    m_OffsetGz = 0;
    m_OffsetAngleX = 0.0;
    m_OffsetAngleY = 0.0;

    m_kalmanAngleRoll = 0.0;
    m_kalmanUncertaintyAngleRoll = 0.0;
    m_kalmanAnglePitch = 0.0;
    m_kalmanUncertaintyAnglePitch = 0.0;
}

MPU6050::MPU6050()
{
    m_initDone = false;
    m_OffsetAx = 0;
    m_OffsetAy = 0;
    m_OffsetAz = 0;
    m_OffsetGx = 0;
    m_OffsetGy = 0;
    m_OffsetGz = 0;
    m_OffsetAngleX = 0.0;
    m_OffsetAngleY = 0.0;

    m_kalmanAngleRoll = 0.0;
    m_kalmanUncertaintyAngleRoll = 0.0;
    m_kalmanAnglePitch = 0.0;
    m_kalmanUncertaintyAnglePitch = 0.0;
}

void MPU6050::setI2C(I2C_HandleTypeDef* i2c)
{
    m_i2c = i2c;
}

void MPU6050::setDeviceAddress(std::uint8_t deviceAddress)
{
    m_device_address = deviceAddress << 1;
}

bool MPU6050::init()
{
    HAL_I2C_DeInit(m_i2c);
    HAL_I2C_Init(m_i2c);

    bool init_ok = true;
    HAL_Delay(100);

    // Check who_am_i for address read.
    std::uint8_t check[] = {0};
    init_ok &= HAL_I2C_Mem_Read(m_i2c, m_device_address, MPU6050_REG::WHO_AM_I, 1, check, 1, 200) == HAL_OK;
    if(*check != m_device_address >> 1){return false;}

    // Set PWR_MGMT_1 to 0x00 - to wake up and set clock to 8mhz.
    std::uint8_t data[] = {0};
    init_ok &= HAL_I2C_Mem_Write(m_i2c, m_device_address, MPU6050_REG::PWR_MGMT_1, 1, data, 1, 200) == HAL_OK;

    // Set data rate to 1khz
    data[0] = 0x07;
    init_ok &= HAL_I2C_Mem_Write(m_i2c, m_device_address, MPU6050_REG::SMPLRT_DIV, 1, data, 1, 200) == HAL_OK;

    // Set DLPF to 10khz
    data[0] = 0x05;
    init_ok &= HAL_I2C_Mem_Write(m_i2c, m_device_address, MPU6050_REG::CONFIG, 1, data, 1, 200) == HAL_OK;


    // Set the full scale range of accel to 2g.
    data[0] = 0x00;
    init_ok &= HAL_I2C_Mem_Write(m_i2c, m_device_address, MPU6050_REG::ACCEL_CONFIG, 1, data, 1, 200) == HAL_OK;

    // Set the full scale range of gyro to 2000.
    data[0] = 0x07;
    init_ok &= HAL_I2C_Mem_Write(m_i2c, m_device_address, MPU6050_REG::GYRO_CONFIG, 1, data, 1, 200) == HAL_OK;

    m_initDone=init_ok;





    return init_ok;
}

bool MPU6050::calibrateGyro()
{
    if(!m_initDone){ return false;}

    float GxValue=0; float GyValue=0; float GzValue=0;


    for(std::uint32_t calibrationNumber = 0; calibrationNumber<=2000; calibrationNumber++)
    {
	std::array<float, 3> gyro = getGyro();

	GxValue+=gyro[0]; GyValue+=gyro[1]; GzValue+=gyro[2];
    }

    m_OffsetGx = GxValue/2000; m_OffsetGy = GyValue/2000; m_OffsetGz = GzValue/2000;

    if(GxValue == 0 || GyValue == 0 || GzValue == 0)
    {
	return false;
    }
    else
    {
	return true;
    }
}

float MPU6050::getOffset(Offset offset)
{
    switch(offset)
    {
	case Ax:
	    return m_OffsetAx;
	case Ay:
	    return m_OffsetAx;
	case Az:
	    return m_OffsetAx;
	case Gx:
	    return m_OffsetAx;
	case Gy:
	    return m_OffsetAx;
	case Gz:
	    return m_OffsetAx;
	default:
	    return 0;
    }
}

void MPU6050::setOffset(Offset offset, float value)
{
    switch(offset)
    {
	case Ax:
	    m_OffsetAx = value;
	case Ay:
	    m_OffsetAy = value;
	case Az:
	    m_OffsetAz = value;
	case Gx:
	    m_OffsetGx = value;
	case Gy:
	    m_OffsetGy = value;
	case Gz:
	    m_OffsetGz = value;
    }
}

std::array<float, 3> MPU6050::getAccel()
{
    // Read accel data
    std::uint8_t accel_data[6] = {0};
    HAL_I2C_Mem_Read(m_i2c, m_device_address, MPU6050_REG::ACCEL_XOUT_H, 1, accel_data, 6, 200);
    std::int16_t accel_x_raw = (std::uint16_t)accel_data[0] << 8 | accel_data[1];
    std::int16_t accel_y_raw = (std::uint16_t)accel_data[2] << 8 | accel_data[3];
    std::int16_t accel_z_raw = (std::uint16_t)accel_data[4] << 8 | accel_data[5];

    float Ax = (accel_x_raw/ACCEL_LSB) - m_OffsetAx;
    float Ay = (accel_y_raw/ACCEL_LSB) - m_OffsetAy;
    float Az = (accel_z_raw/ACCEL_LSB) - m_OffsetAz;

    return {Ax, Ay, Az};
}

std::array<float, 2> MPU6050::getAccelAngle()
{
    std::array<float, 3> accel = getAccel();

    float angleXRad = atan(accel[1]/sqrt(accel[0]*accel[0] + accel[2]*accel[2]));
    float angleYRad = -atan(accel[0]/sqrt(accel[1]*accel[1] + accel[2]*accel[2]));

    float angleXDeg = angleXRad * 1/(3.1415/180);
    float angleYDeg = angleYRad * 1/(3.1415/180);

    return {angleXDeg - m_OffsetAngleX, angleYDeg - m_OffsetAngleY};
}

bool MPU6050::calibrateAccelAngle()
{
    float AxValue=0; float AyValue=0;

    for(std::uint32_t calibrationNumber = 0; calibrationNumber<=2000; calibrationNumber++)
    {
	std::array<float, 2> accelAngle = getAccelAngle();
	AxValue+=accelAngle[0]; AyValue+=accelAngle[1];
    }

    m_OffsetAngleX = AxValue/2000; m_OffsetAngleY = AyValue/2000;

    if(AxValue == 0 || AyValue == 0)
    {
	return false;
    }
    else
    {
	return true;
    }
}

std::array<float, 3> MPU6050::getGyro()
{
    // Read gyro data
    std::uint8_t gyro_data[6] = {0};
    HAL_I2C_Mem_Read(m_i2c, m_device_address, MPU6050_REG::GYRO_XOUT_H, 1, gyro_data, 6, 200);
    std::int16_t gyro_x_raw = (std::uint16_t)gyro_data[0] << 8 | gyro_data[1];
    std::int16_t gyro_y_raw = (std::uint16_t)gyro_data[2] << 8 | gyro_data[3];
    std::int16_t gyro_z_raw = (std::uint16_t)gyro_data[4] << 8 | gyro_data[5];

    float Gx = (gyro_x_raw/GYRO_LSB) - m_OffsetGx;
    float Gy = (gyro_y_raw/GYRO_LSB) - m_OffsetGy;
    float Gz = (gyro_z_raw/GYRO_LSB) - m_OffsetGz;

    return {Gx, Gy, Gz};
}

float MPU6050::getTemp()
{
    // Read temp data
    std::uint8_t temp_data[2] = {0};
    HAL_I2C_Mem_Read(m_i2c, m_device_address, MPU6050_REG::TEMP_OUT_H, 1, temp_data, 2, 200);
    std::int16_t temp_raw = (std::uint16_t)temp_data[0] << 8 | temp_data[1];

    float temp = temp_raw/340 + 36.53;

    return temp;
}

std::array<float, 3> MPU6050::getAngle()
{
    return {0, 0, 0};
}

std::array<float, 3> MPU6050::getVel()
{
    return {0, 0, 0};
}

std::array<float,2> MPU6050::kalmanFilter(float state, float uncertainty, float input, float measurement)
{
    state = state + 0.004*input;
    uncertainty = uncertainty + (0.5 * 0.5 * 2 * 2);

    float gain = uncertainty * 1/(1*uncertainty + pow(3,2));
    state = state + gain * (measurement - state);
    uncertainty = (1 - gain) * uncertainty;

    return {state, uncertainty};

}
bool MPU6050::getInitDone()
{
    return m_initDone;
}

std::array<float,2> MPU6050::getKalmanOutput()
{
    std::array<float, 3> gyro = getGyro();
    std::array<float, 2> accelAngle = getAccelAngle();

    float rateRoll = gyro[0];
    float ratePitch = gyro[1];
    float angleRoll = accelAngle[0];
    float anglePitch = accelAngle[1];

    std::array<float, 2> kalmanOutput = kalmanFilter(m_kalmanAngleRoll, m_kalmanUncertaintyAngleRoll, rateRoll, angleRoll);
    m_kalmanAngleRoll = kalmanOutput[0];
    m_kalmanUncertaintyAngleRoll = kalmanOutput[1];

    kalmanOutput = kalmanFilter(m_kalmanAnglePitch, m_kalmanUncertaintyAnglePitch, ratePitch, anglePitch);
    m_kalmanAnglePitch = kalmanOutput[0];
    m_kalmanUncertaintyAnglePitch = kalmanOutput[1];

    return {m_kalmanAngleRoll, m_kalmanAnglePitch};
}
