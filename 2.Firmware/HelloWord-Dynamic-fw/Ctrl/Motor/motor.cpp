#include "motor.h"
#include "st_hardware.h"
#include <cmath>


bool Motor::Init(float _zeroElectricOffset, EncoderBase::Direction _encoderDir)
{
    if (encoder) encoder->Init();
    if (driver) driver->Init();

    if (config.voltageLimit > driver->voltagePowerSupply)
        config.voltageLimit = driver->voltagePowerSupply;

    if (config.voltageUsedForSensorAlign > config.voltageLimit)
        config.voltageUsedForSensorAlign = config.voltageLimit;

    config.pidVelocity.limit = config.voltageLimit;
    config.pidAngle.limit = config.velocityLimit;

    return InitFOC(_zeroElectricOffset, _encoderDir);
}


void Motor::AttachEncoder(EncoderBase* _encoder)
{
    encoder = _encoder;
}


void Motor::AttachDriver(DriverBase* _driver)
{
    driver = _driver;
}


void Motor::SetEnable(bool _enable)
{
    enabled = _enable;
}


// 获取估计累计角度
float Motor::GetEstimateAngle()
{
    // If no sensor linked return previous value (for open-loop case)
    if (!encoder) return estimateAngle;
    state.rawAngle = (float) (encoder->countDirection) * encoder->GetFullAngle();
    state.estAngle = config.lpfAngle(state.rawAngle);

    return state.estAngle;
}


// 获取估计速度
float Motor::GetEstimateVelocity()
{
    // If no sensor linked return previous value (for open-loop case)
    if (!encoder) return estimateVelocity;
    state.rawVelocity = (float) (encoder->countDirection) * encoder->GetVelocity();
    state.estVelocity = config.lpfVelocity(state.rawVelocity);

    return state.estVelocity;
}


// 获取周期角度
float Motor::GetElectricalAngle()
{
    // If no sensor linked return previous value (for open-loop case)
    return encoder ? Normalize((float) (encoder->countDirection * polePairs)
                               * encoder->GetLapAngle() - zeroElectricAngleOffset)
                   : electricalAngle;
}


void Motor::Tick()
{
    CloseLoopControlTick();
    FocOutputTick();
}


bool Motor::InitFOC(float _zeroElectricOffset, EncoderBase::Direction _sensorDirection)
{
    // Absolute zero offset provided, no need to align
    if (ASSERT(_zeroElectricOffset))
    {
        zeroElectricAngleOffset = _zeroElectricOffset;
        encoder->countDirection = _sensorDirection;
    }

    if (encoder)
    {
        if (!AlignSensor()) return false;

        encoder->Update();
        estimateAngle = GetEstimateAngle();
    }

    return true;
}


bool Motor::AlignSensor()
{
    if (encoder->countDirection == EncoderBase::UNKNOWN)
    {
        SetPowerMotor(true);
        HAL_Delay(100);

        // Find natural direction
        for (int i = 0; i <= 500; i++)
        {
            float angle = _3PI_2 + _2PI * (float) i / 500.0f;
            SetPhaseVoltage(config.voltageUsedForSensorAlign, 0, angle);
            HAL_Delay(2);
        }

        encoder->Update();
        float midAngle = encoder->GetFullAngle();

        for (int i = 500; i >= 0; i--)
        {
            float angle = _3PI_2 + _2PI * (float) i / 500.0f;
            SetPhaseVoltage(config.voltageUsedForSensorAlign, 0, angle);
            HAL_Delay(2);
        }

        encoder->Update();
        float endAngle = encoder->GetFullAngle();

        SetPhaseVoltage(0, 0, 0);
        SetPowerMotor(false);
        HAL_Delay(200);

        // Determine the direction the sensor moved
        if (midAngle == endAngle)
        {
            error = FAILED_TO_NOTICE_MOVEMENT;
            return false;
        } else if (midAngle < endAngle)
        {
            encoder->countDirection = EncoderBase::Direction::CCW;
        } else
        {
            encoder->countDirection = EncoderBase::Direction::CW;
        }

        // Check pole pair number
        float deltaAngle = std::fabs(midAngle - endAngle);
        if (std::fabs(deltaAngle * (float) polePairs - _2PI) > 0.5f)
        {
            // 0.5f is arbitrary number, it can be tuned
            error = POLE_PAIR_MISMATCH;
            return false;
        }
    }

    // Align the electrical phases of the motor and sensor
    if (!ASSERT(zeroElectricAngleOffset))
    {
        SetPowerMotor(true);
        // Set angle -90(270 = 3PI/2) degrees
        SetPhaseVoltage(config.voltageUsedForSensorAlign, 0, _3PI_2);

        HAL_Delay(1000);
        encoder->Update();
        zeroElectricAngleOffset = 0; // Clear offset first
        zeroElectricAngleOffset = GetElectricalAngle();

        SetPhaseVoltage(0, 0, 0);
        SetPowerMotor(false);
        HAL_Delay(200);
    }

    return true;
}


void Motor::CloseLoopControlTick()
{
    estimateAngle = GetEstimateAngle();
    estimateVelocity = GetEstimateVelocity();

    if (!enabled) return;

    switch (config.controlMode)
    {
        // 力矩模式
        case ControlMode_t::TORQUE:
            voltage.q = target;
            voltage.d = 0;
            setPointCurrent = voltage.q;
            break;
        // 角度模式
        case ControlMode_t::ANGLE:
            setPointAngle = target;
            setPointVelocity = config.pidAngle(setPointAngle - estimateAngle);
            setPointCurrent = config.pidVelocity(setPointVelocity - estimateVelocity);
            break;
        // 速度模式
        case ControlMode_t::VELOCITY:
            setPointVelocity = target;
            setPointCurrent = config.pidVelocity(setPointVelocity - estimateVelocity);
            break;
    }
}


void Motor::FocOutputTick()
{
    if (encoder) encoder->Update();

    if (!enabled) return;

    electricalAngle = GetElectricalAngle();

    voltage.q = setPointCurrent;
    voltage.d = 0;

    SetPhaseVoltage(voltage.q, voltage.d, electricalAngle);
}


void Motor::SetPhaseVoltage(float _voltageQ, float _voltageD, float _angleElectrical)
{
    float uOut;

    if (_voltageD != 0)
    {
        uOut = SQRT(_voltageD * _voltageD + _voltageQ * _voltageQ) / driver->voltagePowerSupply;
        _angleElectrical = Normalize(_angleElectrical + std::atan2(_voltageQ, _voltageD));
    } else
    {
        uOut = _voltageQ / driver->voltagePowerSupply;
        _angleElectrical = Normalize(_angleElectrical + _PI_2);
    }
    uint8_t sec = (int) (std::floor(_angleElectrical / _PI_3)) + 1;
    float t1 = _SQRT3
               * SinApprox((float) (sec) * _PI_3 - _angleElectrical)
               * uOut;
    float t2 = _SQRT3
               * SinApprox(_angleElectrical - ((float) (sec) - 1.0f) * _PI_3)
               * uOut;
    float t0 = 1 - t1 - t2;

    float tA, tB, tC;
    switch (sec)
    {
        case 1:
            tA = t1 + t2 + t0 / 2;
            tB = t2 + t0 / 2;
            tC = t0 / 2;
            break;
        case 2:
            tA = t1 + t0 / 2;
            tB = t1 + t2 + t0 / 2;
            tC = t0 / 2;
            break;
        case 3:
            tA = t0 / 2;
            tB = t1 + t2 + t0 / 2;
            tC = t2 + t0 / 2;
            break;
        case 4:
            tA = t0 / 2;
            tB = t1 + t0 / 2;
            tC = t1 + t2 + t0 / 2;
            break;
        case 5:
            tA = t2 + t0 / 2;
            tB = t0 / 2;
            tC = t1 + t2 + t0 / 2;
            break;
        case 6:
            tA = t1 + t2 + t0 / 2;
            tB = t0 / 2;
            tC = t1 + t0 / 2;
            break;
        default:
            tA = 0;
            tB = 0;
            tC = 0;
    }

    // calculate the phase voltages and center
    voltageA = tA * driver->voltagePowerSupply;
    voltageB = tB * driver->voltagePowerSupply;
    voltageC = tC * driver->voltagePowerSupply;

    driver->SetVoltage(voltageA, voltageB, voltageC);
}


// 设置力矩限制
void Motor::SetTorqueLimit(float _val)
{
    config.voltageLimit = _val;

    if (config.voltageLimit > driver->voltagePowerSupply)
        config.voltageLimit = driver->voltagePowerSupply;

    if (config.voltageUsedForSensorAlign > config.voltageLimit)
        config.voltageUsedForSensorAlign = config.voltageLimit;

    config.pidVelocity.limit = config.voltageLimit;
    config.pidAngle.limit = config.velocityLimit;
}







