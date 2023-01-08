#ifndef HELLOWORD_DYNAMIC_FW_KNOB_H
#define HELLOWORD_DYNAMIC_FW_KNOB_H

#include "motor.h"


class KnobSimulator
{
public:
    KnobSimulator() = default;;

    enum Mode_t
    {
        MODE_DISABLE = 0,
        MODE_INERTIA, // 惯性模式
        MODE_ENCODER, // 编码器模式
        MODE_SPRING,  // 弹簧模式
        MODE_DAMPED,  // 阻尼模式
        MODE_SPIN     // 旋转模式
    };

    void Init(Motor* _motor);
    void Tick();
    void SetEnable(bool _en);
    void ApplyZeroPos(float _angle = 0);
    void SetMode(Mode_t _mode);
    Mode_t GetMode();
    void SetLimitPos(float _min, float _max);
    float GetPosition();
    float GetVelocity();
    int GetEncoderModePos();


private:
    Motor* motor{};
    Mode_t mode = MODE_DISABLE;
    float zeroPosition = 0;
    float limitPositionMax = 5.1;
    float limitPositionMin = 3.3;
    int encoderDivides = 12;
    int encoderPosition = 0;

    float lastAngle{};
    float lastVelocity{};
};

#endif //HELLOWORD_DYNAMIC_FW_KNOB_H
