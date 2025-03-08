#pragma once
#include <rev/SparkMax.h>
#include <optional>
#include <algorithm>

class MotorRegulator
{
private:
    rev::spark::SparkMax *spark_max;
    rev::spark::SparkClosedLoopController *closed_loop;
    std::optional<double> limit_up, limit_down;

    double input = 0;
    double position_ratio = 1;
    const double throttle_speed;

public:
    MotorRegulator(rev::spark::SparkMax *spark_max, rev::spark::SparkClosedLoopController *closed_loop, double throttle_speed) : spark_max(spark_max), closed_loop(closed_loop), throttle_speed(throttle_speed)
    {
        // WARNING: motor must stay in the same place on boot !!!
        spark_max->GetEncoder().SetPosition(0);
    }
    ~MotorRegulator() {}

    void SetReference(double p_input)
    {
        input = std::clamp(p_input, -1.0, 1.0);
    }

    void SetRatio(double p_ratio)
    {
        position_ratio = p_ratio;
    }

    void SetLimits(double p_limit_up, double p_limit_down)
    {
        limit_up = p_limit_up;
        limit_down = p_limit_down;
    }

    void Periodic()
    {
        double position = spark_max->GetEncoder().GetPosition() / position_ratio;
        double target_speed = input * throttle_speed;
        if (limit_down.has_value() && limit_up.has_value() && (limit_down > position + target_speed/10 || position + target_speed/10 > limit_up))
            target_speed = 0;

        printf("sending %f with curently %f\n", target_speed, position);
        closed_loop->SetReference(target_speed, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
    }
};
