#pragma once
#include <rev/SparkMax.h>
#include <optional>
#include <algorithm>

class MotorRegulator
{
private:
    rev::spark::SparkMax *spark_max;
    rev::spark::SparkClosedLoopController *closed_loop;
    std::optional<double> target_up, target_down;

    double position_ratio = 1;
    const double throttle_speed;

public:
    MotorRegulator(rev::spark::SparkMax *spark_max, rev::spark::SparkClosedLoopController *closed_loop, double throttle_speed) : spark_max(spark_max), closed_loop(closed_loop), throttle_speed(throttle_speed)
    {
        // WARNING: motor must stay in the same place on boot !!!
    }
    ~MotorRegulator() {}

    void SetRatio(double p_ratio)
    {
        position_ratio = p_ratio;
    }

    void SetTargets(double p_target_up, double p_target_down)
    {
        target_up = p_target_up;
        target_down = p_target_down;
    }

    void Zero()
    {
        spark_max->GetEncoder().SetPosition(0);
    }

    void Up()
    {
        if (target_up.has_value())
        {
            printf("going to %f\n", target_up.value());
            closed_loop->SetReference(target_up.value() * position_ratio, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    void Down()
    {
        if (target_down.has_value())
        {
            printf("going to %f\n", target_down.value());
            printf("curently at %f\n", spark_max->GetEncoder().GetPosition());
            closed_loop->SetReference(target_down.value() * position_ratio, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    void Pause()
    {
        printf("paused\n");
        closed_loop->SetReference(spark_max->GetEncoder().GetPosition(), rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl);
    }
};
