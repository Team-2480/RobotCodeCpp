#pragma once
#include <rev/SparkMax.h>
#include <optional>

class MotorRegulator
{
private:
    rev::spark::SparkMax *spark_max;
    rev::spark::SparkClosedLoopController *closed_loop;
    double target_speed = 0;
    std::optional<double> last_input = 0;
    std::optional<double> limit_up, limit_down;

    const double throttle_speed;

public:
    MotorRegulator(rev::spark::SparkMax *spark_max, rev::spark::SparkClosedLoopController *closed_loop, double throttle_speed) : spark_max(spark_max), closed_loop(closed_loop), throttle_speed(throttle_speed) {}
    ~MotorRegulator() {}

    void Periodic()
    {
        last_input = spark_max->GetEncoder().GetPosition();
        target_speed = throttle_speed;
        if (limit_down.has_value() && limit_up.has_value() && (limit_down > last_input || last_input > limit_up))
            target_speed = 0;

        closed_loop->SetReference(target_speed, rev::spark::SparkLowLevel::ControlType::kVoltage);
    }
};
