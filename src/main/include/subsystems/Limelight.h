#pragma once

#include "networktables/NetworkTable.h"
#include <memory>

class Limelight{
public:
    Limelight();

    double getTx();

    bool hasTarget();

private:
    std::shared_ptr<nt::NetworkTable> limelightTable;
};
