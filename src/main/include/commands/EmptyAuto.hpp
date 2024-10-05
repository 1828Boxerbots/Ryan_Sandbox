#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"

class EmptyAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, EmptyAuto>
{
public:
    EmptyAuto();

private:
};