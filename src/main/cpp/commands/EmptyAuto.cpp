#include "commands/EmptyAuto.hpp"

EmptyAuto::EmptyAuto()
{
    // NOTE: Does nothing. Meant to act as a void command that is used to satisfy
    //       the return value of the GetAutomousCommand function in the robot
    //       container. Use this when robot should do nothing during auto.
}