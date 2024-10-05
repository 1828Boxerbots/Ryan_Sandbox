#include "PortManager.hpp"

#include <frc/Errors.h>

PortManager::PortManager()
{
    // NOTE: Currently does nothing
}

PortManager::~PortManager()
{
    // NOTE: Currently does nothing
}

PortManager& PortManager::Instance()
{
    static PortManager instance;
    return instance;
}

int PortManager::GetCanID(std::string IDName)
{
    int ID = CANPorts[IDName];

    if (ID < 0)
        FRC_ReportError(frc::warn::Warning, "Requested CAN ID was {}. CAN IDs can not be less than 0.", ID);

    return ID;
}