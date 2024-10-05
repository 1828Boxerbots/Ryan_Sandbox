#pragma once

#include <string>
#include <map>

class PortManager
{
    public:
        PortManager(const PortManager& obj) = delete;
        PortManager& operator=(const PortManager& obj) = delete;

        ~PortManager();

        static PortManager& Instance();

        int GetCanID(std::string IDName);

    private:
        PortManager();

        std::map<std::string, int> CANPorts {
            {"LeftFrontDriveController", -1},
            {"LeftFrontTurnController", -1},
            {"LeftFrontTurnEncoder", -1},
            {"LeftRearDriveController", -1},
            {"LeftRearTurnController", -1},
            {"LeftRearTurnEncoder", -1},
            {"RightFrontDriveController", -1},
            {"RightFrontTurnController", -1},
            {"RightFrontTurnEncoder", -1},
            {"RightRearDriveController", -1},
            {"RightRearTurnController", -1},
            {"RightRearTurnEncoder", -1}
        };
};