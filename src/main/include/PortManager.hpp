#pragma once

#include <string>
#include <map>

/**
 * @brief Class that is used to manage and retreive the various hardware ports
 * and IDs across the roboot.
*/
class PortManager
{
    public:
        // Delete copy constructor and operator to maintain singleton
        PortManager(const PortManager& obj) = delete;
        PortManager& operator=(const PortManager& obj) = delete;

        /**
         * @brief Destroys the PortManager object.
        */
        ~PortManager();

        /**
         * @brief Retrieves the singleton instance of the PortManager. If no
         * instance exists, then it creates one.
        */
        static PortManager& Instance();

        /**
         * @brief Takes the name of a CAN device and retrieves the requested
         * ID. If the ID is invalid, it will display a warning message in the
         * driver station console.
         * 
         * @param IDName This is the name of the can id to retrieve.
         * @return This returns the integer of the requested ID.
        */
        unsigned int GetCanID(std::string IDName);

    private:
        /**
         * @brief Constructs a new PortManager object.
        */
        PortManager();

        std::map<std::string, int> m_CANPorts {
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