#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>
#include <units/current.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>

#include <numbers>

/**
 * Class that holds the individual state of a single swerve module.
*/
class SwerveModule
{
    public:
        /**
         * Constructs a new SwerveModule object.
         * 
         * @param driveCANId The ID used to talk with the CAN device used
         * for driving the module.
         * 
         * @param turningCANId The ID used to talk with the CAN device used
         * for turning the module.
         * 
         * @param chassisAngularOffset 
        */
        SwerveModule(unsigned int driveCANId, unsigned int turningCANId, double chassisAngularOffset);

        /**
         * Destroys the SwerveModule object.
        */
        ~SwerveModule();

        /**
         * Retrieves the current state of the module.
         * 
         * @return The current state of the module.
        */
        frc::SwerveModuleState GetState() const;

        /**
         * Sets the desired state for the module.
         * 
         * @param state The desired state the module should have.
        */
        void SetState(const frc::SwerveModuleState& state);

        /**
         * Retrieves the current position of the module.
         * 
         * @return The current position of the module.
        */
        frc::SwerveModulePosition GetPosition() const;

        /**
         * Zero all of the encoders in the module.
        */
        void ResetEncoders();

    private:
        // Motor Controllers
        rev::CANSparkMax m_DrivingController;
        rev::CANSparkMax m_TurningController;

        // Encoders
        rev::SparkRelativeEncoder m_DrivingEncoder = m_DrivingController.GetEncoder();
        rev::SparkAbsoluteEncoder m_TurningEncoder = m_TurningController.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

        // PID Controllers
        rev::SparkPIDController m_DrivingPIDController = m_DrivingController.GetPIDController();
        rev::SparkPIDController m_TurningPIDController = m_TurningController.GetPIDController();

        double m_ChassisAngularOffset = 0.0;
        frc::SwerveModuleState m_DesiredState { units::meters_per_second_t { 0.0 }, frc::Rotation2d() };

        // --------------------------------------------------------------------
        //                         Constants
        // --------------------------------------------------------------------
        // Drive Gear teeth counts
        const int m_FirstStageSpurTeeth = 22;
        const int m_WheelBevelTeeth = 45;
        const int m_DrivingMotorPinionTeeth = 14;
        const int m_DriveBevelPinionTeeth = 15;

        // Calculations required for driving motor conversion factors and feed forward
        const units::meter_t m_WheelDiameter = 0.0762_m;
        const units::meter_t m_WheelCircumference = m_WheelDiameter * std::numbers::pi;

        const double m_DrivingMotorReduction = (m_WheelBevelTeeth * m_FirstStageSpurTeeth) / (m_DrivingMotorPinionTeeth * m_DriveBevelPinionTeeth);
        const double m_DrivingMotorFreeSpeedRps = 5676.0 / 60;  // NEO free speed is 5676 RPM
        const double m_DriveWheelFreeSpeedRps = (m_DrivingMotorFreeSpeedRps * m_WheelCircumference.value()) / m_DrivingMotorReduction;

        // Drive Conversion Constants
        const double m_SecondsInAMinute = 60.0;
        const double m_DrivingEncoderPositionFactor = (m_WheelDiameter.value() * std::numbers::pi) / m_DrivingMotorReduction;  // meters
        const double m_DrivingEncoderVelocityFactor = m_DrivingEncoderPositionFactor/ m_SecondsInAMinute;  // meters per second

        // Turning Conversion Constants
        const double m_TurningEncoderPositionFactor = (2 * std::numbers::pi);  // radians
        const double m_TurningEncoderVelocityFactor = m_TurningEncoderPositionFactor/ m_SecondsInAMinute;  // radians per second

        bool m_TurningEncoderInverted = true;

        // PID Constants
        const double m_DrivingP = 0.04;
        const double m_DrivingI = 0;
        const double m_DrivingD = 0;
        const double m_DrivingFF = (1 / m_DriveWheelFreeSpeedRps);
        const double m_DrivingMinOutput = -1;
        const double m_DrivingMaxOutput = 1;

        const double m_TurningP = 1;
        const double m_TurningI = 0;
        const double m_TurningD = 0;
        const double m_TurningFF = 0;
        const double m_TurningMinOutput = -1;
        const double m_TurningMaxOutput = 1;

        const rev::CANSparkMax::IdleMode m_DrivingMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;
        const rev::CANSparkMax::IdleMode m_TurningMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

        // Set amperage limits to prevent brownouts
        const units::ampere_t m_DrivingMotorCurrentLimit = 50_A;
        const units::ampere_t m_TurningMotorCurrentLimit = 20_A;
        // --------------------------------------------------------------------
};