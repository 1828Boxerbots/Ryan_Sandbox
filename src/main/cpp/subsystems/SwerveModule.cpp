#include "subsystems/SwerveModule.hpp"

#include "Constants.h"

SwerveModule::SwerveModule(unsigned int driveCANId, unsigned int turningCANId, double chassisAngularOffset)
    : m_DrivingController(driveCANId, rev::CANSparkMax::MotorType::kBrushless), m_TurningController(turningCANId, rev::CANSparkMax::MotorType::kBrushless)
{
    // Factory Reset controllers to known state
    m_DrivingController.RestoreFactoryDefaults();
    m_TurningController.RestoreFactoryDefaults();

    // Apply postion and velocity conversion factors to convert from native rotation and RPM
    // to the desired meters and meters per second.
    m_DrivingEncoder.SetPositionConversionFactor(m_DrivingEncoderPositionFactor);
    m_DrivingEncoder.SetVelocityConversionFactor(m_DrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors to convert to radians
    m_TurningEncoder.SetPositionConversionFactor(m_TurningEncoderPositionFactor);
    m_TurningEncoder.SetVelocityConversionFactor(m_TurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    m_TurningEncoder.SetInverted(m_TurningEncoderInverted);

    // Set the PID gains for the driving motor.
    m_DrivingPIDController.SetP(m_DrivingP);
    m_DrivingPIDController.SetI(m_DrivingI);
    m_DrivingPIDController.SetD(m_DrivingD);
    m_DrivingPIDController.SetFF(m_DrivingFF);
    m_DrivingPIDController.SetOutputRange(m_DrivingMinOutput, m_DrivingMaxOutput);

    // Set the PID gains for the turning motor.
    m_TurningPIDController.SetP(m_TurningP);
    m_TurningPIDController.SetI(m_TurningI);
    m_TurningPIDController.SetD(m_TurningD);
    m_TurningPIDController.SetFF(m_TurningFF);
    m_TurningPIDController.SetOutputRange(m_TurningMinOutput, m_TurningMaxOutput);

    m_DrivingController.SetIdleMode(m_DrivingMotorIdleMode);
    m_TurningController.SetIdleMode(m_TurningMotorIdleMode);
    m_DrivingController.SetSmartCurrentLimit(m_DrivingMotorCurrentLimit.value());
    m_TurningController.SetSmartCurrentLimit(m_DrivingMotorCurrentLimit.value());

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_DrivingController.BurnFlash();
    m_TurningController.BurnFlash();

    m_ChassisAngularOffset = chassisAngularOffset;
    m_DesiredState.angle = frc::Rotation2d(units::radian_t{m_TurningEncoder.GetPosition()});
    m_DrivingEncoder.SetPosition(0);
}

SwerveModule::~SwerveModule()
{
    // NOTE: Currently does nothing
}

frc::SwerveModuleState SwerveModule::GetState() const
{
    return { 
        units::meters_per_second_t { m_DrivingEncoder.GetVelocity() }, 
        units::radian_t { m_TurningEncoder.GetPosition() - m_ChassisAngularOffset } 
    };
}

void SwerveModule::SetState(const frc::SwerveModuleState& state)
{
    // Apply chassis angular offset to the desired state.
    frc::SwerveModuleState correctedDesiredState {};
    correctedDesiredState.speed = state.speed;
    correctedDesiredState.angle = state.angle + frc::Rotation2d(units::radian_t{m_ChassisAngularOffset});

    // Optimize the reference state to avoid spinning further than 90 degrees.
    frc::SwerveModuleState optimizedDesiredState {
        frc::SwerveModuleState::Optimize(
            correctedDesiredState, 
            frc::Rotation2d(units::radian_t { m_TurningEncoder.GetPosition() })
        )
    };

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_DrivingPIDController.SetReference((double)optimizedDesiredState.speed, rev::CANSparkMax::ControlType::kVelocity);
    m_TurningPIDController.SetReference(optimizedDesiredState.angle.Radians().value(), rev::CANSparkMax::ControlType::kPosition);

  m_DesiredState = state;
}

frc::SwerveModulePosition SwerveModule::GetPosition() const
{
    return {
        units::meter_t{ m_DrivingEncoder.GetPosition() }, 
        units::radian_t { m_TurningEncoder.GetPosition() - m_ChassisAngularOffset } 
    };
}

void SwerveModule::ResetEncoders()
{
    m_DrivingEncoder.SetPosition(0);
}