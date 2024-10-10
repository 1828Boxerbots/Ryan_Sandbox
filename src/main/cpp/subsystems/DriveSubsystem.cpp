#include "subsystems/DriveSubsystem.hpp"

#include "PortManager.hpp"
#include "Utils/SwerveUtils.hpp"

DriveSubsystem::DriveSubsystem()
    : m_FrontLeft {PortManager::Instance().GetCanID("LeftFrontDriveController"),
                 PortManager::Instance().GetCanID("LeftFrontTurningController"),
                 ChassisConstants::frontLeftChassisAngularOffset },
      m_RearLeft {PortManager::Instance().GetCanID("LeftRearDriveController"),
                 PortManager::Instance().GetCanID("LeftRearTurningController"),
                 ChassisConstants::rearLeftChassisAngularOffset },
      m_FrontRight {PortManager::Instance().GetCanID("RightFrontDriveController"),
                 PortManager::Instance().GetCanID("RightFrontTurningController"),
                 ChassisConstants::frontRightChassisAngularOffset },
      m_RearRight {PortManager::Instance().GetCanID("RightRearDriveController"),
                 PortManager::Instance().GetCanID("RightRearTurningController"),
                 ChassisConstants::rearRightChassisAngularOffset }
{
    // NOTE: Initialized in initializer list
}

DriveSubsystem::~DriveSubsystem()
{
    // NOTE: Currently does nothing
}

void DriveSubsystem::Periodic()
{
    // NOTE: Currently does nothing
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           bool rateLimit) {
  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          fabs((double)(DriveConstants::directionSlewRate / m_currentTranslationMag));
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_MagLimiter.Calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag >
          1e-4) {  // some small number to avoid floating-point errors with
                   // equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_MagLimiter.Calculate(0.0);
      } else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_MagLimiter.Calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_MagLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_RotLimiter.Calculate(rot.value());

  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::maxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::maxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * DriveConstants::maxAngularSpeed;

  auto states = m_DriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    m_Gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  m_DriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::maxSpeed);

  auto [fl, fr, bl, br] = states;

  m_FrontLeft.SetState(fl);
  m_FrontRight.SetState(fr);
  m_RearLeft.SetState(bl);
  m_RearRight.SetState(br);
}

void DriveSubsystem::SetX() {
  m_FrontLeft.SetState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_FrontRight.SetState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_RearLeft.SetState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_RearRight.SetState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_DriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::maxSpeed);
  m_FrontLeft.SetState(desiredStates[0]);
  m_FrontRight.SetState(desiredStates[1]);
  m_RearLeft.SetState(desiredStates[2]);
  m_RearRight.SetState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_FrontLeft.ResetEncoders();
  m_RearLeft.ResetEncoders();
  m_FrontRight.ResetEncoders();
  m_RearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return frc::Rotation2d(
             units::radian_t{m_Gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { m_Gyro.Reset(); }

double DriveSubsystem::GetTurnRate() {
  return -m_Gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}