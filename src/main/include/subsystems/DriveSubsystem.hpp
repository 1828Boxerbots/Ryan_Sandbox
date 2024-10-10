#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/ADIS16470_IMU.h>

#include <vector>

#include "Constants.h"
#include "subsystems/SwerveModule.hpp"

class DriveSubsystem : public frc2::SubsystemBase
{
public:
    /**
     * Constructs a new DriveSubsystem object.
    */
    DriveSubsystem();

    /**
     * Destroys the DriveSubsystem object.
    */
    ~DriveSubsystem();

    /**
     * Called periodically (when the command scheduler is run)
    */
    void Periodic() override;

    /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, 
             units::radians_per_second_t rot,
             bool fieldRelative, 
             bool rateLimit);

    /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

private:
    SwerveModule m_FrontLeft;
    SwerveModule m_RearLeft;
    SwerveModule m_FrontRight;
    SwerveModule m_RearRight;

    // The gyro sensor
    frc::ADIS16470_IMU m_Gyro;

    frc::SwerveDriveKinematics<4> m_DriveKinematics{
      frc::Translation2d{ChassisConstants::wheelBase / 2,
                         ChassisConstants::trackWidth / 2},
      frc::Translation2d{ChassisConstants::wheelBase / 2,
                         -ChassisConstants::trackWidth / 2},
      frc::Translation2d{-ChassisConstants::wheelBase / 2,
                         ChassisConstants::trackWidth / 2},
      frc::Translation2d{-ChassisConstants::wheelBase / 2,
                         -ChassisConstants::trackWidth / 2}};

    // Slew rate filter variables for controlling lateral acceleration
    double m_currentRotation = 0.0;
    double m_currentTranslationDir = 0.0;
    double m_currentTranslationMag = 0.0;

    frc::SlewRateLimiter<units::scalar> m_MagLimiter{
        DriveConstants::magnitudeSlewRate / 1_s};
    frc::SlewRateLimiter<units::scalar> m_RotLimiter{
        DriveConstants::rotationalSlewRate / 1_s};
    double m_prevTime = wpi::Now() * 1e-6;
};