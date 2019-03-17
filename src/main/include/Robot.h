/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>


class Robot : public frc::TimedRobot {
  	/** state for tracking whats controlling the drivetrain */
	enum {
		GoStraightOff, GoStraightWithPidgeon, GoStraightSameThrottle
	} goStraight = GoStraightOff;

	/* Some gains for heading servo,
	 * these were tweaked by using the web-based config (CAN Talon) and
	 * pressing gamepad button 6 to load them.
	 */
	double kPgain = 0.04; /* percent throttle per degree of error */
	double kDgain = 0.0004; /* percent throttle per angular velocity dps */
	double kMaxCorrectionRatio = 0.30; /* cap corrective turning throttle to 30 percent of forward throttle */
	/** holds the current angle to servo to */
	double targetAngle = 0;
	/** count loops to print every second or so */
	int _printLoops = 0;

 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  static constexpr int kFrontLeftChannel  = 1;
  static constexpr int kRearLeftChannel   = 2;
  static constexpr int kFrontRightChannel = 3;
  static constexpr int kRearRightChannel  = 4;

  static constexpr int kJoystickChannel = 0;
  
  PigeonIMU *pidgey;

  WPI_TalonSRX *m_FrontLeft  = new WPI_TalonSRX(kFrontLeftChannel);
  WPI_TalonSRX *m_RearLeft   = new WPI_TalonSRX(kRearLeftChannel);
  WPI_TalonSRX *m_FrontRight = new WPI_TalonSRX(kFrontRightChannel);
  WPI_TalonSRX *m_RearRight  = new WPI_TalonSRX(kRearRightChannel);

  frc::MecanumDrive m_robotDrive{*m_FrontLeft, *m_RearLeft, *m_FrontRight, *m_RearRight};

  frc::XboxController xbox0{kJoystickChannel};

};
