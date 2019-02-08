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
#include <ctre/phoenix.h>

class Robot : public frc::TimedRobot {
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

  WPI_TalonSRX *m_FrontLeft  = new WPI_TalonSRX(kFrontLeftChannel);
  WPI_TalonSRX *m_RearLeft   = new WPI_TalonSRX(kRearLeftChannel);
  WPI_TalonSRX *m_FrontRight = new WPI_TalonSRX(kFrontRightChannel);
  WPI_TalonSRX *m_RearRight  = new WPI_TalonSRX(kRearRightChannel);


  frc::MecanumDrive m_robotDrive{*m_FrontLeft, *m_RearLeft, *m_FrontRight, *m_RearRight};

  frc::Joystick m_stick{kJoystickChannel};

};
