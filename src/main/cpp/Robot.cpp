/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  /* choose which cabling method for Pigeon */
  pidgey = new PigeonIMU(9); /* Pigeon is on CANBus (powered from ~12V, and
                                      has a device ID of zero */

  // Right side wheels are backwards
  // m_FrontRight->SetInverted(true);
  // m_RearRight->SetInverted(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  // kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

/** @return 10% deadband */
double Db(double axisVal) {
  if (axisVal < -0.10) return axisVal;
  if (axisVal > +0.10) return axisVal;
  return 0;
}
/** @param value to cap.
 * @param peak positive double representing the maximum (peak) value.
 * @return a capped value.
 */
double Cap(double value, double peak) {
  if (value < -peak) return -peak;
  if (value > +peak) return +peak;
  return value;
}
/**
 * As a simple trick, lets take the spare talon and use the web-based
 * config to easily change the gains we use for the Pigeon servo.
 * The talon isn't being used for closed-loop, just use it as a convenient
 * storage for gains.
 */
void UpdatGains() {}
/**
 * Given the robot forward throttle and ratio, return the max
 * corrective turning throttle to adjust for heading.  This is
 * a simple method of avoiding using different gains for
 * low speed, high speed, and no-speed (zero turns).
 */
double MaxCorrection(double forwardThrot, double scalor) {
  /* make it positive */
  if (forwardThrot < 0) {
    forwardThrot = -forwardThrot;
  }
  /* max correction is the current forward throttle scaled down */
  forwardThrot *= scalor;
  /* ensure caller is allowed at least 10% throttle,
   * regardless of forward throttle */
  if (forwardThrot < 0.10) return 0.10;
  return forwardThrot;
}

void Robot::TeleopInit() {
  const int kTimeoutMs = 30;
  pidgey->SetFusedHeading(
      0.0, kTimeoutMs); /* reset heading, angle measurement wraps at plus/minus
                           23,040 degrees (64 rotations) */
  goStraight = GoStraightOff;
}

void Robot::TeleopPeriodic() {
		/* some temps for Pigeon API */
		PigeonIMU::GeneralStatus genStatus;
		double xyz_dps[3];
		/* grab some input data from Pigeon and gamepad*/
		pidgey->GetGeneralStatus(genStatus);
		pidgey->GetRawGyro(xyz_dps);

		PigeonIMU::FusionStatus *stat = new PigeonIMU::FusionStatus();
		pidgey->GetFusedHeading(*stat);
		double currentAngle = stat->heading;
		bool angleIsGood = (pidgey->GetState() == PigeonIMU::Ready) ? true : false;
		double currentAngularRate = xyz_dps[2];
		/* get input from gamepad */
		bool userWantsGoStraight = xbox0.GetRawButton(5); /* top left shoulder button */
		double forwardThrottle = xbox0.GetY(frc::GenericHID::JoystickHand::kRightHand) * -1.0; /* sign so that positive is forward */
		double turnThrottle = xbox0.GetX(frc::GenericHID::JoystickHand::kRightHand) * -1.0; /* sign so that positive means turn left */
		/* deadbands so centering joysticks always results in zero output */
		forwardThrottle = Db(forwardThrottle);
		turnThrottle = Db(turnThrottle);
		/* simple state machine to update our goStraight selection */
		switch (goStraight) {

			/* go straight is off, better check gamepad to see if we should enable the feature */
			case GoStraightOff:
				if (userWantsGoStraight == false) {
					/* nothing to do */
				} else if (angleIsGood == false) {
					/* user wants to servo but Pigeon isn't connected? */
					goStraight = GoStraightSameThrottle; /* just apply same throttle to both sides */
				} else {
					/* user wants to servo, save the current heading so we know where to servo to. */
					goStraight = GoStraightWithPidgeon;
					targetAngle = currentAngle;
				}
				break;
	
			/* we are servo-ing heading with Pigeon */
			case GoStraightWithPidgeon:
				if (userWantsGoStraight == false) {
					goStraight = GoStraightOff; /* user let go, turn off the feature */
				} else if (angleIsGood == false) {
					goStraight = GoStraightSameThrottle; /* we were servoing with pidgy, but we lost connection?  Check wiring and deviceID setup */
				} else {
					/* user still wants to drive straight, keep doing it */
				}
				break;
	
			/* we are simply applying the same throttle to both sides, apparently Pigeon is not connected */
			case GoStraightSameThrottle:
				if (userWantsGoStraight == false) {
					goStraight = GoStraightOff; /* user let go, turn off the feature */
				} else {
					/* user still wants to drive straight, keep doing it */
				}
				break;
		}

		/* if we can servo with IMU, do the math here */
		if (goStraight == GoStraightWithPidgeon) {
			/* very simple Proportional and Derivative (PD) loop with a cap,
			 * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
			turnThrottle = (targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
			/* the max correction is the forward throttle times a scalar,
			 * This can be done a number of ways but basically only apply small turning correction when we are moving slow
			 * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
			double maxThrot = MaxCorrection(forwardThrottle, kMaxCorrectionRatio);
			turnThrottle = Cap(turnThrottle, maxThrot);
		} else if (goStraight == GoStraightSameThrottle) {
			/* clear the turn throttle, just apply same throttle to both sides */
			turnThrottle = 0;
		} else {
			/* do nothing */
		}

		/* positive turnThrottle means turn to the left, this can be replaced with ArcadeDrive object, or teams drivetrain object */
		float left = forwardThrottle - turnThrottle;
		float right = forwardThrottle + turnThrottle;
		left = Cap(left, 1.0);
		right = Cap(right, 1.0);

		/* my right side motors need to drive negative to move robot forward */
		m_FrontLeft->Set(ControlMode::PercentOutput, left);
		m_RearLeft->Set(ControlMode::PercentOutput, left);
		m_FrontRight->Set(ControlMode::PercentOutput, -1. * right);
		m_RearRight->Set(ControlMode::PercentOutput, -1. * right);

		/* some printing for easy debugging */
		if (++_printLoops > 50) {
			_printLoops = 0;
			printf("------------------------------------------\n");
			printf("error: %f\n", targetAngle - currentAngle);
			printf("angle: %f\n", currentAngle);
			printf("rate: %f\n", currentAngularRate);
			printf("noMotionBiasCount: %i\n", genStatus.noMotionBiasCount);
			printf("tempCompensationCount: %i\n",
					genStatus.tempCompensationCount);
			printf("%s\n", angleIsGood ? "Angle is good" : "Angle is NOT GOOD");
			printf("------------------------------------------\n");
		}

		/* press btn 6, top right shoulder, to apply gains from webdash.  This can
		 * be replaced with your favorite means of changing gains. */
		if (xbox0.GetRawButton(6)) {
			UpdatGains();
		}



/*
  double tempX, tempY, tempRotate;
  double x, y, rotate;

  // Read the Xbox Controller inputs
  tempX = -(xbox0.GetX(
      frc::GenericHID::JoystickHand::kRightHand));  // XBox Righthand joystick
  tempY = (xbox0.GetY(
      frc::GenericHID::JoystickHand::kRightHand));  // XBox Righthand Joystick
  tempRotate = -(xbox0.GetX(
      frc::GenericHID::JoystickHand::kLeftHand));  // XBox Lefthand  joystick
  // Cube them to shape the curve while maintaining the sign
  x = tempX * tempX * tempX;                      // pow(tempX,3);
  y = tempY * tempY * tempY;                      // pow(tempY,3);
  rotate = tempRotate * tempRotate * tempRotate;  // pow(tempRotate,3)
*/
  /* Use the joystick X axis for lateral movement, Y axis for forward
   * movement, and Z axis for rotation.
   */
 // m_robotDrive.DriveCartesian(tempX, tempY, rotate);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
