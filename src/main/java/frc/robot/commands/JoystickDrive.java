/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SubsystemNames;

public class JoystickDrive extends Command {
  DriveTrain drive;
  public JoystickDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drive = (DriveTrain) (Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double throttle =  OI.joyThrottle.getRawAxis(1);
    double turn  = OI.joyTurn.getRawAxis(0);

    drive.drive(ControlMode.PercentOutput, throttle, throttle);


  }

  public double cubeRoot(double val) {
		if (val >= 0) {
			return Math.pow(val, 3 / 2d);
		} else {
			return -Math.pow(-val, 3 / 2d);
		}
	}

	public double sig(double val) {
		return 2 / (1 + Math.pow(Math.E, -7 * Math.pow(val, 3))) - 1;
	}
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drive.drive(ControlMode.PercentOutput, 0, 0);
	}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
