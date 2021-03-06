/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SubsystemNames;

public class JoystickDrive extends Command {
  DriveTrain drive;
  DifferentialDrive diffDrive;
  public JoystickDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
   // System.out.println("Joystick" + Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
    requires(Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drive = (DriveTrain) Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN);
    
  }

  // Called repeatedly when this Command is scheduled to run
  /**set motor power values based on joystick throttle and turn  */
  @Override
  protected void execute() {
    double throttle =  OI.joyThrottle.getRawAxis(1);
    throttle = Math.abs(throttle) < 0.1 ? 0 : throttle;
    double turn  = OI.joyTurn.getRawAxis(0);
    turn  = Math.abs(turn) < 0.1 ? 0 : turn;


    drive.drive(ControlMode.PercentOutput, sig(throttle - cubeRoot(turn)), sig(throttle + cubeRoot(turn)));
    
   SmartDashboard.putNumber("throttle", throttle);

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
    drive.drive(ControlMode.PercentOutput,0,0);
  }
}
