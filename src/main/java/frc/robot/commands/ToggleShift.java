/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SubsystemNames;
import frc.robot.Robot;

public class ToggleShift extends Command {
  DriveTrain drive;
  public ToggleShift() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
     requires(Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
   
   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drive = (DriveTrain) (Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
     drive.toggleShift();
   // System.out.println("i");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   // drive.toggleShift();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
