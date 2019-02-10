/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX lt1, lt2, lt3, rt1, rt2, rt3;
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new JoystickDrive());
  }

  public DriveTrain(){
      lt1 = new TalonSRX(RobotMap.lt1);
      lt2 = new TalonSRX(RobotMap.lt2);
      lt3 = new TalonSRX(RobotMap.lt3);
      rt1 = new TalonSRX(RobotMap.rt1);
      rt2 = new TalonSRX(RobotMap.rt2);
      rt3 = new TalonSRX(RobotMap.rt3);


      lt1.setInverted(true);
      lt2.setInverted(true);
      lt3.setInverted(true);
      
  }

  public void drive(ControlMode mode, double left, double right){
      lt1.set(mode, left);
      lt2.set(mode, left);
      lt3.set(mode, left);
      rt1.set(mode, left);
      rt2.set(mode, left);
      rt3.set(mode, left);
  }
}
