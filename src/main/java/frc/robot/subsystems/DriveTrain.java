/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.PIDTemplate;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX lt1, lt2, lt3, rt1, rt2, rt3;
  

  private GearState gear;
  private DoubleSolenoid shifter;
  private DifferentialDrive diffDrive;
  private SpeedControllerGroup leftM;
  private SpeedControllerGroup rightM;

  private Value fast = Value.kForward;
  private Value slow = Value.kReverse;
  private boolean isFast = false;

  public enum GearState {
		SLOW, FAST, UNKNOWN;

		public static Value gearToValue(GearState state) {
			switch (state) {
			case SLOW:
				return Value.kForward;
			case FAST:
				return Value.kReverse;
			case UNKNOWN:
			default:
				return Value.kOff;
			}
		}

		public static GearState valueToGear(Value value) {
			switch (value) {
			case kReverse:
				return GearState.FAST;
			case kOff:
				return GearState.UNKNOWN;
			case kForward:
				return GearState.SLOW;
			default:
				return GearState.UNKNOWN;
			}
		}
	}

 
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

      rt1.setInverted(true);
      rt2.setInverted(true);
      rt3.setInverted(true);   
      
      lt2.follow(lt1);
      lt3.follow(lt1);
      
      rt2.follow(rt1);
      rt3.follow(rt1);

      NeutralMode mode = NeutralMode.Brake;

	  lt1.setNeutralMode(mode);
      lt2.setNeutralMode(mode);
      lt3.setNeutralMode(mode);
      rt1.setNeutralMode(mode);
      rt2.setNeutralMode(mode);
	  rt3.setNeutralMode(mode);
	  
	  

	
	//  gear = GearState.UNKNOWN;
	  
	//   PIDTemplate.configTalon(lt1, false);
	//   PIDTemplate.configTalon(rt1, true);

      shifter = new DoubleSolenoid(RobotMap.shifterUp, RobotMap.shifterDown);
  }

  public void drive(ControlMode mode, double left, double right){
      lt1.set(mode, left);
      rt1.set(mode, right);
  }

  
  public void toggleShift(){
    if(isFast){
      shifter.set(slow);
    }else {
      shifter.set(fast);
	}
	
	isFast = !isFast;
  }
/* public void applyShiftPID(GearState desiredState) {
	if(desiredState == GearState.SLOW) {
		//gear = GearState.SLOW;
		double P_Drive_LOW = 0;
		double I_Drive_LOW = 0;
		double D_Drive_LOW =  0;
		double F_Drive_LOW = 0; 
		double targetSpeed_Drive_LOW = 0;
		PIDTemplate.updatePID(lt1, P_Drive_LOW, I_Drive_LOW, D_Drive_LOW, F_Drive_LOW,targetSpeed_Drive_LOW) ;
		PIDTemplate.updatePID(rt1, P_Drive_LOW, I_Drive_LOW, D_Drive_LOW, F_Drive_LOW,targetSpeed_Drive_LOW) ;
	}else if(desiredState == GearState.FAST) {
	//	gear = FAST;
		double P_Drive_HIGH = 0.35;
		double I_Drive_HIGH = 1.0E-4;
		double D_Drive_HIGH = 0.11;
		double F_Drive_HIGH = 0;
		double targetSpeed_Drive_FAST = 0;
		
		PIDTemplate.updatePID(lt1, P_Drive_HIGH, I_Drive_HIGH, D_Drive_HIGH, F_Drive_HIGH, targetSpeed_Drive_FAST);
		PIDTemplate.updatePID(rt1, P_Drive_HIGH, I_Drive_HIGH, D_Drive_HIGH, F_Drive_HIGH, targetSpeed_Drive_FAST);
	}
}*/
/*
  private void applyShift(GearState desiredState, int attempt) {
		if (attempt > 10) {
			System.err.println("Shifter is not shifting to " + desiredState + " at attempt " + attempt);
			return;
		}
		shifter.set(GearState.gearToValue(desiredState));

		// Timer.delay(.005);
		if (!shifter.get().equals(GearState.gearToValue(desiredState))) { 
			// compressor pressure
			java.util.Timer taskTimer = new java.util.Timer();
			TimerTask task = new TimerTask() {

				@Override
				public void run() {
					applyShift(desiredState, attempt + 1);
				}
			};
			taskTimer.schedule(task, 5); // try to shift again in half a second

		} else {
			System.out.println("shifted to " + desiredState);
			gear = desiredState;
			// PIDConstant constant = (gear == GearState.SLOW ? PIDConstant.slowDrive : PIDConstant.fastDrive);
			// PIDUtil.updatePID(rightT2, constant);
			// PIDUtil.updatePID(leftT2, constant);
		}
  }
  
  public void toggleShift() {
    System.out.println("shifitng");
		if (gear == GearState.SLOW) {
			applyShift(GearState.FAST, 0);
		} else if (gear == GearState.FAST) {
			applyShift(GearState.SLOW, 0);
		} else {
			System.out.println("In unknown state, defaulting to LOW");
			applyShift(GearState.FAST, 0);
		}

	}*/
}
