package org.usfirst.frc.team1601.robot.subsystems;

import org.usfirst.frc.team1601.robot.RobotMap;
import org.usfirst.frc.team1601.robot.Gearcommands.GearHolderDefault;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearHolder extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	DoubleSolenoid gear = new DoubleSolenoid(RobotMap.gearActExtend, RobotMap.gearActRetract);
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new GearHolderDefault());
    }
    public  void extendGear(){
    	gear.set(Value.kForward);
    }
    public void retractGear(){
    	gear.set(Value.kReverse);
    }
    public void neutral(){
    	gear.set(Value.kOff);
    }
}

