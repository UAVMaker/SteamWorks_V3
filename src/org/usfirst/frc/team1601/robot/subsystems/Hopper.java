package org.usfirst.frc.team1601.robot.subsystems;

import org.usfirst.frc.team1601.robot.RobotMap;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class Hopper extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	Spark hopperMotor = new Spark(RobotMap.hopperMotor);
	
	public Hopper(){
		super();
		
	LiveWindow.addActuator("Hopper", "Hopper Motor", hopperMotor);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void intake(){
    	hopperMotor.set(-1);
    }
    
    public void exhaust(){
    	hopperMotor.set(1);
    }
    public void stop(){
    	hopperMotor.set(0);
    }
    
}

