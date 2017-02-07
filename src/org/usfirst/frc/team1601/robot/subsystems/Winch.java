package org.usfirst.frc.team1601.robot.subsystems;

import org.usfirst.frc.team1601.robot.RobotMap;
import org.usfirst.frc.team1601.robot.Climbercommands.WinchOff;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class Winch extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	Spark winch = new Spark(RobotMap.winchMotor);
	
	public Winch(){
		super();
		LiveWindow.addActuator("Winch", "Winch Motor", winch);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new WinchOff());
    }
    public void winchForward(){
    	winch.set(1);
    }
    public void winchBackward(){
    	winch.set(-1);
    }
    public void stop(){
    	winch.set(0);
    }
}

