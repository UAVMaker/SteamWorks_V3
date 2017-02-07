package org.usfirst.frc.team1601.robot.DriveTraincommands;

import org.usfirst.frc.team1601.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * This Command is provided to rotate to angle
 * We are using a skid steer driving system
 * which makes us have to rotate one side in the 
 * opposite direction.
 * @author Naresh
 *
 */
public class RotateToAngle extends Command {
	//Create a PID Controller to initiate a PID Loop
	PIDController pid;
	
	//Constructor this is where we initialize the loop.
    public RotateToAngle(double angle) {
    	
    	//Specify the Subsytem
    	requires(Robot.driveTrain);
    	//Specify the loop parameters.
		pid = new PIDController(4, 0, 0, new PIDSource() {
			PIDSourceType m_sourceType = PIDSourceType.kRate;

			@Override
			public double pidGet() {
				//Use the MXP Gyro Angle as the PID Source.
				return Robot.driveTrain.getMXPAngle();
			}

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				m_sourceType = pidSource;
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return m_sourceType;
			}
		}, new PIDOutput() {
			@Override
			public void pidWrite(double curve) {
				//Write our output of the PID Loop to the robot drive Motors
				Robot.driveTrain.drive(0, curve);
			}
		});
		//set the input range
		pid.setInputRange(-180, 180);
		
		//Set the Minimum and Maximum Output
		pid.setOutputRange(-1.0, 1.0);
		
		//Sets the tolerance we have for the loop.
		pid.setAbsoluteTolerance(2);
		
		//Set the angle we want to rotate toward.
		pid.setSetpoint(angle);
		//Add a LiveWindow to make it easier to tune.
		LiveWindow.addActuator("DriveTrain", "RotateToAngle", pid);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.driveTrain.reset();
		pid.reset();
		pid.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return pid.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	pid.disable();
    	Robot.driveTrain.tankDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
