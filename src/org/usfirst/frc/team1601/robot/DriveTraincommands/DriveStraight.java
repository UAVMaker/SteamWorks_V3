package org.usfirst.frc.team1601.robot.DriveTraincommands;

import org.usfirst.frc.team1601.robot.Constants;
import org.usfirst.frc.team1601.robot.Robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command {
	CANTalon leftFront, leftRear, rightFront, rightRear;
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	double output;
	double targetRotations = 50;

	public DriveStraight() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		leftFront = Robot.driveTrain.leftFront();
		leftRear = Robot.driveTrain.leftRear();
		rightFront = Robot.driveTrain.rightFront();
		rightRear = Robot.driveTrain.rightRear();

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetEncoder();
		leftFront.changeControlMode(TalonControlMode.Position);
		leftRear.changeControlMode(TalonControlMode.Follower);
		rightFront.changeControlMode(TalonControlMode.Follower);
		rightRear.changeControlMode(TalonControlMode.Follower);
		leftFront.setAllowableClosedLoopErr(0);

		leftFront.setProfile(0);
		leftFront.setPID(Constants.DRIVE_STRAIGHT_D, Constants.DRIVE_STRAIGHT_I, Constants.DRIVE_STRAIGHT_D);
		leftFront.setCloseLoopRampRate(Constants.DRIVE_STRAIGHT_RAMP_RATE);

		leftFront.set(targetRotations);
		leftFront.enable();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		output = leftFront.getOutputVoltage() / leftFront.getBusVoltage();
		_sb.append("\toutput:");
		_sb.append(output);
		_sb.append("\terrNative:");
		_sb.append(leftFront.getClosedLoopError());
		_sb.append("\ttrg:");
		_sb.append(50);

		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		leftFront.disable();
		leftFront.changeControlMode(TalonControlMode.PercentVbus);
		leftRear.changeControlMode(TalonControlMode.PercentVbus);
		rightFront.changeControlMode(TalonControlMode.PercentVbus);
		rightRear.changeControlMode(TalonControlMode.PercentVbus);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
