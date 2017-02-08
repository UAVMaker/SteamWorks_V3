package org.usfirst.frc.team1601.robot.DriveTraincommands;

import org.usfirst.frc.team1601.robot.Constants;
import org.usfirst.frc.team1601.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class RotateToAngle extends Command {
	double angle;
	double linearSpeed = 0;
	double curveSpeed = 0;
	double toleranceDegrees = 2.0f;
	PIDController turnController;

	public RotateToAngle(double angle) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		this.angle = angle;

		turnController = new PIDController(Constants.ROTATE_ANGLE_P, Constants.DRIVE_STRAIGHT_I,
				Constants.DRIVE_STRAIGHT_D, new PIDSource() {
					PIDSourceType m_sourceType = PIDSourceType.kRate;

					@Override
					public double pidGet() {
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
					public void pidWrite(double output) {
						curveSpeed = output;
					}
				});

		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(toleranceDegrees);
		turnController.setContinuous(true);

		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		turnController.reset();
		turnController.setSetpoint(angle);
		turnController.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveTrain.drive(linearSpeed, curveSpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return turnController.onTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		turnController.disable();
		Robot.driveTrain.stop();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}

}
