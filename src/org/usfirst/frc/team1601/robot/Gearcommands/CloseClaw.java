package org.usfirst.frc.team1601.robot.Gearcommands;

import org.usfirst.frc.team1601.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class CloseClaw extends TimedCommand {

	public CloseClaw() {
		super(1);
		requires(Robot.claw);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.claw.exhaust();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.claw.stop();
	}

}
