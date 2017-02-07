package org.usfirst.frc.team1601.robot.Gearcommands;

import org.usfirst.frc.team1601.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class OpenClaw extends TimedCommand {

    public OpenClaw() {
        super(1);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.claw);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.claw.intake();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Called once after timeout
    protected void end() {
    	Robot.claw.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
