package org.usfirst.frc.team1601.robot.Systemcommands;

import org.usfirst.frc.team1601.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class VisionSwitcher extends Command {

    public VisionSwitcher() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.vision);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    Robot.vision.setResolution(320, 240, 20);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.vision.switchVisionFeed(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
