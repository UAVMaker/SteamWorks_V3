package org.usfirst.frc.team1601.robot;

import org.usfirst.frc.team1601.robot.Climbercommands.WinchOn;
import org.usfirst.frc.team1601.robot.DriveTraincommands.DriveStraight;
import org.usfirst.frc.team1601.robot.DriveTraincommands.RotateToAngle;
import org.usfirst.frc.team1601.robot.Gearcommands.CloseClaw;
import org.usfirst.frc.team1601.robot.Gearcommands.OpenClaw;
import org.usfirst.frc.team1601.robot.Hoppercommands.HopperExhaust;
import org.usfirst.frc.team1601.robot.Hoppercommands.HopperIntake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	public Joystick driverL = new Joystick(RobotMap.leftDriverJoystick);
	public Joystick driverR = new Joystick(RobotMap.rightDriverJoystick);
	public Joystick operator = new Joystick(RobotMap.opertorJoystick);
	public JoystickButton aButton, bButton, xButton, yButton, leftTrigger, rightTrigger;

	public OI(){
		//SmartDashboard Data
		SmartDashboard.putData("RotateToAngle", new RotateToAngle(90));
		SmartDashboard.putData("DriveStraight", new DriveStraight(10));
		
		//Buttons
		aButton = new JoystickButton(operator, 1);
		bButton = new JoystickButton(operator, 2);
		xButton = new JoystickButton(operator, 3);
		yButton = new JoystickButton(operator, 4);
		leftTrigger = new JoystickButton(operator, 5);
		rightTrigger = new JoystickButton(operator, 6);
		
		aButton.whenPressed(new RotateToAngle(90));
		bButton.whenPressed(new OpenClaw());
		xButton.whenPressed(new CloseClaw());
		yButton.whenPressed(new WinchOn());
		leftTrigger.whenPressed(new HopperIntake());
		rightTrigger.whenPressed(new HopperExhaust());
		
	}
	public Joystick getOperatorJoystick() {
		return operator;
	}

	public Joystick getLeftDriverJoystick() {
		return driverL;
	}

	public Joystick getRightDriverJoystick() {
		return driverR;
	}
	
	

}
