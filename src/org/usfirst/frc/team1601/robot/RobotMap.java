package org.usfirst.frc.team1601.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.


    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;

    
    //Joysticks
    public  static int leftDriverJoystick =1,
    				   rightDriverJoystick =2,
    				   opertorJoystick =3;
    //Drive Train Motors
    public static int leftFront =1, 
    				  leftRight = 2,
    				  rightFront = 3,
    				  rightRear = 4;
    public static double speedLow =.3, 
    					 speedMed =.5, 
    					 speedHigh =1.0;
    //Claw (PWM SPARK Controller)
    public static int hopperMotor = 0;
    
    //Winch (PWM SPARK Controller)
    public static int winchMotor= 1;
	
    //Gear Hopper
    public static int gearActExtend = 1; 
    public static int gearActRetract =2;
    
    //Gear Pusher
    public static int gearPusherExtend =3;
    public static int gearPusherRetract = 4;
    
    
	}
