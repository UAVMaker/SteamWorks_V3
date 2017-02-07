package org.usfirst.frc.team1601.robot.subsystems;

import org.usfirst.frc.team1601.robot.RobotMap;
import org.usfirst.frc.team1601.robot.DriveTraincommands.DriveWithJoysticks;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXL362.Axes;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	/*
	 * Here we will Initialize the Hardware
	 */

	CANTalon leftFront = new CANTalon(RobotMap.leftFront);
	CANTalon leftRear = new CANTalon(RobotMap.leftRight);
	CANTalon rightFront = new CANTalon(RobotMap.rightFront);
	CANTalon rightRear = new CANTalon(RobotMap.rightRear);
	CANTalon[] driveMotors = { leftFront, leftRear, rightFront, rightRear };
	RobotDrive drive = new RobotDrive(leftFront, rightFront, leftRear, rightRear);

	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	ADXL362 accel = new ADXL362(Range.k16G);
	AHRS mxp = new AHRS(SPI.Port.kMXP);
	private int errorTolerance = 2;

	double last_world_linear_accel_x;
	double last_world_linear_accel_y;
	final static double kCollisionThreshold_DeltaG = 0.5f;

	public DriveTrain() {
		super();

		LiveWindow.addActuator("DriveTrain", "Front Left Motor", leftFront);
		LiveWindow.addActuator("DriveTrain", "Front Right", rightFront);
		LiveWindow.addActuator("DriveTrain", "Rear Left", leftRear);
		LiveWindow.addActuator("DriveTrain", "Rear Right", rightRear);
		LiveWindow.addSensor("DriveTrain", "ADXRS450_Gyro", gyro);
		LiveWindow.addSensor("DriveTrain", "NavX ", mxp);
		LiveWindow.addSensor("DriveTrain", "ADXL362_Accelerometer", accel);

		// TODO: Add Encoders Later On.
		leftFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftFront.configEncoderCodesPerRev(1440); // 360 * 4

		this.setFrameRateUpdate(10);

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveWithJoysticks());
	}

	public void drive(double linear, double curve) {
		if (driveMotors[0].getControlMode() != TalonControlMode.PercentVbus) {
			this.changeAllControlModes(TalonControlMode.PercentVbus);
		}

		drive.drive(linear, curve);
	}

	public void tankDrive(double left, double right) {
		changeAllControlModes(TalonControlMode.PercentVbus);
		driveMotors[0].set(left);
		driveMotors[1].set(-left);
		driveMotors[2].set(right);
		driveMotors[3].set(-right);

	}

	public void log() {
		SmartDashboard.putNumber("Gyro Heading", gyro.getAngle());
		SmartDashboard.putNumber("Accelerometer", accel.getX());
		SmartDashboard.putNumber("NavX Gyro", mxp.getAngle());
		SmartDashboard.putNumber("NavX Accelerometer", mxp.getRawAccelX());
		SmartDashboard.putNumber("Encoder Value", driveMotors[0].get());
		SmartDashboard.putBoolean("Collsion Detected", collisionDetection());
		dataMonitor();
	}

	public void driveStraight(double rotations) {
		changeAllControlModes(TalonControlMode.Position);
		this.setAllowableError(errorTolerance);
		this.setPIDProfile(0);
		leftFront.set(rotations);

	}

	/**
	 * Checks to see if we have reached our setpoint.
	 * 
	 * @return
	 */
	public boolean isTargetReached() {
		if (Math.abs(leftFront.getClosedLoopError()) <= errorTolerance) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Reset the Encoder Values
	 */
	public void resetEncoder() {
		driveMotors[0].setPosition(0);
		driveMotors[0].setEncPosition(0);
	}

	/**
	 * Change the Control Mode of all the DriveTrain Talons.
	 * 
	 * @param mode
	 */
	public void changeAllControlModes(TalonControlMode mode) {
		for (int i = 0; i <= driveMotors.length; i++) {
			if (driveMotors[i].getControlMode() != mode) {
				driveMotors[i].changeControlMode(mode);
			}
		}
	}

	/**
	 * Set the allowable integer value the PID Loop can be off by.
	 * 
	 * @param error
	 */
	public void setAllowableError(int error) {
		leftFront.setAllowableClosedLoopErr(error);
		this.errorTolerance = error;
	}

	public void setPIDProfile(int profile) {
		switch (profile) {
		case 0:
			driveMotors[0].setProfile(0);
			driveMotors[0].setPID(3, 0, 0);
			break;
		case 1:
			driveMotors[0].setProfile(1);
			driveMotors[0].setPID(3, 0, 0);
			break;
		default:
			System.out.println("Invalid Number.");
			break;

		}
	}

	/**
	 * Returns the Gyro Angle.
	 * 
	 * @return
	 */
	public double getGyroAngle() {
		return gyro.getAngle();
	}

	/**
	 * Returns the gyro reading from the NAVX Board
	 * 
	 * @return
	 */
	public double getMXPAngle() {
		return mxp.getAngle();
	}

	/**
	 * Returns the Value of the Accelerometer reading from the accelerometer.
	 * Must input which Axes you would like.
	 * 
	 * @param axis
	 * @return
	 */
	public double getAcceleration(Axes axis) {
		return accel.getAcceleration(axis);
	}

	/**
	 * Disables the driveTrain.
	 */
	public void stop() {
		tankDrive(0, 0);
	}

	/**
	 * We are comparing the output voltage of the speed controller versuses the
	 * voltage running across the CAN Bus.
	 * 
	 * @return
	 */
	public double getDriveTrainSpeed() {
		return (driveMotors[0].getOutputVoltage() / driveMotors[0].getBusVoltage());
	}

	/**
	 * We should reset the sensors so we can use in PID Loops
	 */
	public void reset() {
		gyro.reset();
		mxp.reset();

	}

	/**
	 * Here we are getting our drive Motors if we need to use the motors
	 * elsewhere.
	 * 
	 * @return
	 */
	public CANTalon[] getAllDriveMotors() {
		return driveMotors;
	}

	/**
	 * Enables and disables the brake mode on the drive Motor Talons
	 * 
	 * @param brake
	 */
	public void setBrakeMode(boolean brake) {
		for (int i = 0; i <= driveMotors.length; i++) {
			if (driveMotors[i].getBrakeEnableDuringNeutral() != brake) {
				driveMotors[i].enableBrakeMode(brake);
			}
		}

	}

	/**
	 * Sets the update frequency for the Feedback from the sensor. Updating to
	 * fast is bad for performance. Ex. 10 is the update period we are updating
	 * at 100HZ
	 * 
	 * @param period
	 */
	public void setFrameRateUpdate(int period) {
		driveMotors[0].setStatusFrameRateMs(StatusFrameRate.Feedback, period);
	}

	public boolean collisionDetection() {
		boolean collisionDetected = false;

		double curr_world_linear_accel_x = mxp.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = mxp.getWorldLinearAccelY();
		double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;

		if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
				|| (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
			collisionDetected = true;
		}

		return collisionDetected;
	}

	public void dataMonitor() {
		/* Display 6-axis Processed Angle Data */
		SmartDashboard.putBoolean("IMU_Connected", mxp.isConnected());
		SmartDashboard.putBoolean("IMU_IsCalibrating", mxp.isCalibrating());
		SmartDashboard.putNumber("IMU_Yaw", mxp.getYaw());
		SmartDashboard.putNumber("IMU_Pitch", mxp.getPitch());
		SmartDashboard.putNumber("IMU_Roll", mxp.getRoll());

		/* Display tilt-corrected, Magnetometer-based heading (requires */
		/* magnetometer calibration to be useful) */

		SmartDashboard.putNumber("IMU_CompassHeading", mxp.getCompassHeading());

		/*
		 * Display 9-axis Heading (requires magnetometer calibration to be
		 * useful)
		 */
		SmartDashboard.putNumber("IMU_FusedHeading", mxp.getFusedHeading());

		/*
		 * These functions are compatible w/the WPI Gyro Class, providing a
		 * simple
		 */
		/* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

		SmartDashboard.putNumber("IMU_TotalYaw", mxp.getAngle());
		SmartDashboard.putNumber("IMU_YawRateDPS", mxp.getRate());

		/*
		 * Display Processed Acceleration Data (Linear Acceleration, Motion
		 * Detect)
		 */

		SmartDashboard.putNumber("IMU_Accel_X", mxp.getWorldLinearAccelX());
		SmartDashboard.putNumber("IMU_Accel_Y", mxp.getWorldLinearAccelY());
		SmartDashboard.putBoolean("IMU_IsMoving", mxp.isMoving());
		SmartDashboard.putBoolean("IMU_IsRotating", mxp.isRotating());

		/*
		 * Display estimates of velocity/displacement. Note that these values
		 * are
		 */
		/*
		 * not expected to be accurate enough for estimating robot position on a
		 */
		/*
		 * FIRST FRC Robotics Field, due to accelerometer noise and the
		 * compounding
		 */
		/*
		 * of these errors due to single (velocity) integration and especially
		 */
		/* double (displacement) integration. */

		SmartDashboard.putNumber("Velocity_X", mxp.getVelocityX());
		SmartDashboard.putNumber("Velocity_Y", mxp.getVelocityY());
		SmartDashboard.putNumber("Displacement_X", mxp.getDisplacementX());
		SmartDashboard.putNumber("Displacement_Y", mxp.getDisplacementY());

		/* Display Raw Gyro/Accelerometer/Magnetometer Values */
		/*
		 * NOTE: These values are not normally necessary, but are made available
		 */
		/*
		 * for advanced users. Before using this data, please consider whether
		 */
		/* the processed data (see above) will suit your needs. */

		SmartDashboard.putNumber("RawGyro_X", mxp.getRawGyroX());
		SmartDashboard.putNumber("RawGyro_Y", mxp.getRawGyroY());
		SmartDashboard.putNumber("RawGyro_Z", mxp.getRawGyroZ());
		SmartDashboard.putNumber("RawAccel_X", mxp.getRawAccelX());
		SmartDashboard.putNumber("RawAccel_Y", mxp.getRawAccelY());
		SmartDashboard.putNumber("RawAccel_Z", mxp.getRawAccelZ());
		SmartDashboard.putNumber("RawMag_X", mxp.getRawMagX());
		SmartDashboard.putNumber("RawMag_Y", mxp.getRawMagY());
		SmartDashboard.putNumber("RawMag_Z", mxp.getRawMagZ());
		SmartDashboard.putNumber("IMU_Temp_C", mxp.getTempC());
		SmartDashboard.putNumber("IMU_Timestamp", mxp.getLastSensorTimestamp());

		/* Omnimount Yaw Axis Information */
		/*
		 * For more info, see
		 * http://navx-mxp.kauailabs.com/installation/omnimount
		 */
		AHRS.BoardYawAxis yaw_axis = mxp.getBoardYawAxis();
		SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
		SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

		/* Sensor Board Information */
		SmartDashboard.putString("FirmwareVersion", mxp.getFirmwareVersion());

		/* Quaternion Data */
		/*
		 * Quaternions are fascinating, and are the most compact representation
		 * of
		 */
		/*
		 * orientation data. All of the Yaw, Pitch and Roll Values can be
		 * derived
		 */
		/*
		 * from the Quaternions. If interested in motion processing, knowledge
		 * of
		 */
		/* Quaternions is highly recommended. */
		SmartDashboard.putNumber("QuaternionW", mxp.getQuaternionW());
		SmartDashboard.putNumber("QuaternionX", mxp.getQuaternionX());
		SmartDashboard.putNumber("QuaternionY", mxp.getQuaternionY());
		SmartDashboard.putNumber("QuaternionZ", mxp.getQuaternionZ());

		/* Connectivity Debugging Support */
		SmartDashboard.putNumber("IMU_Byte_Count", mxp.getByteCount());
		SmartDashboard.putNumber("IMU_Update_Count", mxp.getUpdateCount());
	}
}
