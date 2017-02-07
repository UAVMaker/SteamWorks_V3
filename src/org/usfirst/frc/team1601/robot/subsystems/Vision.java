package org.usfirst.frc.team1601.robot.subsystems;

import org.opencv.core.Mat;
import org.usfirst.frc.team1601.robot.Robot;
import org.usfirst.frc.team1601.robot.Systemcommands.VisionSwitcher;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Vision extends Subsystem {

	boolean allowCam1 = false;

	UsbCamera camera1, camera2;

	CvSource outputStream;
	CvSink cvSink1, cvSink2;
	Mat image;

	public Vision() {
		camera1 = CameraServer.getInstance().startAutomaticCapture(0);
		camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 240);
		cvSink1 = CameraServer.getInstance().getVideo(camera1);
		cvSink2 = CameraServer.getInstance().getVideo(camera2);
		image = new Mat();
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new VisionSwitcher());

	}

	public void setResolution(int width, int height, int fps) {

		camera1.setResolution(width, height);
		camera1.setFPS(fps);

		camera2.setResolution(width, height);
		camera2.setFPS(fps);
	}

	public void switchVisionFeed(boolean switcherButton) {

		if (switcherButton) {
			cvSink2.setEnabled(false);
			cvSink1.setEnabled(true);
			cvSink1.grabFrame(image);
		} else {
			cvSink1.setEnabled(false);
			cvSink2.setEnabled(true);
			cvSink2.grabFrame(image);
		}

		outputStream.putFrame(image);
	}
}
