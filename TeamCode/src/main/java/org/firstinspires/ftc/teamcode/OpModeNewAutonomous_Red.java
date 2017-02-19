package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.concurrent.TimeUnit;

/**
 * Created by Benjamin Jensrud on 1/28/2017.
 * The new autonomous, for red
 */
@Autonomous(name = "New Autonomous - Red", group = "Competition")
public class OpModeNewAutonomous_Red  extends LinearOpMode{
	//True, blue
	//False, red
	public static final boolean TEAM = false;
	public static final double WHEEL_RADIUS = 1.75;
	public static final double WHEEL_DEGREES = 54;
	public static final double INNER_RADIUS = 9.31;
	public static final double OUTER_RADIUS = 10.16;
	public static final double WHITE = 20;
	public static final double SPEED = 0.2;

	private boolean interrupt = false;


	//l-left
	//r-right
	//f-front
	//b-back
	DcMotor lfd, lbd, rfd, rbd, conveyor, leftLauncher, rightLauncher;
	Servo frontServo;
	//ColorSensor leftColor, rightColor, bottomColor;
	UltrasonicSensor ultrasonic;
	I2cDevice leftColor, rightColor, bottomColor;
	I2cDeviceSynch leftColorReader, rightColorReader, bottomColorReader;

	@Override
	public void runOpMode() {

		new EndThread(this).start();
		try {
			lfd = hardwareMap.dcMotor.get("lfd");
			lbd = hardwareMap.dcMotor.get("lbd");
			rfd = hardwareMap.dcMotor.get("rfd");
			rbd = hardwareMap.dcMotor.get("rbd");

			conveyor = hardwareMap.dcMotor.get("conveyor");
			frontServo = hardwareMap.servo.get("frontservo");
			leftLauncher = hardwareMap.dcMotor.get("leftlauncher");
			rightLauncher = hardwareMap.dcMotor.get("rightlauncher");

			//leftColor = hardwareMap.colorSensor.get("leftcolor");
			//rightColor = hardwareMap.colorSensor.get("rightcolor");
			//bottomColor = hardwareMap.colorSensor.get("bottomcolor");

			leftColor = hardwareMap.i2cDevice.get("leftcolor");
			rightColor = hardwareMap.i2cDevice.get("rightcolor");
			bottomColor = hardwareMap.i2cDevice.get("bottomcolor");

			leftColorReader = new I2cDeviceSynchImpl(leftColor, I2cAddr.create8bit(0x3c), false);
			rightColorReader = new I2cDeviceSynchImpl(rightColor, I2cAddr.create8bit(0x4c), false);
			bottomColorReader = new I2cDeviceSynchImpl(bottomColor, I2cAddr.create8bit(0x5c), false);

			leftColorReader.engage();
			rightColorReader.engage();
			bottomColorReader.engage();

			leftColorReader.write8(3, 1);
			rightColorReader.write8(3, 1);
			bottomColorReader.write8(3, 0);

			ultrasonic = hardwareMap.ultrasonicSensor.get("ultrasonic");


			//leftColor.enableLed(false);
			//rightColor.enableLed(false);
			//bottomColor.enableLed(true);

			lfd.setDirection(DcMotor.Direction.REVERSE);
			lbd.setDirection(DcMotor.Direction.REVERSE);
			rfd.setDirection(DcMotor.Direction.FORWARD);
			rbd.setDirection(DcMotor.Direction.FORWARD);

			leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

			frontServo.setPosition(0.5);

			waitForStart();

			/*
			while (!isStopRequested()) {
				//telemetry.addData("ARGB",""+bottomColor.argb()+", "+leftColor.argb()+", "+rightColor.argb());
				//telemetry.addData("Alpha",""+bottomColor.alpha()+", "+leftColor.alpha()+", "+rightColor.alpha());
				//telemetry.addData("Red",""+bottomColor.red()+", "+leftColor.red()+", c"+rightColor.red());
				//telemetry.addData("Green",""+bottomColor.green()+", "+leftColor.green()+", "+rightColor.green());
				//telemetry.addData("Blue",""+bottomColor.blue()+", "+leftColor.blue()+", "+rightColor.blue());
				telemetry.addData("Color", "" + (bottomColorReader.read(0x04, 1)[0]));
				telemetry.update();
			}
			*/

			telemetry.addData("Status: ", "Neutral");
			telemetry.update();

			/*

			alignToBeacon(TEAM);

			telemetry.addData("Left: " ,""+getColor(leftColorReader));
			telemetry.addData("Right: ",""+getColor(rightColorReader));
			telemetry.update();

			pressCorrectBeacon(TEAM);


			*/
			driveForward(72);
			turn(nt(70));
			/*
			leftLauncher.setPower(0.4);
			rightLauncher.setPower(0.4);
			TimeUnit.SECONDS.sleep(1);
			conveyor.setPower(0.4);
			TimeUnit.SECONDS.sleep(2);
			leftLauncher.setPower(0);
			rightLauncher.setPower(0);
			conveyor.setPower(0);
			*/

			driveForward(36);



			//DONE get to beacon

			alignToBeacon(TEAM);
			pressCorrectBeacon(TEAM);
			driveForward(-5);
			driveLeft(nt(24));
			alignToBeacon(TEAM);
			pressCorrectBeacon(TEAM);

			/**/
		}
		catch (InterruptedException e) {
			setPowers(0,0,0);
		}
	}

	//x, y, and z clamp between -1.0 and 1.0
	private void setPowers(double forward, double right, double rot) {
		lfd.setPower((1 - 0.5 * Math.abs(rot)) * forward + (1 - 0.5 * Math.abs(right)) * rot);
		lbd.setPower((1 - 0.5 * Math.abs(rot)) * right + (1 - 0.5 * Math.abs(forward)) * rot * (INNER_RADIUS / OUTER_RADIUS));
		rfd.setPower((1 - 0.5 * Math.abs(rot)) * right - (1 - 0.5 * Math.abs(forward)) * rot * (INNER_RADIUS / OUTER_RADIUS));
		rbd.setPower((1 - 0.5 * Math.abs(rot)) * forward - (1 - 0.5 * Math.abs(right)) * rot);
	}

	private static double nt(double val) {
		return TEAM ? val : -val;
	}

	private void driveForward(double inches, double power) throws InterruptedException {
		int initialTicks = lfd.getCurrentPosition();
		double targetRadians = inches / WHEEL_RADIUS;
		/*
		telemetry.addData("Initial Ticks: ", "" + initialTicks);
		telemetry.addData("Current Ticks: ", "" + (lfd.getCurrentPosition() - initialTicks));
		telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
		telemetry.addData("Condition: ", "Less than");
		telemetry.update();
		*/
		if (inches == 0) return;
		if (inches > 0) {
			setPowers(Math.abs(power), 0, 0);
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 <= targetRadians) {
				/*
				telemetry.addData("Status: ", "Driving forward");
				//telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Radians: ", "" + ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12));
				telemetry.addData("Target Radians: ", "" + targetRadians);
				telemetry.addData("Condition: ", "Less than");
				telemetry.update();
				*/
				checkInterrupted();
			}
			setPowers(0, 0, 0);
		} else if (inches < 0) {
			setPowers(-Math.abs(power), 0, 0);
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 >= targetRadians) {
				/*
				telemetry.addData("Status: ", "Driving backward");
				//telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Radians: ", "" + ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12));
				telemetry.addData("Target Radians: ", "" + targetRadians);
				telemetry.addData("Condition: ", "Greater than");
				telemetry.update();
				*/
				checkInterrupted();
			}
			setPowers(0, 0, 0);
		}
	}

	private void driveLeft(double inches, double power) throws InterruptedException {
		int initialTicks = lbd.getCurrentPosition();
		double targetRadians = inches / WHEEL_RADIUS;
		/*
		telemetry.addData("Initial Ticks: ", "" + initialTicks);
		telemetry.addData("Current Ticks: ", "" + (lbd.getCurrentPosition() - initialTicks));
		telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
		telemetry.addData("Condition: ", "Less than");
		telemetry.update();
		*/
		if (inches == 0) return;
		if (inches > 0) {
			setPowers(0, Math.abs(power), 0);
			while ((lbd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 <= targetRadians) {

				/*
				telemetry.addData("Status: ", "Driving forward");
				//telemetry.addData("Initial Ticks: ", "" + initialTicks);

				telemetry.addData("Current Radians: ", "" + ((lbd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12));
				telemetry.addData("Target Radians: ", "" + targetRadians);
				telemetry.addData("Condition: ", "Less than");
				telemetry.update();
				*/
				checkInterrupted();
			}
			setPowers(0, 0, 0);
		} else if (inches < 0) {
			setPowers(0, -Math.abs(power), 0);
			while ((lbd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 >= targetRadians) {
				/*
				telemetry.addData("Status: ", "Driving backward");
				//telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Radians: ", "" + ((lbd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12));
				telemetry.addData("Target Radians: ", "" + targetRadians);
				telemetry.addData("Condition: ", "Greater than");
				telemetry.update();
				*/
				checkInterrupted();
			}
			setPowers(0, 0, 0);
		}
	}

	private void driveLeft(double inches) throws InterruptedException {
		driveLeft(inches, SPEED);
	}

	private void turn(double degrees, double power) throws InterruptedException {
		int initialTicks = lfd.getCurrentPosition();
		double targetRadians = (INNER_RADIUS * degrees * Math.PI / 180) / (WHEEL_RADIUS * Math.cos(WHEEL_DEGREES * Math.PI / 180));
		if (degrees == 0) return;
		if (degrees > 0) {
			setPowers(0, 0, Math.abs(power));
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI /757.12 <= targetRadians) {
				/*
				telemetry.addData("Status: ", "Turning right");
				telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Ticks: ", "" + (lfd.getCurrentPosition() - initialTicks));
				telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
				telemetry.addData("Condition: ", "Less than");
				telemetry.update();
				*/
				checkInterrupted();
			}
			setPowers(0, 0, 0);
		} else if (degrees < 0) {
			setPowers(0, 0, -Math.abs(power));
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI /757.12 >= targetRadians) {
				/*
				telemetry.addData("Status: ", "Turning left");
				telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Ticks: ", "" + (lfd.getCurrentPosition() - initialTicks));
				telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
				telemetry.addData("Condition: ", "Greater than");
				telemetry.update();
				*/
				checkInterrupted();
			}
		}
	}

	private void turn(double degrees) throws InterruptedException {
		turn(degrees, SPEED);
	}

	private void driveForward(double inches) throws InterruptedException{
		driveForward(inches, SPEED);
	}

	private void waitForA() throws InterruptedException {

		/*
		while (!gamepad1.a) {
			checkInterrupted();
		}
		while (gamepad1.a) {
			checkInterrupted();
		}
		*/

	}

	//For side
	//If true, starting to the left
	//If false, starting to the right
	private void alignToBeacon(boolean side) throws InterruptedException {
		setPowers(0,SPEED*(!side?-1:1),0);
		int color = getWhite(bottomColorReader);
		while (color < WHITE) {
			color = getWhite(bottomColorReader);
			checkInterrupted();
		}
		setPowers(0,0,0);
		if (ultrasonic.getUltrasonicLevel()>15) {
			setPowers(SPEED,0,0);
		}

		while (ultrasonic.getUltrasonicLevel()>15) {
			telemetry.addData("Distance: ",""+ultrasonic.getUltrasonicLevel() );
			telemetry.update();
			checkInterrupted();
			TimeUnit.MILLISECONDS.sleep(100);
		}
		setPowers(0,0,0);
	}


	//Color >= 7 means red (should be 2 or 3)
	//      <= 6 means blue (should be 10 or 11)
	//For team, true is blue and false is red
	private void pressCorrectBeacon(boolean team) throws InterruptedException {
		int right = getColor(rightColorReader);
		int left = getColor(leftColorReader);
		while (left==0||right==0) {
			if (left==0&&right==0) {
				throw new InterruptedException();
			}
			else if (left==0) {
				driveLeft(0.7);
			}
			else if (right==0) {
				driveLeft(-0.7);
			}
			right = getColor(rightColorReader);
			left = getColor(leftColorReader);
		}
		if (right >=7&&left<=6) {
			driveLeft((team?-1:1)*10);
			setPowers(SPEED,0,0);
			TimeUnit.SECONDS.sleep(2);
			driveForward(-8);
			setPowers(0,0,0);
		}
		else if (left >= 7 && right <= 6) {
			driveLeft((team?1:-1)*10);
			setPowers(SPEED,0,0);
			TimeUnit.SECONDS.sleep(2);
			driveForward(-8);
			setPowers(0,0,0);
		}
	}

	private int getColor(I2cDeviceSynch device) {
		return device.read(0x04,1)[0]&0xFF;
	}

	private int getWhite(I2cDeviceSynch device) {
		return device.read(0x08,1)[0]&0xFF;
	}

	private void interrupt() {
		interrupt=true;

	}

	private void checkInterrupted() throws InterruptedException {
		if (interrupt) throw new InterruptedException();
	}

	private class EndThread extends Thread{

		OpModeNewAutonomous_Red calledBy;

		public EndThread(OpModeNewAutonomous_Red called) {
			calledBy=called;
		}
		@Override
		public void run(){
			while (!calledBy.isStopRequested()) {
				try {
					TimeUnit.MILLISECONDS.sleep(100);
				}
				catch (InterruptedException e) {
					telemetry.addData("ERROR", "There was a timer interruption, don't know why ¯\\_(ツ)_/¯");
				}
			}
			setPowers(0,0,0);
			calledBy.interrupt();
		}
	}
}
