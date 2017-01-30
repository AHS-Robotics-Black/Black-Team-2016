package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
 * Created by Benjamin Jensrud on 1/26/2017.
 * The new autonomous, for blue
 */
@Autonomous(name = "New Autonomous - Blue", group = "Competition")
public class OpModeNewAutonomous_Blue extends LinearOpMode {
	//True, blue
	//False, red
	public static final boolean TEAM = true;
	public static final double WHEEL_RADIUS = 1.75;
	public static final double WHEEL_DEGREES = 54;
	public static final double INNER_RADIUS = 9.31;
	public static final double OUTER_RADIUS = 10.16;
	public static final double WHITE = 10;
	public static final double SPEED = 0.17239;


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
	byte[] leftColorCache, rightColorCache, bottomColorCache;

	@Override
	public void runOpMode() throws InterruptedException {
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
		bottomColorReader = new I2cDeviceSynchImpl(bottomColor, I2cAddr.create8bit(0xc), false);

		leftColorReader.engage();
		rightColorReader.engage();
		bottomColorReader.engage();

		leftColorReader.write8(3,1);
		rightColorReader.write8(3,1);
		bottomColorReader.write8(3,0);

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
		while (true) {
			//telemetry.addData("ARGB",""+bottomColor.argb()+", "+leftColor.argb()+", "+rightColor.argb());
			//telemetry.addData("Alpha",""+bottomColor.alpha()+", "+leftColor.alpha()+", "+rightColor.alpha());
			//telemetry.addData("Red",""+bottomColor.red()+", "+leftColor.red()+", c"+rightColor.red());
			//telemetry.addData("Green",""+bottomColor.green()+", "+leftColor.green()+", "+rightColor.green());
			//telemetry.addData("Blue",""+bottomColor.blue()+", "+leftColor.blue()+", "+rightColor.blue());
			telemetry.addData("Color", ""+(bottomColorReader.read(0x04,1)[0]));
			telemetry.update();
		}
		*/

		telemetry.addData("Status: ", "Neutral");
		telemetry.update();


		driveForward(15);
		turn(nt(45));
		leftLauncher.setPower(0.25);
		rightLauncher.setPower(0.25);
		TimeUnit.SECONDS.sleep(2);
		conveyor.setPower(0.4);
		TimeUnit.SECONDS.sleep(5);
		leftLauncher.setPower(0);
		rightLauncher.setPower(0);
		conveyor.setPower(0);
		driveForward(33.94);

		//DONE get to beacon
		/*
		turn(nt(45));
		driveForward(60);

		alignToBeacon(!TEAM);
		pressCorrectBeacon(TEAM);
		setPowers(0,nt(SPEED),0);
		TimeUnit.SECONDS.sleep(1);
		alignToBeacon(!TEAM);
		pressCorrectBeacon(TEAM);


		/*
		driveForward(15, SPEED);
		waitForA();
		turn(nt(45), SPEED);
		turn(nt(210), SPEED);
		waitForA();
		leftLauncher.setPower(0.25);
		rightLauncher.setPower(0.25);
		waitForA();
		TimeUnit.SECONDS.sleep(2);
		conveyor.setPower(0.4);
		TimeUnit.SECONDS.sleep(5);
		waitForA();
		leftLauncher.setPower(0);
		rightLauncher.setPower(0);
		conveyor.setPower(0);
		waitForA();
		turn(nt(205), SPEED);
		waitForA();
		driveForward(33.94, SPEED);
		waitForA();

		telemetry.addData("Status: ", "Pushing ball");
		telemetry.update();

		//rightPush.setToTop();
		//rightPush.setToBottom();

		waitForA();

		turn(nt(45));

		driveForward(10);
		TimeUnit.SECONDS.sleep(5);
		driveForward(-8);
		turn(nt(-45));
		driveForward(15);
		/*
		telemetry.addData("Status: ", "Finding line");
		telemetry.update();
		driveForward(21);

		setPowers(0, nt(SPEED), 0);
		while (rightColor.red() < 2 && rightColor.blue() < 2) ;
		setPowers(0, 0, 0);



		//Press correct button
		int red = rightColor.red();
		int blue = rightColor.blue();
		telemetry.addData("Red: ", "" + red);
		telemetry.addData("Blue: ", "" + blue);
		telemetry.update();
		driveForward(-10);
		if (TEAM ? (blue >= 2) : (red >= 2)) {
			leftPush.setToTop();
		} else
			rightPush.setToTop();

		driveForward(5);
		driveForward(-8);
		*/

		/*
		setPowers(0, nt(SPEED), 0);
		while (bottomColor.green() >= 100) {
			telemetry.addData("Green: ", "" + bottomColor.green());
			telemetry.update();
		}
		while (bottomColor.green() <= 100) {
			telemetry.addData("Green: ", ""+ bottomColor.green());
			telemetry.update();
		}
		setPowers(0,0,0);
		red = rightColor.red();
		blue = rightColor.blue();
		telemetry.addData("Red: ", "" + red);
		telemetry.addData("Blue: ", "" + blue);
		telemetry.update();
		driveForward(-10);
		if (TEAM ? (blue >= 2) : (red >= 2)) {
			leftPush.setToTop();
		} else
			rightPush.setToTop();
		System.out.print("Kill Me");
		*/
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

	private void driveForward(double inches, double power) {
		int initialTicks = lfd.getCurrentPosition();
		double targetRadians = inches / WHEEL_RADIUS;
		telemetry.addData("Initial Ticks: ", "" + initialTicks);
		telemetry.addData("Current Ticks: ", "" + (lfd.getCurrentPosition() - initialTicks));
		telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
		telemetry.addData("Condition: ", "Less than");
		telemetry.update();
		if (inches == 0) return;
		if (inches > 0) {
			setPowers(Math.abs(power), 0, 0);
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 <= targetRadians) {
				telemetry.addData("Status: ", "Driving forward");
				//telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Radians: ", "" + ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12));
				telemetry.addData("Target Radians: ", "" + targetRadians);
				telemetry.addData("Condition: ", "Less than");
				telemetry.update();
			}
			setPowers(0, 0, 0);
		} else if (inches < 0) {
			setPowers(-Math.abs(power), 0, 0);
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 >= targetRadians) {
				telemetry.addData("Status: ", "Driving backward");
				//telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Radians: ", "" + ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12));
				telemetry.addData("Target Radians: ", "" + targetRadians);
				telemetry.addData("Condition: ", "Greater than");
				telemetry.update();
			}
			setPowers(0, 0, 0);
		}
	}

	private void turn(double degrees, double power) {
		int initialTicks = lfd.getCurrentPosition();
		double targetRadians = (INNER_RADIUS * degrees * Math.PI / 180) / (WHEEL_RADIUS * Math.cos(WHEEL_DEGREES * Math.PI / 180));
		if (degrees == 0) return;
		if (degrees > 0) {
			setPowers(0, 0, Math.abs(power));
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI /757.12 <= targetRadians) {
				telemetry.addData("Status: ", "Turning right");
				telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Ticks: ", "" + (lfd.getCurrentPosition() - initialTicks));
				telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
				telemetry.addData("Condition: ", "Less than");
				telemetry.update();
			}
			setPowers(0, 0, 0);
		} else if (degrees < 0) {
			setPowers(0, 0, -Math.abs(power));
			while ((lfd.getCurrentPosition() - initialTicks) * 2 * Math.PI /757.12 >= targetRadians) {
				telemetry.addData("Status: ", "Turning left");
				telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Ticks: ", "" + (lfd.getCurrentPosition() - initialTicks));
				telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
				telemetry.addData("Condition: ", "Greater than");
				telemetry.update();
			}
		}
	}

	private void turn(double degrees) {
		turn(degrees, SPEED);
	}

	private void driveForward(double inches) {
		driveForward(inches, SPEED);
	}

	private void waitForA() {

		while (!gamepad1.a) {

		}
		while (gamepad1.a) {

		}

	}

	//For side
	//If true, starting to the left
	//If false, starting to the right
	private void alignToBeacon(boolean side) {
		setPowers(0,SPEED*(side?-1:1),0);
		int color = getColor(bottomColorReader);
		while (color <= WHITE) {
			color = getColor(bottomColorReader);
		}
		setPowers(0,0,0);
		if (ultrasonic.getUltrasonicLevel()>13) {
			setPowers(SPEED,0,0);
		}

		while (ultrasonic.getUltrasonicLevel()>13) {}
		setPowers(0,0,0);
	}


	//Color >= 7 means red
	//      <= 6 means blue
	//For team, true is blue and false is red
	private void pressCorrectBeacon(boolean team) throws InterruptedException {
		int right = getColor(rightColorReader);
		int left = getColor(leftColorReader);
		if (right >=7&&left<=6) {
			setPowers(0,(team?1:-1)*SPEED,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(SPEED,0,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(-SPEED,0,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(0,(team?-1:1)*SPEED,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(0,0,0);
		}
		else if (left >= 7 && right <= 6) {
			setPowers(0,(team?-1:1)*SPEED,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(SPEED,0,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(-SPEED,0,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(0,(team?1:-1)*SPEED,0);
			TimeUnit.SECONDS.sleep(1);//TODO test this number, it's wrong
			setPowers(0,0,0);
		}
		else {
			TimeUnit.SECONDS.sleep(1);
			pressCorrectBeacon(team);
		}
	}

	private int getColor(I2cDeviceSynch device) {
		return device.read(0x04,1)[0]&0xFF;
	}
}
