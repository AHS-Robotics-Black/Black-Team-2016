package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

/**
 * Created by Benjamin Jensrud on 11/10/2016.
 * The final autonomous opmode
 */

@Autonomous(name = "Default Autonomous - Blue", group = "Competition")
public class OpModeFullAutonomous extends LinearOpMode {

	//True, blue
	//False, red
	public static final boolean TEAM = true;
	public static final double WHEEL_RADIUS = 1.75;
	public static final double WHEEL_DEGREES = 54;
	public static final double INNER_RADIUS = 9.31;
	public static final double OUTER_RADIUS = 10.16;
	//public static final double ARM_LENGTH = 9.5;


	//l-left
	//r-right
	//f-front
	//b-back
	DcMotor lfd, lbd, rfd, rbd, conveyor, leftLauncher, rightLauncher;
	Servo /*leftPushServo, rightPushServo,*/ frontServo;
	ColorSensor leftColor, rightColor, bottomColor;
	//PushServoMotor leftPush, rightPush;

	@Override
	public void runOpMode() throws InterruptedException {
		lfd = hardwareMap.dcMotor.get("lfd");
		lbd = hardwareMap.dcMotor.get("lbd");
		rfd = hardwareMap.dcMotor.get("rfd");
		rbd = hardwareMap.dcMotor.get("rbd");

		conveyor = hardwareMap.dcMotor.get("conveyor");
		//leftPushServo = hardwareMap.servo.get("leftpushservo");
		//rightPushServo = hardwareMap.servo.get("rightpushservo");
		frontServo = hardwareMap.servo.get("frontservo");
		leftLauncher = hardwareMap.dcMotor.get("leftlauncher");
		rightLauncher = hardwareMap.dcMotor.get("rightlauncher");

		//rightPush = new PushServoMotor(rightPushServo, 0, 0.482);
		//leftPush = new PushServoMotor(leftPushServo, 0.88318, 0.3658);

		//leftColor = hardwareMap.colorSensor.get("leftcolor");
		rightColor = hardwareMap.colorSensor.get("rightcolor");
		//bottomColor = hardwareMap.colorSensor.get("bottomcolor");
		//opticalDistance = hardwareMap.opticalDistanceSensor.get("distance");

		rightColor.enableLed(false);

		lfd.setDirection(DcMotor.Direction.REVERSE);
		lbd.setDirection(DcMotor.Direction.REVERSE);
		rfd.setDirection(DcMotor.Direction.FORWARD);
		rbd.setDirection(DcMotor.Direction.FORWARD);

		leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

		//leftPush.setToBottom();
		//rightPush.setToBottom();
		frontServo.setPosition(0.5);

		waitForStart();

		//Hit ball and align to beacon 1
		telemetry.addData("Status: ", "Neutral");
		telemetry.update();

		driveForward(15, 0.17239);
		waitForA();
		turn(nt(45), 0.17239);
		turn(nt(210), 0.17239);
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
		turn(nt(205), 0.17239);
		waitForA();
		driveForward(33.94, 0.17239);
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

		setPowers(0, nt(0.17239), 0);
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
		setPowers(0, nt(0.17239), 0);
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
	private void setPowers(double x, double y, double rot) {
		lfd.setPower((1 - 0.5 * Math.abs(rot)) * y + (1 - 0.5 * Math.abs(y)) * rot);
		lbd.setPower((1 - 0.5 * Math.abs(rot)) * x + (1 - 0.5 * Math.abs(x)) * rot * (INNER_RADIUS / OUTER_RADIUS));
		rfd.setPower((1 - 0.5 * Math.abs(rot)) * x - (1 - 0.5 * Math.abs(x)) * rot * (INNER_RADIUS / OUTER_RADIUS));
		rbd.setPower((1 - 0.5 * Math.abs(rot)) * y - (1 - 0.5 * Math.abs(y)) * rot);
	}

	private static double nt(double val) {
		return TEAM ? val : -val;
	}

	private void driveForward(double inches, double power) {
		int initialTicks = rfd.getCurrentPosition();
		double targetRadians = 1.75 * inches / WHEEL_RADIUS;
		telemetry.addData("Initial Ticks: ", "" + initialTicks);
		telemetry.addData("Current Ticks: ", "" + (rfd.getCurrentPosition() - initialTicks));
		telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
		telemetry.addData("Condition: ", "Less than");
		telemetry.update();
		if (inches == 0) return;
		if (inches > 0) {
			setPowers(Math.abs(power), 0, 0);
			while ((rfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 <= targetRadians) {
				telemetry.addData("Status: ", "Driving forward");
				telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Ticks: ", "" + (rfd.getCurrentPosition() - initialTicks));
				telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
				telemetry.addData("Condition: ", "Less than");
				telemetry.update();
			}
			setPowers(0, 0, 0);
		} else if (inches < 0) {
			setPowers(-Math.abs(power), 0, 0);
			while ((rfd.getCurrentPosition() - initialTicks) * 2 * Math.PI / 757.12 >= targetRadians) {
				telemetry.addData("Status: ", "Driving backward");
				telemetry.addData("Initial Ticks: ", "" + initialTicks);
				telemetry.addData("Current Ticks: ", "" + (rfd.getCurrentPosition() - initialTicks));
				telemetry.addData("Target Ticks: ", "" + (targetRadians*757.12/(2*Math.PI)));
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
		turn(degrees, 0.17239);
	}

	private void driveForward(double inches) {
		driveForward(inches, 0.17239);
	}

	private void waitForA() {
		/*
		while (!gamepad1.a) {

		}
		while (gamepad1.a) {

		}
		*/
	}



}
