package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Benjamin Jensrud on 11/18/2016.
 * Testing the new wheel system
 */
@TeleOp(name = "Death", group = "Tests")
@Disabled
public class OpModeQuestionableTest extends LinearOpMode {

	private DcMotor leftFront, leftBack, rightFront, rightBack;
	private ElapsedTime runtime = new ElapsedTime();

	@Override
	public void runOpMode() throws InterruptedException {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		leftFront = hardwareMap.dcMotor.get("lfd");
		leftBack = hardwareMap.dcMotor.get("lbd");
		rightFront = hardwareMap.dcMotor.get("rfd");
		rightBack = hardwareMap.dcMotor.get("rbd");

		leftFront.setDirection(DcMotor.Direction.FORWARD);
		leftBack.setDirection(DcMotor.Direction.FORWARD);
		rightFront.setDirection(DcMotor.Direction.REVERSE);
		rightBack.setDirection(DcMotor.Direction.REVERSE);

		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {
			telemetry.addData("Status", "Runtime: " + runtime.toString());
			telemetry.update();

			leftFront.setPower((1 - 0.5 * Math.abs(gamepad1.right_stick_y)) * gamepad1.left_stick_x);
			leftBack.setPower((1 - 0.5 * Math.abs(gamepad1.right_stick_y)) * gamepad1.left_stick_y);
			rightFront.setPower((1 - 0.5 * Math.abs(gamepad1.right_stick_y)) * gamepad1.left_stick_y);
			rightBack.setPower((1 - 0.5 * Math.abs(gamepad1.right_stick_y)) * gamepad1.left_stick_x);
		}
	}
}
