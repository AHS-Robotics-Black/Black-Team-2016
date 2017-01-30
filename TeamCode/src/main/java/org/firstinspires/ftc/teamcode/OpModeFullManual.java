package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Benjamin Jensrud on 10/21/2016.
 * The final manual control opmode
 */

@TeleOp(name = "Default TeleOp", group = "Competition")
public class OpModeFullManual extends OpMode {

    //l-left
    //r-right
    //f-front
    //b-back
    DcMotor lfd, lbd, rfd, rbd, conveyor, leftLauncher, rightLauncher, drawerSlide;
    Servo frontServo, capBallServo;
    //ColorSensor leftColor, rightColor, bottomColor;

    @Override
    public void init() {
        lfd           = hardwareMap.dcMotor.get("lfd");
        lbd           = hardwareMap.dcMotor.get("lbd");
        rfd           = hardwareMap.dcMotor.get("rfd");
        rbd           = hardwareMap.dcMotor.get("rbd");

        conveyor      = hardwareMap.dcMotor.get("conveyor");
        frontServo    = hardwareMap.servo  .get("frontservo");
        leftLauncher  = hardwareMap.dcMotor.get("leftlauncher");
        rightLauncher = hardwareMap.dcMotor.get("rightlauncher");
		drawerSlide   = hardwareMap.dcMotor.get("drawerSlide");
		capBallServo  = hardwareMap.servo  .get("capBallServo");

        //leftColor   = hardwareMap.colorSensor.get("leftcolor");
        //rightColor  = hardwareMap.colorSensor.get("rightcolor");
        //bottomColor = hardwareMap.colorSensor.get("bottomcolor");

        //rightColor.enableLed(false);

        lfd.setDirection(DcMotor.Direction.REVERSE);
        lbd.setDirection(DcMotor.Direction.REVERSE);
        rfd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        frontServo.setPosition(0.5);
    }

    @Override
    public void loop() {
        //telemetry.addData("Red: ", ""+rightColor.red());
        //telemetry.addData("Blue: ", ""+rightColor.blue());
        //telemetry.update();

        lfd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*-gamepad1.left_stick_y+0.5*gamepad1.right_stick_x);
        lbd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*-gamepad1.left_stick_x+0.5*gamepad1.right_stick_x);
        rfd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*-gamepad1.left_stick_x-0.5*gamepad1.right_stick_x);
        rbd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*-gamepad1.left_stick_y-0.5*gamepad1.right_stick_x);

        if (gamepad2.right_bumper) {
            rightLauncher.setPower(0.2);
            leftLauncher.setPower(0.2);
        }
        else {
            rightLauncher.setPower(0);
            leftLauncher.setPower(0);
        }

        if (gamepad1.right_bumper) {
            drawerSlide.setPower(1);
        }
		else if (gamepad1.left_bumper) {
			drawerSlide.setPower(-1);
		}
		else {
			drawerSlide.setPower(0);
		}

		if (!gamepad1.a) {
			capBallServo.setPosition(-1.0);
		}
		else {
			capBallServo.setPosition(1.0);
		}

        frontServo.setPosition(0.5*(gamepad2.left_stick_y+1));

        conveyor.setPower(gamepad2.right_stick_y);
    }
}
