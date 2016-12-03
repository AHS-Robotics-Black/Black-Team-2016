package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    DcMotor lfd, lbd, rfd, rbd, conveyor;
    Servo leftPushServo, rightPushServo, frontServo, leftLauncher, rightLauncher;
    ColorSensor leftColor, rightColor, bottomColor;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        lfd=hardwareMap.dcMotor.get("lfd");
        lbd=hardwareMap.dcMotor.get("lbd");
        rfd=hardwareMap.dcMotor.get("rfd");
        rbd=hardwareMap.dcMotor.get("rbd");

        conveyor=hardwareMap.dcMotor.get("conveyor");
        leftPushServo=hardwareMap.servo.get("leftpushservo");
        rightPushServo=hardwareMap.servo.get("rightpushservo");
        frontServo=hardwareMap.servo.get("frontservo");
        leftLauncher=hardwareMap.servo.get("leftlauncher");
        rightLauncher=hardwareMap.servo.get("rightlauncher");

        leftColor=hardwareMap.colorSensor.get("leftcolor");
        rightColor=hardwareMap.colorSensor.get("rightcolor");
        bottomColor=hardwareMap.colorSensor.get("bottomcolor");

        lfd.setDirection(DcMotor.Direction.REVERSE);
        lbd.setDirection(DcMotor.Direction.REVERSE);
        rfd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.FORWARD);

        rightLauncher.setPosition(1);
        leftLauncher.setPosition(1);
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Runtime: "+runtime.toString());
        telemetry.update();

        lfd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*gamepad1.left_stick_x+0.5*gamepad1.right_stick_y);
        lbd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*gamepad1.left_stick_y+0.5*gamepad1.right_stick_y);
        rfd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*gamepad1.left_stick_y-0.5*gamepad1.right_stick_y);
        rbd.setPower((1-0.5*Math.abs(gamepad1.right_stick_y))*gamepad1.left_stick_x-0.5*gamepad1.right_stick_y);

        if (gamepad1.right_bumper) {
            //TODO determine the correct values for servo positions
            leftPushServo.setPosition(1);
            rightPushServo.setPosition(1);
        }
        else {
            leftPushServo.setPosition(-1);
            rightPushServo.setPosition(-1);
        }

        if (gamepad2.right_bumper) {
            rightLauncher.setPosition(0);
            leftLauncher.setPosition(0);
        }
        else {
            rightLauncher.setPosition(-1);
            leftLauncher.setPosition(-1);
        }

        frontServo.setPosition(gamepad2.left_stick_y);

        conveyor.setPower(gamepad2.right_stick_y);
    }
}
