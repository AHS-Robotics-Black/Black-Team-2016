package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Benjamin Jensrud on 11/18/2016.
 * Testing the new wheel system
 */
public class OpModeQuestionableTest extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status","Initialized");
        telemetry.update();

        leftFront=hardwareMap.dcMotor.get("lfd");
        leftBack=hardwareMap.dcMotor.get("lbd");
        rightFront=hardwareMap.dcMotor.get("rfd");
        rightBack=hardwareMap.dcMotor.get("rbd");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Runtime: "+runtime.toString());
            telemetry.update();

            leftFront.setPower(gamepad1.left_stick_x);
            leftBack.setPower(gamepad1.left_stick_y);
            rightFront.setPower(gamepad1.right_stick_y);
            rightBack.setPower(gamepad1.right_stick_x);
        }
    }
}
