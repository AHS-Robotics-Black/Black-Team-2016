package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Benjamin Jensrud on 11/18/2016.
 * Testing the new wheel system
 */
@TeleOp(name = "Death" , group = "Tests")
public class OpModeQuestionableTest extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
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

            double forward = gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotRate = gamepad1.right_stick_x;

            leftFront.setPower(clamp((0.5*(forward+rotRate)),-1,1));
            leftBack.setPower(clamp((0.5*(-1*right+rotRate)),-1,1));
            rightFront.setPower(clamp((0.5*(right+rotRate)),-1,1));
            rightBack.setPower(clamp((0.5*(-1*forward+rotRate)),-1,1));
        }
    }

    private static double clamp(double val, double min, double max) {
        if (val<=max&&val>=min) return val;
        else if (val>max) return max;
        else return min;
    }
}
