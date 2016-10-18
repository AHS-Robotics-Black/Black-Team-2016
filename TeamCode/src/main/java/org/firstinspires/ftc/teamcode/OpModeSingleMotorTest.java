package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Benjamin Jensrud on 10/18/2016.
 * Allows manual control of a single motor
 */

@TeleOp(name = "Single Motor Test" , group = "Tests")
public class OpModeSingleMotorTest extends LinearOpMode{

    DcMotor motor = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motor=hardwareMap.dcMotor.get("motor");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
