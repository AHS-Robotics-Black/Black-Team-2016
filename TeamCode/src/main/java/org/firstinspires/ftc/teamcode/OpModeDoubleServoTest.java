package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Benjamin Jensrud on 11/17/2016.
 * For testing the two light pushers
 */

@TeleOp(name = "Servo Test" , group = "Tests")
@Disabled
public class OpModeDoubleServoTest extends OpMode {

    Servo left = null;
    //Servo right = null;

    @Override
    public void init() {
        left = hardwareMap.servo.get("left");
        //right = hardwareMap.servo.get("right");
    }

    @Override
    public void loop() {
        left.setPosition((gamepad1.left_stick_y+1)/2.0);
        //right.setPosition((gamepad1.right_stick_y+1)/2.0);
    }
}
