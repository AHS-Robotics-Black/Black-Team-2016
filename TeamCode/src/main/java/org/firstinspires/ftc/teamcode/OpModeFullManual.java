package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    //t-throwing
    DcMotor lfd, lbd, rfd, rbd, lt, rt;

    @Override
    public void init() {
        lfd=hardwareMap.dcMotor.get("lfd");
        lbd=hardwareMap.dcMotor.get("lbd");
        rfd=hardwareMap.dcMotor.get("rfd");
        rbd=hardwareMap.dcMotor.get("rbd");
        lt=hardwareMap.dcMotor.get("lt");
        rt=hardwareMap.dcMotor.get("rt");
    }

    @Override
    public void loop() {

    }
}
