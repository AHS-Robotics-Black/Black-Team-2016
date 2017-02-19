package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

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
	I2cDevice leftColor, rightColor, bottomColor;
	I2cDeviceSynch leftColorReader, rightColorReader, bottomColorReader;
	UltrasonicSensor ultrasonic;
	byte[] bottomColorCache, leftColorCache, rightColorCache;

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

		ultrasonic = hardwareMap.ultrasonicSensor.get("ultrasonic");

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

		leftColor = hardwareMap.i2cDevice.get("leftcolor");
		rightColor = hardwareMap.i2cDevice.get("rightcolor");
		bottomColor = hardwareMap.i2cDevice.get("bottomcolor");

		leftColorReader = new I2cDeviceSynchImpl(leftColor, I2cAddr.create8bit(0x3c), false);
		rightColorReader = new I2cDeviceSynchImpl(rightColor, I2cAddr.create8bit(0x4c), false);
		bottomColorReader = new I2cDeviceSynchImpl(bottomColor, I2cAddr.create8bit(0x5c), false);

		leftColorReader.engage();
		rightColorReader.engage();
		bottomColorReader.engage();

		leftColorReader.write8(3, 0);
		rightColorReader.write8(3, 0);
		bottomColorReader.write8(3, 0);
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
            rightLauncher.setPower(0.23);
            leftLauncher.setPower(0.23);
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

		telemetry.addData("Bottom: ",""+getColor(bottomColorReader, bottomColorCache));
		telemetry.addData("Left: ",""+getColor(leftColorReader, leftColorCache));
		telemetry.addData("Right: ",""+getColor(rightColorReader, rightColorCache));
		telemetry.addData("Ultrasonic: ", ""+ultrasonic.getUltrasonicLevel());
		telemetry.update();
    }

	@Override
	public void stop() {
		lfd.setPower(0);
		lbd.setPower(0);
		rfd.setPower(0);
		rbd.setPower(0);
		conveyor.setPower(0);
		leftLauncher.setPower(0);
		rightLauncher.setPower(0);
		drawerSlide.setPower(0);
	}

	private int getColor(I2cDeviceSynch device, byte[] arr) {
		arr = device.read(0x04, 1);
		return arr[0]&0xFF;
	}
}
