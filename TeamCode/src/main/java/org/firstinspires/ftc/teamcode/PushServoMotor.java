package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Benjamin Jensrud on 12/9/2016.
 * A class to simplify the use of pushing servos
 */
public class PushServoMotor {
	private Servo servo;
	private double top, bottom;

	public PushServoMotor(Servo servo, double top, double bottom) {
		this.servo = servo;
		this.top = top;
		this.bottom = bottom;
	}

	public void setToTop() {
		servo.setPosition(top);
	}

	public void setToBottom() {
		servo.setPosition(bottom);
	}
}
