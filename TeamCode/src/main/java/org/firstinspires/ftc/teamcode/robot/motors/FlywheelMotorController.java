package org.firstinspires.ftc.teamcode.robot.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlywheelMotorController {
    private final DcMotorEx motor;

    public FlywheelMotorController(DcMotorEx motor) {
        this.motor = motor;
    }

    public void init(DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    public void setTargetVelocity(double velocity, PIDFCoefficients pidfCoefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        motor.setVelocity(velocity);
    }
}
