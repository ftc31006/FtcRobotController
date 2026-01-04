package org.firstinspires.ftc.teamcode.robot.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.Sequence;
import org.firstinspires.ftc.teamcode.robot.pid.VelocityPIDController;

public class FlywheelMotorController {
    private final double pulsePerRevolution = 28;
    private final DcMotorEx motor;

    public FlywheelMotorController(DcMotorEx motor) {
        this.motor = motor;
    }

    public void init(DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    public void setTargetVelocity(double rpm, PIDFCoefficients pidfCoefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        double pulsePerMinute = rpm * pulsePerRevolution;
        motor.setVelocity(pulsePerMinute);
    }
}
