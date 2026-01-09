package org.firstinspires.ftc.teamcode.robot.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlywheelVelocitySettings {
    public final double targetVelocity;
    public final PIDFCoefficients leftFlywheel;
    public final PIDFCoefficients rightFlywheel;

    public static final FlywheelVelocitySettings Stopped = new FlywheelVelocitySettings(0, new PIDFCoefficients(0, 0, 0, 0), new PIDFCoefficients(0, 0, 0, 0));
    public static final FlywheelVelocitySettings Default = new FlywheelVelocitySettings(700, new PIDFCoefficients(130, 0, 0, 12.4), new PIDFCoefficients(130, 0, 0, 12.4));
    public static final FlywheelVelocitySettings Far = new FlywheelVelocitySettings(850, new PIDFCoefficients(93, 0, 0, 11.3), new PIDFCoefficients(93, 0, 0, 11.3));
    public static final FlywheelVelocitySettings Auto = new FlywheelVelocitySettings(640, new PIDFCoefficients(100, 0, 0, 12.92), new PIDFCoefficients(100, 0, 0, 12.92));

    FlywheelVelocitySettings(double targetVelocity, PIDFCoefficients leftFlywheel, PIDFCoefficients rightFlywheel) {
        this.targetVelocity = targetVelocity;
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
    }
}
