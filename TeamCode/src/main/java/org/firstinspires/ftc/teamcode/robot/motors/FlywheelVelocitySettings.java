package org.firstinspires.ftc.teamcode.robot.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlywheelVelocitySettings {
    public final double targetVelocity;
    public final PIDFCoefficients leftFlywheel;
    public final PIDFCoefficients rightFlywheel;

    public static final FlywheelVelocitySettings Stopped = new FlywheelVelocitySettings(0, new PIDFCoefficients(0, 0, 0, 0), new PIDFCoefficients(0, 0, 0, 0));

    public static final FlywheelVelocitySettings[] Velocities = {
        Stopped,
        new FlywheelVelocitySettings(1500, new PIDFCoefficients(0, 0, 0, 0), new PIDFCoefficients(0, 0, 0, 0))
    };

    FlywheelVelocitySettings(double targetVelocity, PIDFCoefficients leftFlywheel, PIDFCoefficients rightFlywheel) {
        this.targetVelocity = targetVelocity;
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
    }
}
