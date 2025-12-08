package org.firstinspires.ftc.teamcode.robot.motors;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.pid.VelocityPIDController;

public class FlywheelMotorController {
    private final double Kp = 0.0001;
    private final double Ki = 0;
    private final double Kd = 0;
    private final double maxVelocity = 2600;
    private final double Kf = 1 / maxVelocity;

    private final double pulsePerRevolution = 28;

    private final DcMotorEx motor;
    private double targetRpm = 0;
    private double targetPulsePerSecond = 0;

    private final VelocityPIDController pidController = new VelocityPIDController(Kp, Ki, Kd, Kf);


    public FlywheelMotorController(DcMotorEx motor) {
        this.motor = motor;
        this.setTargetRpm(0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetRpm(double targetRpm) {
        this.targetRpm = targetRpm;
        double targetPulsePerMinute = targetRpm * pulsePerRevolution;
        this.targetPulsePerSecond = targetPulsePerMinute / 60;
    }

    public double getTargetPulsePerSecond() {
        return this.targetPulsePerSecond;
    }

    public double getMeasuredPulsePerSecond() {
        return this.motor.getVelocity();
    }

    public double getPower() {
        return this.motor.getPower();
    }

    public void update() {
        // Get measured velocity from motor (FTC SDK method)
        double measuredTicksPerSec = this.motor.getVelocity();

        long now = System.nanoTime();
        double power = this.pidController.update(measuredTicksPerSec, targetPulsePerSecond, now);
        this.motor.setPower(power);
    }
}
