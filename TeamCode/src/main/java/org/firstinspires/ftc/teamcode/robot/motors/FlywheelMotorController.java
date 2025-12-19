package org.firstinspires.ftc.teamcode.robot.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robot.pid.VelocityPIDController;

public class FlywheelMotorController {
    private final double Kp = 0.00000001;
    private final double Ki = 0.01;
    private final double Kd = 0;
    private final double maxVelocity = 2600;
    private final double Kf = .98 / maxVelocity;

    private final double pulsePerRevolution = 28;

    private final DcMotorEx motor;
    private double targetRpm = 0;
    private double targetPulsePerSecond = 0;

    private final VelocityPIDController pidController = new VelocityPIDController(Kp, Ki, Kd, Kf);

    private boolean isFlywheelRunning;

    public FlywheelMotorController(DcMotorEx motor) {
        this.motor = motor;
        this.setTargetRpm(0);
    }

    public void init(DcMotor.Direction direction)
    {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    public void setPIDFCoefficients(double p, double i, double d, double f)
    {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetRpm(double targetRpm) {
        if (targetRpm == 0) {
            motor.setPower(0);
        }

        this.targetRpm = targetRpm;
        double targetPulsePerMinute = targetRpm * pulsePerRevolution;
        this.targetPulsePerSecond = targetPulsePerMinute / 60;
    }

    public void start(){
        isFlywheelRunning = true;
    }

    public void stop(){
        isFlywheelRunning = false;
    }

    public DcMotorEx getMotor() {
        return motor;
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
        if (!isFlywheelRunning) {
            this.motor.setPower(0);
            return;
        }
        // Get measured velocity from motor (FTC SDK method)
        double measuredTicksPerSec = this.motor.getVelocity();

        long now = System.nanoTime();
        double power = this.pidController.update(measuredTicksPerSec, targetPulsePerSecond, now);
        this.motor.setPower(power);
    }
}
