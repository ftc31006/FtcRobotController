package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pid.VelocityPIDController;

@TeleOp(name = "PID test")
public class PIDtest extends LinearOpMode {

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;

    private final double pulsePerRevolution = 28;

    private double targetRPM = 1700;

    private double integralsum = 0;

    private final double Kp = 0.0001;

    private final double Ki = 0;

    private final double Kd = 0;

    private final double maxVelocity = 2600;
    private final double Kf = 1 / maxVelocity;

    @Override
    public void runOpMode() {
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
//        flywheelRight = hardwareMap.get(DcMotorEx.class, "FlywheelRight");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
//        flywheelRight.setDirection(DcMotor.Direction.FORWARD);

        VelocityPIDController flywheelLeftPidController = new VelocityPIDController(Kp, Ki, Kd, Kf);

        double targetPulsePerMinute = targetRPM * pulsePerRevolution;
        double targetPulsePerSecond = targetPulsePerMinute / 60;

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            // Get measured velocity from motor (FTC SDK method)
            double measuredTicksPerSec = flywheelLeft.getVelocity();

            long now = System.nanoTime();
            double power = flywheelLeftPidController.update(measuredTicksPerSec, targetPulsePerSecond, now);
            flywheelLeft.setPower(power);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Desired Ticks/Sec", targetPulsePerSecond);
            packet.put("Measured Ticks/Sec", measuredTicksPerSec);
            packet.put("Power", power);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
