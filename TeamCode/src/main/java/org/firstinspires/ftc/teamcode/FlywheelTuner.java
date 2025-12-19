package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

@TeleOp(name = "Flywheel Tuner")
public class FlywheelTuner extends LinearOpMode {
    private final double[] AVAILABLE_VELOCITIES = { 900, 1500 };
    private int currentTargetVelocityIndex = 1;
    private double F = 0;
    private double P = 0;

    private final double[] stepSizes = { 10.0, 1.0, 0.1, 0.001, 0.0001 };
    private int stepIndex = 1;

    private boolean isTuningLeftFlywheel = true;

    @Override
    public void runOpMode() {
        RampageRobot robot = new RampageRobot(this);

        waitForStart();

        while (opModeIsActive()) {
            processInput(robot);
        }
    }

    private void processInput(RampageRobot robot) {
        if (gamepad1.aWasPressed()) {
            isTuningLeftFlywheel = !isTuningLeftFlywheel;
        }

        if (gamepad1.yWasPressed()) {
            currentTargetVelocityIndex = (currentTargetVelocityIndex + 1) % AVAILABLE_VELOCITIES.length;
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            F += stepSizes[stepIndex];
        }

        double targetVelocity = AVAILABLE_VELOCITIES[currentTargetVelocityIndex];

        FlywheelMotorController targetMotor = isTuningLeftFlywheel ? robot.getFlywheelLeft() : robot.getFlywheelRight();
        FlywheelMotorController otherMotor = isTuningLeftFlywheel ? robot.getFlywheelRight() : robot.getFlywheelLeft();
        otherMotor.setTargetRpm(0);

        targetMotor.setPIDFCoefficients(P, 0, 0, F);
        targetMotor.setTargetRpm(targetVelocity);

        double currentVelocity = robot.getFlywheelLeft().getMeasuredPulsePerSecond();
        double currentError = targetVelocity - currentVelocity;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Motor", isTuningLeftFlywheel ? "Left" : "Right");
        packet.put("Target Velocity", targetVelocity);
        packet.put("Current Velocity", currentVelocity);
        packet.put("Error", currentError);
        packet.addLine("-------------------------");
        packet.put("Tuning P", P);
        packet.put("Tuning F", F);
        packet.put("Step Size", stepSizes[stepIndex]);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
