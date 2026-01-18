package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.geometry.TargetLocator;
import org.firstinspires.ftc.teamcode.pid.AprilTagAimingController;
import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.DriveMotorPower;
import org.firstinspires.ftc.teamcode.robot.LEDSequence;
import org.firstinspires.ftc.teamcode.robot.LEDState;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryWriter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "April Tag")
public class AprilTagTest extends RampageOpMode {
    private final TargetLocator targetLocator = new TargetLocator(22);
    private final AprilTagAimingController aimingController = new AprilTagAimingController(0);
    private final LEDSequence ledSequence = new LEDSequence();

    private double P = 0;
    private double turnSpeed = .2;

    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    protected void onStart(Context context) {
        context.registerSequence(ledSequence);
        super.onStart(context);
    }

    @Override
    protected void processInput(Context context) {
        RampageRobot robot = context.getRobot();

        if (gamepad1.dpadLeftWasPressed()) {
            stepIndex -= 1;
            if (stepIndex < 0) {
                stepIndex = stepSizes.length - 1;
            }
        }
        if (gamepad1.dpadRightWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
            turnSpeed -= stepSizes[stepIndex];
            aimingController.setKp(P);
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
            turnSpeed += stepSizes[stepIndex];
            aimingController.setKp(P);
        }

        updateDriveMotorPower(context);
    }

    @Override
    protected void writeTelemetry(Context context, TelemetryWriter writer) {
        RampageRobot robot = context.getRobot();

        writer.write("Step Size", stepSizes[stepIndex]);
        writer.write("Kp", P);
        writer.write("Turn Speed", turnSpeed);

        DriveMotorPower driveMotorPower = robot.getDriveMotorPower();
        writer.write("Front Left Wheel Power", driveMotorPower.frontLeft);
        writer.write("Front Right Wheel Power", driveMotorPower.frontRight);
        writer.write("Back Left Wheel Power", driveMotorPower.backLeft);
        writer.write("Back Right Wheel Power", driveMotorPower.backRight);
        writer.write("");
        writer.write("April Tag LED State", robot.getAprilTagLEDState());

        AprilTagDetection detection = robot.findAprilTag(20);
        if (detection != null) {
            if (detection.metadata != null) {
                writer.write(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                writer.write(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                writer.write(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                writer.write(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                writer.write("Target Angle", targetLocator.getAngle(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw));
            } else {
                writer.write(String.format("\n==== (ID %d) Unknown", detection.id));
                writer.write(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

    private void updateDriveMotorPower(Context context) {
        RampageRobot robot = context.getRobot();

        double turn = gamepad1.right_stick_x;
        LEDState aprilTagState = LEDState.OFF;
        Integer frequency = null;

        AprilTagDetection detection = robot.findAprilTag(20);
        if (detection != null && detection.metadata != null) {
            double angle = targetLocator.getAngle(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);
//                turn = aimingController.calculate(angle, 0);

            aprilTagState = LEDState.RED;

            if (Math.abs(angle) < 3) {
                aprilTagState = LEDState.GREEN;
            } else if (gamepad1.x) {
                turn = angle < 0 ? -turnSpeed : turnSpeed;
                frequency = 100;
            }
        }

        double slowmo = gamepad1.right_bumper ? .35 : 1;

        double frontLeft = turn * slowmo;

        double frontRight = -turn * slowmo;

        double backLeft = turn * slowmo;

        double backRight = -turn * slowmo;

        robot.setDriveMotorPower(frontLeft, frontRight, backLeft, backRight);
        ledSequence.setState(aprilTagState, frequency);
    }
}
