package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.geometry.TargetLocator;
import org.firstinspires.ftc.teamcode.pid.AprilTagAimingController;
import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.DriveMotorPower;
import org.firstinspires.ftc.teamcode.robot.FeederSequence;
import org.firstinspires.ftc.teamcode.robot.GlobalState;
import org.firstinspires.ftc.teamcode.robot.LEDSequence;
import org.firstinspires.ftc.teamcode.robot.LEDState;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.ShootSequence;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelVelocitySettings;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryWriter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends RampageOpMode {
    private final TargetLocator targetLocator = new TargetLocator(22);
    private final LEDSequence ledSequence = new LEDSequence();
    private FeederSequence feederSequence = null;
    private String distance;
    private final double autoAimTurnSpeed = .2;

    @Override
    protected void onStart(Context context) {
        RampageRobot robot = context.getRobot();

        context.registerSequence(ledSequence);
        robot.setFlywheelVelocity(FlywheelVelocitySettings.Default);
        distance = "Near";
    }

    @Override
    protected void processInput(Context context) {
        RampageRobot robot = context.getRobot();

        Double turnOverride = updateAutoAimingDetails(context);
        updateDriveMotorPower(context, turnOverride);

        if (gamepad2.leftBumperWasPressed() ){
           robot.setFlywheelVelocity(FlywheelVelocitySettings.Default);
            distance = "Near";
        }

        if (gamepad2.rightBumperWasPressed()) {
            robot.setFlywheelVelocity(FlywheelVelocitySettings.Far);
            distance = "Far";
        }

        if (gamepad2.yWasPressed() || gamepad1.yWasPressed()) {
            cancelFeederSequence();
        }

        if (gamepad2.aWasPressed()) {
            initiateShootSequence(context, 1);
        }

        if (gamepad2.bWasPressed()) {
            initiateShootSequence(context, 3);
        }
    }

    @Override
    protected void writeTelemetry(Context context, TelemetryWriter writer) {
        RampageRobot robot = context.getRobot();

        writer.write("Feeder State", robot.getFeederState());
        writer.write("Feeder Home Position", GlobalState.FeederHomePosition);
        writer.write("Feeder Current Position", robot.getFeederPosition());
        writer.write("Sequence Count", context.getSequenceCount());

        DriveMotorPower driveMotorPower = robot.getDriveMotorPower();
        writer.write("Front Left Wheel Power", driveMotorPower.frontLeft);
        writer.write("Front Right Wheel Power", driveMotorPower.frontRight);
        writer.write("Back Left Wheel Power", driveMotorPower.backLeft);
        writer.write("Back Right Wheel Power", driveMotorPower.backRight);

        writer.write("Distance", distance);

        AprilTagDetection detection = robot.getClosestTagById(20, 24);
        if (detection != null) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

    private Double updateAutoAimingDetails(Context context) {
        RampageRobot robot = context.getRobot();

        LEDState aprilTagState = LEDState.OFF;
        Double turn = null;
        Integer frequency = null;

        AprilTagDetection detection = robot.getClosestTagById(20, 24);
        if (detection != null && detection.metadata != null) {
            double angle = targetLocator.getAngle(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);

            aprilTagState = LEDState.RED;

            if (Math.abs(angle) < 3) {
                aprilTagState = LEDState.GREEN;
                if (gamepad1.x) {
                    frequency = 100;
                    turn = 0.0;
                }
            } else if (gamepad1.x) {
                turn = angle < 0 ? -autoAimTurnSpeed : autoAimTurnSpeed;
                frequency = 100;
            }
        }

        ledSequence.setState(aprilTagState, frequency);

        return turn;
    }

    private void updateDriveMotorPower(Context context, Double turnOverride) {
        RampageRobot robot = context.getRobot();

        double slowmo = gamepad1.right_bumper ? .35 : 1;

        double forwardBack = -gamepad1.left_stick_y * slowmo;

        double strafe = gamepad1.left_stick_x * slowmo;

        double turn = turnOverride == null ? gamepad1.right_stick_x * slowmo : turnOverride;

        double frontLeft = forwardBack + strafe + turn;

        double frontRight = forwardBack - strafe - turn;

        double backLeft = forwardBack - strafe + turn;

        double backRight = forwardBack + strafe - turn;

        robot.setDriveMotorPower(frontLeft, frontRight, backLeft, backRight);
    }

    private void initiateShootSequence(Context context, int count) {
        cancelFeederSequence();
        feederSequence = new ShootSequence(count);
        context.registerSequence(feederSequence);
    }

    private void cancelFeederSequence() {
        if (feederSequence == null) {
            return;
        }

        feederSequence.cancel();
        feederSequence = null;
    }
}
