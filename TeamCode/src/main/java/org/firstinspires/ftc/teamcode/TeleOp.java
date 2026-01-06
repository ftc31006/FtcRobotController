package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.DriveMotorPower;
import org.firstinspires.ftc.teamcode.robot.FeederSequence;
import org.firstinspires.ftc.teamcode.robot.GlobalState;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.ShootSequence;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelVelocitySettings;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryWriter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends RampageOpMode {
    private FeederSequence feederSequence = null;

    @Override
    protected void processInput(Context context) {
        RampageRobot robot = context.getRobot();

        updateDriveMotorPower(context);

        if (gamepad1.xWasPressed()) {
            GlobalState.FeederHomePosition = null;
        }

        if (gamepad1.yWasPressed()) {
            cancelFeederSequence();
        }

        if (gamepad1.aWasPressed()) {
            initiateShootSequence(context, 1);
        }

        if (gamepad1.bWasPressed()) {
            initiateShootSequence(context, 3);
        }

        if (gamepad1.dpadUpWasPressed()) {
            robot.setFlywheelVelocity(FlywheelVelocitySettings.Default);
        }
        if (gamepad1.dpadDownWasPressed()) {
            robot.setFlywheelVelocity(FlywheelVelocitySettings.Stopped);
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

        AprilTagDetection detection = robot.findAprilTag(20);
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

    private void updateDriveMotorPower(Context context) {
        RampageRobot robot = context.getRobot();

        robot.setDriveMotorPower(-gamepad1.left_stick_y, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_stick_x);
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
