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
    protected void onStart(Context context) {
        RampageRobot robot = context.getRobot();

        robot.setFlywheelVelocity(FlywheelVelocitySettings.Default);
    }

    @Override
    protected void processInput(Context context) {
        RampageRobot robot = context.getRobot();

        updateDriveMotorPower(context);

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

        double forwardBack = -gamepad1.left_stick_y;

        double strafe = gamepad1.left_stick_x;

        double turn = gamepad1.right_stick_x;

        double slowmo = gamepad1.right_bumper ? .35 : 1;

        double frontLeft = (forwardBack + strafe + turn) * slowmo;

        double frontRight = (forwardBack - strafe - turn) * slowmo;

        double backLeft = (forwardBack - strafe + turn) * slowmo;

        double backRight = (forwardBack + strafe - turn) * slowmo;

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
