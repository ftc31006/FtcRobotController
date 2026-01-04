package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.DriveMotorPower;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.ShootSequence;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelVelocitySettings;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryWriter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends RampageOpMode {
    private ShootSequence shootSequence = null;

    @Override
    protected void processInput(Context context) {
        RampageRobot robot = context.getRobot();

        updateDriveMotorPower(context);

        if (gamepad1.yWasPressed()) {
            cancelShootSequence();
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
        writer.write("Sequence Count", context.getSequenceCount());

        DriveMotorPower driveMotorPower = robot.getDriveMotorPower();
        writer.write("Front Left Wheel Power", driveMotorPower.frontLeft);
        writer.write("Front Right Wheel Power", driveMotorPower.frontRight);
        writer.write("Back Left Wheel Power", driveMotorPower.backLeft);
        writer.write("Back Right Wheel Power", driveMotorPower.backRight);
    }

    private void updateDriveMotorPower(Context context) {
        RampageRobot robot = context.getRobot();

        robot.setDriveMotorPower(-gamepad1.left_stick_y, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_stick_x);
    }

    private void initiateShootSequence(Context context, int count) {
        cancelShootSequence();
        shootSequence = new ShootSequence(count);
        context.registerSequence(shootSequence);
    }

    private void cancelShootSequence() {
        if (shootSequence == null) {
            return;
        }

        shootSequence.cancel();
        shootSequence = null;
    }
}
