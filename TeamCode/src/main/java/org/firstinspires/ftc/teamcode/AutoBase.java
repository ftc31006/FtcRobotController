package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.Sequence;
import org.firstinspires.ftc.teamcode.robot.ShootSequence;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelVelocitySettings;

import java.util.ArrayList;
import java.util.List;


public abstract class AutoBase extends LinearOpMode {
    @Override
    public void runOpMode() {
        RampageRobot robot = new RampageRobot(this);
        List<Sequence> sequences = new ArrayList<>();
        Context context = new Context() {
            @Override
            public RampageRobot getRobot() {
                return robot;
            }

            @Override
            public void registerSequence(Sequence sequence) {
                sequences.add(sequence);
            }

            @Override
            public int getSequenceCount() {
                return sequences.size();
            }
        };

        robot.initialize(context);

        waitForStart();

        robot.setFlywheelVelocity(FlywheelVelocitySettings.Default);

        sleep(1000);
        drive(robot, -0.5,-0.5,-0.5, -0.5, 450);
        sleep(1000);

        ShootSequence feederSequence = new ShootSequence(3);
        context.registerSequence(feederSequence);

        while (opModeIsActive() && !feederSequence.hasCompleted()) {
            for (Sequence sequence: sequences) {
                sequence.executeFrame(context);
            }
        }

        drive(robot, getFrontLeftPower(), getFrontRightPower(), getBackLeftPower(), getBackRightPower(),1250);
        telemetry.update();
    }

    protected abstract double getFrontLeftPower();
    protected abstract double getFrontRightPower();
    protected abstract double getBackLeftPower();
    protected abstract double getBackRightPower();


    private void drive(RampageRobot robot, double frontLeftPower,double frontRightPower,double backLeftPower,double backRightPower,long duration) {
        robot.setDriveMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        sleep(duration);
        robot.setDriveMotorPower(0, 0, 0, 0);
    }

    private void writeTelemetry(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }
}
