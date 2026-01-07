package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.ShootSequence;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelVelocitySettings;


public abstract class AutoBase extends RampageOpMode {
    @Override
    protected void executeOpMode(Context context) {
        RampageRobot robot = context.getRobot();

        robot.setFlywheelVelocity(FlywheelVelocitySettings.Default);

        sleep(1000);
        drive(robot, -0.5,-0.5,-0.5, -0.5, 450);
        sleep(1000);

        ShootSequence feederSequence = new ShootSequence(3);
        context.registerSequence(feederSequence);

        while (opModeIsActive() && !feederSequence.hasCompleted()) {
            context.executeFrame();
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
}
