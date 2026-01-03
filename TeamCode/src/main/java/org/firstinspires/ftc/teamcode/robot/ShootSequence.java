package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

public class ShootSequence implements Sequence {
    private int count;
    private boolean requiresInitialization = true;
    private boolean isCancelled = false;

    public ShootSequence(int count) {
        this.count = count + 1;
    }

    public void cancel() {
        isCancelled = true;
    }

    @Override
    public boolean hasCompleted() {
        return count <= 0;
    }

    @Override
    public void executeFrame(Context context) {
        if (hasCompleted()) {
            return;
        }

        RampageRobot robot = context.getRobot();

        if (isCancelled) {
            robot.openFeeder();
            count = 0;
            return;
        }

        if (requiresInitialization) {
            robot.openFeeder();
            requiresInitialization = false;
        }

        switch (robot.getFeederState()) {
            case OPEN:
                count--;
                if (!hasCompleted()) {
                    robot.closeFeeder();
                }
                break;
            case CLOSED:
                robot.openFeeder();
                break;
        }
    }
}
