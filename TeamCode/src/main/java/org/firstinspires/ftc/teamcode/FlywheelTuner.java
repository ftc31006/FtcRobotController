package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.Sequence;
import org.firstinspires.ftc.teamcode.robot.ShootSequence;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Flywheel Tuner")
public class FlywheelTuner extends RampageOpMode {
    private DcMotorEx leftFlywheelMotor;
    private DcMotorEx rightFlywheelMotor;
    private final double highVelocity = 860;
    private final double lowVelocity = 300;
    private double curTargetVelocity = highVelocity;

    private double F = 0;
    private double P = 0;

    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    public void runOpMode() {
        leftFlywheelMotor = hardwareMap.get(DcMotorEx.class, Constants.Motors.LeftFlywheelMotor);
        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFlywheelMotor = hardwareMap.get(DcMotorEx.class, Constants.Motors.RightFlywheelMotor);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftFlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init complete");

        super.runOpMode();
    }

    @Override
    protected void processInput(Context context) {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
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
            P += stepSizes[stepIndex];
        }

        if (gamepad1.rightBumperWasPressed()) {
            context.registerSequence(new ShootSequence(1));
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftFlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        leftFlywheelMotor.setVelocity(curTargetVelocity);
        rightFlywheelMotor.setVelocity(curTargetVelocity);

        telemetry.addLine("Left Motor");
        telemetry.addLine("-----------------------------");
        addTelemetry(leftFlywheelMotor);
        telemetry.addLine("-----------------------------");
        telemetry.addLine("-----------------------------");
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Right Motor");
        telemetry.addLine("-----------------------------");
        addTelemetry(rightFlywheelMotor);
    }

    private void addTelemetry(DcMotorEx motor) {
        double curVelocity = motor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
