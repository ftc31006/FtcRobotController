package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryWriter;

@TeleOp(name = "PID test")
public class PIDtest extends RampageOpMode {
    @Override
    protected void processInput(Context context) {
        RampageRobot robot = context.getRobot();

        if (gamepad1.aWasPressed()) {
            robot.startFlywheels();
        }
        if (gamepad1.bWasPressed()){
            robot.stopFlywheels();
        }
//        robot.setDriveMotorPower(0.1, 0.1, 0.1, 0.1);
    }

    @Override
    protected void writeTelemetry(Context context, TelemetryWriter writer) {
        RampageRobot robot = context.getRobot();

        writer.write("Left Flywheel Desired Ticks/Sec", robot.getFlywheelLeft().getTargetPulsePerSecond());
        writer.write("Left Flywheel Measured Ticks/Sec", robot.getFlywheelLeft().getMeasuredPulsePerSecond());
        writer.write("Left Flywheel Power", robot.getFlywheelLeft().getPower());
        writer.write("Right Flywheel Desired Ticks/Sec", robot.getFlywheelRight().getTargetPulsePerSecond());
        writer.write("Right Flywheel Measured Ticks/Sec", robot.getFlywheelRight().getMeasuredPulsePerSecond());
        writer.write("Right Flywheel Power", robot.getFlywheelRight().getPower());
        writer.write("Feeder State", robot.getFeederState());
    }
}
