package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

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
    protected void writeTelemetry(Context context, TelemetryPacket packet) {
        RampageRobot robot = context.getRobot();

        packet.put("Left Flywheel Desired Ticks/Sec", robot.getFlywheelLeft().getTargetPulsePerSecond());
        packet.put("Left Flywheel Measured Ticks/Sec", robot.getFlywheelLeft().getMeasuredPulsePerSecond());
        packet.put("Left Flywheel Power", robot.getFlywheelLeft().getPower());
        packet.put("Right Flywheel Desired Ticks/Sec", robot.getFlywheelRight().getTargetPulsePerSecond());
        packet.put("Right Flywheel Measured Ticks/Sec", robot.getFlywheelRight().getMeasuredPulsePerSecond());
        packet.put("Right Flywheel Power", robot.getFlywheelRight().getPower());
        packet.put("Feeder State", robot.getFeederState());
    }
}
