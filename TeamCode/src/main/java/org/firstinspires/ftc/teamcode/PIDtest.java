package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

@TeleOp(name = "PID test")
public class PIDtest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RampageRobot robot = new RampageRobot(this);

        waitForStart();

        while (opModeIsActive()) {
            processInput(robot);

            robot.update();

            updateTelemetry(robot);
        }
    }

    private void processInput(RampageRobot robot) {
        if (gamepad1.aWasPressed()) {
            robot.toggleFeeder();
        }
//        robot.setDriveMotorPower(0.1, 0.1, 0.1, 0.1);
    }

    private void updateTelemetry(RampageRobot robot) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left Flywheel Desired Ticks/Sec", robot.getFlywheelLeft().getTargetPulsePerSecond());
        packet.put("Left Flywheel Measured Ticks/Sec", robot.getFlywheelLeft().getMeasuredPulsePerSecond());
        packet.put("Left Flywheel Power", robot.getFlywheelLeft().getPower());
        packet.put("Right Flywheel Desired Ticks/Sec", robot.getFlywheelRight().getTargetPulsePerSecond());
        packet.put("Right Flywheel Measured Ticks/Sec", robot.getFlywheelRight().getMeasuredPulsePerSecond());
        packet.put("Right Flywheel Power", robot.getFlywheelRight().getPower());
        packet.put("Is Feeder Closed", robot.isFeederClosed());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
