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
            robot.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Left Desired Ticks/Sec", robot.getFlywheelLeft().getTargetPulsePerSecond());
            packet.put("Left Measured Ticks/Sec", robot.getFlywheelLeft().getMeasuredPulsePerSecond());
            packet.put("Left Power", robot.getFlywheelLeft().getPower());
            packet.put("Right Desired Ticks/Sec", robot.getFlywheelRight().getTargetPulsePerSecond());
            packet.put("Right Measured Ticks/Sec", robot.getFlywheelRight().getMeasuredPulsePerSecond());
            packet.put("Right Power", robot.getFlywheelRight().getPower());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
