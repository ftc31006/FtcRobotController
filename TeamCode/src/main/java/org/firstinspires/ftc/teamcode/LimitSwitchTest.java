package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RampageRobot;

@TeleOp
public class LimitSwitchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RampageRobot robot = new RampageRobot(this);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Is Open", robot.isFeederInOpenPosition());
            packet.put("Is Closed", robot.isFeederInClosedPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
