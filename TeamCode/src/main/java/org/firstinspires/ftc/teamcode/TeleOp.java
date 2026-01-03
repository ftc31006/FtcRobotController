package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.ShootSequence;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends RampageOpMode {
    private ShootSequence shootSequence = null;

    @Override
    protected void processInput(Context context) {
        RampageRobot robot = context.getRobot();

        if (gamepad1.yWasPressed()) {
            cancelShootSequence();
        }

        if (gamepad1.aWasPressed()) {
            initiateShootSequence(context, 1);
        }

        if (gamepad1.bWasPressed()) {
            initiateShootSequence(context, 3);
        }
    }

    @Override
    protected void writeTelemetry(Context context, TelemetryPacket packet) {
        RampageRobot robot = context.getRobot();

        packet.put("Feeder State", robot.getFeederState());
        packet.put("Sequence Count", context.getSequenceCount());
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
