package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Context;
import org.firstinspires.ftc.teamcode.robot.RampageRobot;
import org.firstinspires.ftc.teamcode.robot.Sequence;

import java.util.ArrayList;
import java.util.List;

public abstract class RampageOpMode extends LinearOpMode {

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

        while (opModeIsActive()) {
            processInput(context);

            sequences.removeIf(Sequence::hasCompleted);

            for (Sequence sequence: sequences) {
                sequence.executeFrame(context);
            }

            TelemetryPacket packet = new TelemetryPacket();
            writeTelemetry(context, packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    protected abstract void processInput(Context context);

    protected void writeTelemetry(Context context, TelemetryPacket packet) { }
}
