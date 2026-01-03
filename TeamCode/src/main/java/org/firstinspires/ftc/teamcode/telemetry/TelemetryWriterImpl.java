package org.firstinspires.ftc.teamcode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TelemetryWriterImpl implements TelemetryWriter {
    private final OpMode opMode;
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();

    public TelemetryWriterImpl(OpMode opMode) {
        this.opMode = opMode;
    }

    public void update() {
        opMode.telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public void write(String key, Object data) {
        write(String.format("%s: %s", key, data));
    }

    @Override
    public void write(String line) {
        opMode.telemetry.addLine(line);
        telemetryPacket.addLine(line);
    }
}
