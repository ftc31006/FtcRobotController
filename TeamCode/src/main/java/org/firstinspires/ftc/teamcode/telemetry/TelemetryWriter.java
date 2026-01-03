package org.firstinspires.ftc.teamcode.telemetry;

public interface TelemetryWriter {
    void write(String key, Object data);
    void write(String line);
}
