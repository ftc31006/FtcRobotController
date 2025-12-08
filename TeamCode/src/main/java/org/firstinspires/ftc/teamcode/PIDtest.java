package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.motors.FlywheelMotorController;

@TeleOp(name = "PID test")
public class PIDtest extends LinearOpMode {
    @Override
    public void runOpMode() {
        FlywheelMotorController flywheelLeft = createFlywheelMotorController("FlywheelLeft", DcMotorSimple.Direction.REVERSE);
        FlywheelMotorController flywheelRight = createFlywheelMotorController("FlywheelRight", DcMotorSimple.Direction.FORWARD);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            flywheelLeft.update();
            flywheelRight.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Desired Ticks/Sec", flywheelLeft.getTargetPulsePerSecond());
            packet.put("Measured Ticks/Sec", flywheelLeft.getMeasuredPulsePerSecond());
            packet.put("Power", flywheelLeft.getPower());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private FlywheelMotorController createFlywheelMotorController(String deviceName, DcMotor.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        FlywheelMotorController controller = new FlywheelMotorController(motor);
        controller.setTargetRpm(1700);
        return controller;
    }
}
