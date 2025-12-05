package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servo")
public class HelloWorld extends LinearOpMode {
    public static final double FEEDER_CLOSED_POSITION = 0.25;
    public static final double FEEDER_OPEN_POSITION = 0.82;

    public Servo feeder = null;
    public boolean isFeederOpen = true;

    @Override
    public void runOpMode() {
        feeder = hardwareMap.get(Servo.class, "Feeder");
        feeder.setPosition(FEEDER_OPEN_POSITION);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                isFeederOpen = !isFeederOpen;
                feeder.setPosition(isFeederOpen ? FEEDER_OPEN_POSITION : FEEDER_CLOSED_POSITION);
            }
        }
    }
}
