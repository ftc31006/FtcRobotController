package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto Red")
public class RedAuto extends AutoBase {
    @Override
    protected double getFrontLeftPower() {
        return 1;
    }

    @Override
    protected double getFrontRightPower() {
        return -1;
    }

    @Override
    protected double getBackLeftPower() {
        return -1;
    }

    @Override
    protected double getBackRightPower() {
        return 1;
    }
}
