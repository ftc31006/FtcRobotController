package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

public class Constants {
    public static class Power {
        public static final double Feeder = 0.3;
    }

    public static class Motors {
        public static final String FrontLeftWheelMotor = "FrontLeft";
        public static final String FrontRightWheelMotor = "FrontRight";
        public static final String BackLeftWheelMotor = "BackLeft";
        public static final String BackRightWheelMotor = "BackRight";
        public static final String LeftFlywheelMotor = "FlywheelLeft";
        public static final String RightFlywheelMotor = "FlywheelRight";
        public static final String FeederMotor = "Feeder";
    }

    public static class Sensors {
        public static final String ClosedLimitSwitch = "ClosedLimitSwitch";
        public static final String OpenLimitSwitch = "OpenLimitSwitch";
    }
}
