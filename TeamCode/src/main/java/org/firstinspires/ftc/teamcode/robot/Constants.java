package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

public class Constants {
    public static class Feeder {
        public static final int ClosedPositionOffset = 700;
    }

    public static class Power {
        public static final double Feeder = .85;
        public static final double FeederCalibration = 0.2;
    }

    public static class Motors {
        public static final String FrontLeftWheelMotor = "FrontLeft"; // Control Hub Port 1
        public static final String FrontRightWheelMotor = "FrontRight"; // Control Hub Port 2
        public static final String BackLeftWheelMotor = "BackLeft"; // Control Hub Port 3
        public static final String BackRightWheelMotor = "BackRight"; // Control Hub Port 0
        public static final String LeftFlywheelMotor = "FlywheelLeft"; // Expansion Hub Port 1
        public static final String RightFlywheelMotor = "FlywheelRight"; // Expansion Hub Port 0
        public static final String FeederMotor = "Feeder"; // Expansion Hub Port 2
    }

    public static class Sensors {
        public static final String ClosedLimitSwitch = "ClosedLimitSwitch"; // Expansion Hub Digital Port 1/2
        public static final String OpenLimitSwitch = "OpenLimitSwitch"; // Control Hub Digital Port 1/2
    }

    public static class Cameras {
        public static final String Webcam = "Webcam 1";
    }
}
