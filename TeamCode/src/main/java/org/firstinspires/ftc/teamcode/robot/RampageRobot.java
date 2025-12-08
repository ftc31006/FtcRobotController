package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

public class RampageRobot {
    private final OpMode opMode;
    private final FlywheelMotorController flywheelLeft;
    private final FlywheelMotorController flywheelRight;

    public RampageRobot(OpMode opMode) {
        this.opMode = opMode;
        this.flywheelLeft = createFlywheelMotorController("FlywheelLeft", DcMotorSimple.Direction.REVERSE);
        this.flywheelRight = createFlywheelMotorController("FlywheelRight", DcMotorSimple.Direction.FORWARD);
    }

    public FlywheelMotorController getFlywheelLeft() {
        return flywheelLeft;
    }

    public FlywheelMotorController getFlywheelRight() {
        return flywheelRight;
    }

    public void update() {
        this.flywheelLeft.update();
        this.flywheelRight.update();
    }

    private FlywheelMotorController createFlywheelMotorController(String deviceName, DcMotor.Direction direction) {
        DcMotorEx motor = opMode.hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        FlywheelMotorController controller = new FlywheelMotorController(motor);
        controller.setTargetRpm(1700);
        return controller;
    }
}
