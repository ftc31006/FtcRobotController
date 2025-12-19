package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

public class RampageRobot {
    private static final double FEEDER_CLOSED_POSITION = 0.25;
    private static final double FEEDER_OPEN_POSITION = 0.82;

    private final OpMode opMode;

    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private final FlywheelMotorController flywheelLeft;
    private final FlywheelMotorController flywheelRight;

    private final Servo feeder;
    private boolean isFeederClosed = false;

    private final DigitalChannel closedLimitSwitch;
    private final DigitalChannel openLimitSwitch;

    public RampageRobot(OpMode opMode) {
        this.opMode = opMode;
        this.frontLeftMotor = getDriveMotor("FrontLeft", DcMotorSimple.Direction.REVERSE);
        this.frontRightMotor = getDriveMotor("FrontRight", DcMotorSimple.Direction.FORWARD);
        this.backLeftMotor = getDriveMotor("BackLeft", DcMotorSimple.Direction.REVERSE);
        this.backRightMotor = getDriveMotor("BackRight", DcMotorSimple.Direction.FORWARD);
        this.flywheelLeft = createFlywheelMotorController("FlywheelLeft", DcMotorSimple.Direction.REVERSE);
        this.flywheelRight = createFlywheelMotorController("FlywheelRight", DcMotorSimple.Direction.FORWARD);
        this.feeder = opMode.hardwareMap.get(Servo.class, "Feeder");
        this.closedLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "ClosedLimitSwitch");
        this.openLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "OpenLimitSwitch");
    }

    public void setDriveMotorPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);
    }

    public boolean isFeederClosed() {
        return this.isFeederClosed;
    }

    public void closeFeeder() {
        isFeederClosed = true;
        feeder.setPosition(FEEDER_CLOSED_POSITION);
    }

    public void openFeeder() {
        isFeederClosed = false;
        feeder.setPosition(FEEDER_OPEN_POSITION);
    }

    public void toggleFeeder() {
        if (isFeederClosed) {
            openFeeder();
        } else {
            closeFeeder();
        }
    }

    public boolean isFeederInOpenPosition() { return openLimitSwitch.getState(); }

    public boolean isFeederInClosedPosition() { return closedLimitSwitch.getState(); }

    public FlywheelMotorController getFlywheelLeft() {
        return flywheelLeft;
    }

    public FlywheelMotorController getFlywheelRight() {
        return flywheelRight;
    }

    public void update() {
        flywheelLeft.update();
        flywheelRight.update();
    }
    public void startFlywheels(){
        flywheelLeft.start();
        flywheelRight.start();
    }
    public void stopFlywheels(){
        flywheelLeft.stop();
        flywheelRight.stop();
    }
    private DcMotor getDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        return motor;
    }

    private FlywheelMotorController createFlywheelMotorController(String deviceName, DcMotor.Direction direction) {
        DcMotorEx motor = opMode.hardwareMap.get(DcMotorEx.class, deviceName);
        FlywheelMotorController controller = new FlywheelMotorController(motor);
        controller.init(direction);
        controller.setTargetRpm(1700);
        return controller;
    }
}
