package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;

public class RampageRobot implements Sequence {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private final FlywheelMotorController flywheelLeft;
    private final FlywheelMotorController flywheelRight;

    private final DcMotor feeder;
    private boolean isFeederClosed = false;

    private final DigitalChannel closedLimitSwitch;
    private final DigitalChannel openLimitSwitch;

    public RampageRobot(OpMode opMode) {
        this.frontLeftMotor = getDriveMotor(opMode, Constants.Motors.FrontLeftWheelMotor, DcMotorSimple.Direction.REVERSE);
        this.frontRightMotor = getDriveMotor(opMode, Constants.Motors.FrontRightWheelMotor, DcMotorSimple.Direction.FORWARD);
        this.backLeftMotor = getDriveMotor(opMode, Constants.Motors.BackLeftWheelMotor, DcMotorSimple.Direction.REVERSE);
        this.backRightMotor = getDriveMotor(opMode, Constants.Motors.BackRightWheelMotor, DcMotorSimple.Direction.FORWARD);
        this.flywheelLeft = createFlywheelMotorController(opMode, Constants.Motors.LeftFlywheelMotor, DcMotorSimple.Direction.REVERSE);
        this.flywheelRight = createFlywheelMotorController(opMode, Constants.Motors.RightFlywheelMotor, DcMotorSimple.Direction.FORWARD);
        this.feeder = getFeederMotor(opMode, Constants.Motors.FeederMotor);

        this.closedLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, Constants.Sensors.ClosedLimitSwitch);
        this.openLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, Constants.Sensors.OpenLimitSwitch);
    }

    public void setDriveMotorPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);
    }

    public FeederState getFeederState() {
        if (isFeederClosed) {
            return closedLimitSwitch.getState() ? FeederState.CLOSED : FeederState.CLOSING;
        }
        return openLimitSwitch.getState() ? FeederState.OPEN : FeederState.OPENING;
    }

    public void closeFeeder() {
        isFeederClosed = true;
    }

    public void openFeeder() {
        isFeederClosed = false;
    }

    public void toggleFeeder() {
        if (isFeederClosed) {
            openFeeder();
        } else {
            closeFeeder();
        }
    }

    public FlywheelMotorController getFlywheelLeft() {
        return flywheelLeft;
    }

    public FlywheelMotorController getFlywheelRight() {
        return flywheelRight;
    }

    public void initialize(Context context) {
        context.registerSequence(this);

        flywheelLeft.initialize(context);
        flywheelRight.initialize(context);
    }

    @Override
    public boolean hasCompleted() {
        return false;
    }

    @Override
    public void executeFrame(Context context) {
        switch (getFeederState()) {
            case CLOSING:
                feeder.setPower(Constants.Power.Feeder);
                break;
            case OPENING:
                feeder.setPower(-Constants.Power.Feeder);
                break;
            default:
                feeder.setPower(0);
                break;
        }
    }

    public void startFlywheels(){
        flywheelLeft.start();
        flywheelRight.start();
    }
    public void stopFlywheels(){
        flywheelLeft.stop();
        flywheelRight.stop();
    }

    private DcMotor getDriveMotor(OpMode opMode, String deviceName, DcMotor.Direction direction) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        return motor;
    }

    private DcMotor getFeederMotor(OpMode opMode, String deviceName) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        return motor;
    }

    private FlywheelMotorController createFlywheelMotorController(OpMode opMode, String deviceName, DcMotor.Direction direction) {
        DcMotorEx motor = opMode.hardwareMap.get(DcMotorEx.class, deviceName);
        FlywheelMotorController controller = new FlywheelMotorController(motor);
        controller.init(direction);
        controller.setTargetRpm(1700);
        return controller;
    }
}
