package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelMotorController;
import org.firstinspires.ftc.teamcode.robot.motors.FlywheelVelocitySettings;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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

    private final AprilTagWebcam webcam;

    public RampageRobot(OpMode opMode) {
        this.frontLeftMotor = getDriveMotor(opMode, Constants.Motors.FrontLeftWheelMotor, DcMotorSimple.Direction.REVERSE);
        this.frontRightMotor = getDriveMotor(opMode, Constants.Motors.FrontRightWheelMotor, DcMotorSimple.Direction.FORWARD);
        this.backLeftMotor = getDriveMotor(opMode, Constants.Motors.BackLeftWheelMotor, DcMotorSimple.Direction.REVERSE);
        this.backRightMotor = getDriveMotor(opMode, Constants.Motors.BackRightWheelMotor, DcMotorSimple.Direction.FORWARD);
        this.flywheelLeft = createFlywheelMotorController(opMode, Constants.Motors.LeftFlywheelMotor, DcMotorSimple.Direction.REVERSE);
        this.flywheelRight = createFlywheelMotorController(opMode, Constants.Motors.RightFlywheelMotor, DcMotorSimple.Direction.FORWARD);
        this.feeder = getFeederMotor(opMode);

        this.closedLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, Constants.Sensors.ClosedLimitSwitch);
        this.openLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, Constants.Sensors.OpenLimitSwitch);

        this.webcam = new AprilTagWebcam(opMode.hardwareMap.get(WebcamName.class, Constants.Cameras.Webcam));
    }

    public void setDriveMotorPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);
    }

    public DriveMotorPower getDriveMotorPower() {
        return new DriveMotorPower(frontLeftMotor.getPower(), frontRightMotor.getPower(), backLeftMotor.getPower(), backRightMotor.getPower());
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

    public void setFlywheelVelocity(FlywheelVelocitySettings velocity) {
        flywheelLeft.setTargetVelocity(velocity.targetVelocity, velocity.leftFlywheel);
        flywheelRight.setTargetVelocity(velocity.targetVelocity, velocity.rightFlywheel);
    }

    public void initialize(Context context) {
        context.registerSequence(this);
    }

    public AprilTagDetection findAprilTag(int id) {
        return webcam.getTagById(id);
    }

    public void initializeFrame() {
        webcam.update();
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

    private DcMotor getDriveMotor(OpMode opMode, String deviceName, DcMotor.Direction direction) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        return motor;
    }

    private DcMotor getFeederMotor(OpMode opMode) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, Constants.Motors.FeederMotor);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        return motor;
    }

    private FlywheelMotorController createFlywheelMotorController(OpMode opMode, String deviceName, DcMotor.Direction direction) {
        DcMotorEx motor = opMode.hardwareMap.get(DcMotorEx.class, deviceName);
        FlywheelMotorController controller = new FlywheelMotorController(motor);
        controller.init(direction);
        return controller;
    }
}
