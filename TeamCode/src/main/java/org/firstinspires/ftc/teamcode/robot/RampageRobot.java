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

    private final DcMotor feederMotor;
    private boolean isFeederClosed = false;

    private final DigitalChannel openLimitSwitch;

    private final AprilTagWebcam webcam;

    public RampageRobot(OpMode opMode) {
        this.frontLeftMotor = getDriveMotor(opMode, Constants.Motors.FrontLeftWheelMotor, DcMotorSimple.Direction.REVERSE);
        this.frontRightMotor = getDriveMotor(opMode, Constants.Motors.FrontRightWheelMotor, DcMotorSimple.Direction.FORWARD);
        this.backLeftMotor = getDriveMotor(opMode, Constants.Motors.BackLeftWheelMotor, DcMotorSimple.Direction.REVERSE);
        this.backRightMotor = getDriveMotor(opMode, Constants.Motors.BackRightWheelMotor, DcMotorSimple.Direction.FORWARD);
        this.flywheelLeft = createFlywheelMotorController(opMode, Constants.Motors.LeftFlywheelMotor, DcMotorSimple.Direction.REVERSE);
        this.flywheelRight = createFlywheelMotorController(opMode, Constants.Motors.RightFlywheelMotor, DcMotorSimple.Direction.FORWARD);
        this.feederMotor = getFeederMotor(opMode);

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
        if (GlobalState.FeederHomePosition == null) {
            return FeederState.INITIALIZING;
        }
        if (isFeederClosed) {
            return feederMotor.getCurrentPosition() < getTargetFeederClosedPosition() ? FeederState.CLOSING : FeederState.CLOSED;
        }
        return feederMotor.getCurrentPosition() > GlobalState.FeederHomePosition ? FeederState.OPENING : FeederState.OPEN;
    }

    public int getFeederPosition() {
        return feederMotor.getCurrentPosition();
    }

    public void closeFeeder() {
        isFeederClosed = true;
    }

    public void openFeeder() {
        isFeederClosed = false;
    }

    public void toggleFeeder() {
        isFeederClosed = !isFeederClosed;
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
        if (getFeederState() == FeederState.INITIALIZING && openLimitSwitch.getState()) {
            GlobalState.FeederHomePosition = feederMotor.getCurrentPosition();
        }

        switch (getFeederState()) {
            case INITIALIZING:
                feederMotor.setPower(-Constants.Power.FeederCalibration);
                break;
            case OPENING:
                feederMotor.setPower(-Constants.Power.Feeder);
                break;
            case CLOSING:
                feederMotor.setPower(Constants.Power.Feeder);
                break;
            default:
                feederMotor.setPower(0);
                break;
        }
    }

    private int getTargetFeederClosedPosition() {
        return GlobalState.FeederHomePosition + Constants.Feeder.ClosedPositionOffset;
    }

    private DcMotor getDriveMotor(OpMode opMode, String deviceName, DcMotor.Direction direction) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        return motor;
    }

    private DcMotor getFeederMotor(OpMode opMode) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, Constants.Motors.FeederMotor);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
