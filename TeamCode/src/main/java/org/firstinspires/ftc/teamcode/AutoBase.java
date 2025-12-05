package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class AutoBase extends LinearOpMode {
    private static final double FEEDER_CLOSED_POSITION = 0.25;
    private static final double FEEDER_OPEN_POSITION = 0.82;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor flywheelLeft;
    private DcMotor flywheelRight;

    private Servo feeder = null;
    private boolean isFeederOpen = true;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        flywheelLeft = hardwareMap.get(DcMotor.class, "FlywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "FlywheelRight");
        feeder = hardwareMap.get(Servo.class, "Feeder");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        // shoot motors
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            feeder.setPosition(FEEDER_OPEN_POSITION);
            // Put run blocks here.
            flywheelLeft.setPower(0.34);
            flywheelRight.setPower(0.34);
            sleep(1000);
            drive(-0.5,-0.5,-0.5, -0.5, 450 );
            sleep(1000);
            shoot();
            shoot();
            shoot();
            drive(getFrontLeftPower(),getFrontRightPower(),getBackLeftPower(),getBackRightPower(),1250);
            telemetry.update();
        }
    }

    protected abstract double getFrontLeftPower();
    protected abstract double getFrontRightPower();
    protected abstract double getBackLeftPower();
    protected abstract double getBackRightPower();


    private void drive(double frontLeftPower,double frontRightPower,double backLeftPower,double backRighePower,long duration) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRighePower);
        sleep(duration);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
    }
    private void shoot() {
        feeder.setPosition(FEEDER_CLOSED_POSITION);
        sleep(2000);
        feeder.setPosition(FEEDER_OPEN_POSITION);
        sleep(2000);
        telemetry.addData("FeederPosition", feeder.getPosition());
        telemetry.update();
    }
}
