package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID test")
public  class PIDtest extends LinearOpMode {

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;

    double pulsePerRevolution = 28;

    double targetRPM = 38;

    double integralsum = 0;

    double Kp = 0.05;

    double Ki = 0;

    double Kd = 0;

    double maxVelocity = 2600;
    double Kf = 1/maxVelocity;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() {

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        // run until the end of the match (driver presses STOP)
         while (opModeIsActive()) {
             // Put run blocks here.
            double power = PIDControl(targetRPM * pulsePerRevolution, flywheelRight.getVelocity());
            flywheelRight.setPower(power);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("power", power);
            packet.put("velocity", flywheelRight.getVelocity());
            packet.put("RPMs", flywheelRight.getVelocity() / 28);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private double PIDControl(double reference, double state) {
        double error = reference - state;
        double dt = timer.seconds();
        timer.reset();
        
        integralsum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;


        double output = (error * Kp) + (derivative * Kd) + (integralsum * Ki) + (reference * Kf);
        return output;
    }


}
