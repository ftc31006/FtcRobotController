package org.firstinspires.ftc.teamcode.robot.pid;

public class VelocityPIDController {
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    // Integral windup/clamping
    private double integral = 0.0;
    private static final double integralMin = -1.0;
    private static final double integralMax = 1.0;

    // Derivative low-pass filter (alpha between 0..1)
    private static final double dFilterAlpha = 0.1;
    private double lastDerivative = 0.0;

    // For discrete differentiation
    private double lastError = 0.0;
    private double lastSetpoint = 0.0;

    // Output limits (motor power limits -1..1 typical)
    private static final double outputMin = -1.0;
    private static final double outputMax = 1.0;

    // Smoothing/limiting on setpoint (optional)
    private static final double maxSetpointChangePerSec = Double.POSITIVE_INFINITY;

    // Time helpers
    private long lastTimestampNanos = -1;

    public VelocityPIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double update(double measuredVelocityTicksPerSec, double desiredVelocityTicksPerSec, long timestampNanos) {
        double dt = computeDtSeconds(timestampNanos);
        if (dt <= 0) {
            // first call or invalid dt
            lastSetpoint = desiredVelocityTicksPerSec;
            lastTimestampNanos = timestampNanos;
            return 0.0;
        }

        double maxDelta = maxSetpointChangePerSec * dt;
        double setpoint = limitChange(lastSetpoint, desiredVelocityTicksPerSec, maxDelta);


        // Error (setpoint - measurement)
        double error = setpoint - measuredVelocityTicksPerSec;

        // Proportional
        double pTerm = kP * error;

        // Integral with conditional integration (simple anti-windup: stop integrating when output saturated)
        integral += error * dt;
        integral = clamp(integral, integralMin, integralMax);
        double iTerm = kI * integral;

        // Derivative on error (filtered)
        double rawDerivative = (error - lastError) / dt;
        double derivativeFiltered = dFilterAlpha * rawDerivative + (1 - dFilterAlpha) * lastDerivative;
        double dTerm = kD * derivativeFiltered;

        lastError = error;
        lastDerivative = derivativeFiltered;
        lastSetpoint = setpoint;

        // Feedforward
        double ff = kF * setpoint;

        // Output before clipping
        double output = pTerm + iTerm + dTerm + ff;

        // Clip and anti-windup: if clipped, optionally remove last integral increment (simple)
        double clipped = clamp(output, outputMin, outputMax);
        if (clipped != output) {
            // If you want stronger anti-windup, uncomment: integral -= error * dt; // undo last integral change
            output = clipped;
        }

        return output;
    }

    private double computeDtSeconds(long timestampNanos) {
        if (lastTimestampNanos < 0) {
            lastTimestampNanos = timestampNanos;
            return 0.0;
        }
        double dt = (timestampNanos - lastTimestampNanos) / 1e9;
        lastTimestampNanos = timestampNanos;
        return dt;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private static double limitChange(double last, double desired, double maxDelta) {
        double delta = desired - last;
        if (Math.abs(delta) <= maxDelta) return desired;
        return last + Math.signum(delta) * maxDelta;
    }
}
