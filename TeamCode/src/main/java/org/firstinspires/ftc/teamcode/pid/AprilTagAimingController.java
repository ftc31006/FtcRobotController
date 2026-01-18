package org.firstinspires.ftc.teamcode.pid;

public class AprilTagAimingController {
        private double Kp;

        /**
         * Constructor for the PID Controller.
         * @param Kp Proportional gain.
         */
        public AprilTagAimingController(double Kp) {
            this.Kp = Kp;
        }

        /**
         * Calculates the control output based on the current measurement.
         * This method should be called at a consistent time interval.
         * @param measurement The current value from the sensor.
         * @return The calculated output (e.g., motor power).
         */
        public double calculate(double targetValue, double measurement) {
            // 1. Calculate the error
            double error = targetValue - measurement;

            // 2. Calculate the Proportional term
            double proportionalTerm = Kp * error;

            // 5. Calculate the total output
            double output = proportionalTerm;

            // 6. Apply output limits (clamping)
            if (output > 1) output = 1;
            if (output < -1) output = -1;

            return output;
        }

        public void setKp(double Kp) {
            this.Kp = Kp;
        }
}
