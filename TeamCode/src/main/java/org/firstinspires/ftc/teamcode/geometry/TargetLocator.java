package org.firstinspires.ftc.teamcode.geometry;

public class TargetLocator {
    private final double aprilTagToTargetDistance;

    public TargetLocator(double aprilTagToTargetDistance) {
        this.aprilTagToTargetDistance = aprilTagToTargetDistance;
    }

    public double getAngle(double aprilTagX, double aprilTagY, double aprilTagYawAngle) {
        // For a visualization of this calculation, see https://www.geogebra.org/geometry/zqpxtjyf

        double a = aprilTagYawAngle + 90;
        double x2 = Math.cos(Math.toRadians(a)) * aprilTagToTargetDistance;
        double y2 = Math.sin(Math.toRadians(a)) * aprilTagToTargetDistance;
        double xTotal = aprilTagX + x2;
        double yTotal = aprilTagY + y2;

        return Math.toDegrees(Math.atan(xTotal / yTotal));
    }
}
