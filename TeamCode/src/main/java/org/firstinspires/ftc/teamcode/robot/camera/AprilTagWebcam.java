package org.firstinspires.ftc.teamcode.robot.camera;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    public AprilTagWebcam(WebcamName webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getClosestTagById(int... ids) {
        Iterable<AprilTagDetection> matches = findTagsByIds(ids);
        AprilTagDetection bestMatch = null;
        double closestDistance = Double.MAX_VALUE;
        for (AprilTagDetection match: matches) {
            double distance = calculateAprilTagDistance(match);
            if (distance < closestDistance) {
                bestMatch = match;
                closestDistance = distance;
            }
        }
        return bestMatch;
    }

    public Iterable<AprilTagDetection> findTagsByIds(int... ids) {
        return () -> detectedTags.stream().filter(t -> Arrays.stream(ids).anyMatch(id -> id == t.id)).iterator();
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private double calculateAprilTagDistance(AprilTagDetection detection) {
        double x = detection.ftcPose.x;
        double y = detection.ftcPose.y;
        return Math.sqrt(x * x + y * y);
    }
}
