package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.Collections;
import java.util.List;

public class Vision {
    public enum Cell { FAR, AUDIENCE }

    public static class HiveTarget {
        public final Cell cell;
        public final double range;
        public final double bearing;
        public final double elevation;
        public final int tagsSeen;

        HiveTarget(Cell cell, AprilTagPoseFtc pose, int tagsSeen) {
            this.cell = cell;
            this.range = pose.range;
            this.bearing = pose.bearing;
            this.elevation = pose.elevation;
            this.tagsSeen = tagsSeen;
        }
    }

    private final AprilTagProcessor aprilTag;
    private final VisionPortal portal;

    public Vision(HardwareMap hardwareMap) {
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, RobotConstants.WEBCAM);
        if (webcam == null) {
            aprilTag = null;
            portal = null;
            return;
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, RobotConstants.CAMERA_X_IN,
                                RobotConstants.CAMERA_Y_IN, RobotConstants.CAMERA_Z_IN, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, RobotConstants.CAMERA_YAW_DEG,
                                RobotConstants.CAMERA_PITCH_DEG, RobotConstants.CAMERA_ROLL_DEG, 0))
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(RobotConstants.CAMERA_WIDTH, RobotConstants.CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .build();
    }

    private List<AprilTagDetection> detections() {
        return aprilTag == null ? Collections.emptyList() : aprilTag.getDetections();
    }

    public HiveTarget findHive(boolean blue) {
        HiveTarget best = null;
        for (AprilTagDetection d : detections()) {
            if (d.ftcPose == null) continue;

            if (d instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) d;
                Cell cell = cellForCluster(cluster.metadata == null ? null : cluster.metadata.name, blue);
                if (cell == null) continue;
                int tags = Math.max(1, Math.round(cluster.percentClusterFound / 25f));
                if (best == null || tags > best.tagsSeen) best = new HiveTarget(cell, d.ftcPose, tags);
            } else if (d instanceof AprilTagSingleDetection) {
                Cell cell = cellForTag(((AprilTagSingleDetection) d).id, blue);
                if (cell == null) continue;
                if (best == null || (best.tagsSeen == 1 && d.ftcPose.range < best.range)) {
                    best = new HiveTarget(cell, d.ftcPose, 1);
                }
            }
        }
        return best;
    }

    private static Cell cellForTag(int id, boolean blue) {
        int far = blue ? 42 : 30;
        int audience = blue ? 38 : 34;
        if (id >= far && id <= far + 3) return Cell.FAR;
        if (id >= audience && id <= audience + 3) return Cell.AUDIENCE;
        return null;
    }

    private static Cell cellForCluster(String name, boolean blue) {
        if (name == null || !name.startsWith(blue ? "BLUE" : "RED")) return null;
        return name.endsWith("AUDIENCE") ? Cell.AUDIENCE : Cell.FAR;
    }

    public void close() {
        if (portal != null) portal.close();
    }

    public void addTelemetry(Telemetry telemetry, boolean blue) {
        if (portal == null) {
            telemetry.addLine("No camera named \"" + RobotConstants.WEBCAM + "\" in the configuration.");
            return;
        }
        telemetry.addData("Camera", portal.getCameraState());

        HiveTarget hive = findHive(blue);
        if (hive == null) {
            telemetry.addData("Our HIVE", "not visible");
        } else {
            telemetry.addData("Our HIVE", "%s CELL is UP (%d tags)", hive.cell, hive.tagsSeen);
            telemetry.addData("  range / bearing / elev", "%.1f in / %.1f° / %.1f°",
                    hive.range, hive.bearing, hive.elevation);
        }

        for (AprilTagDetection d : detections()) {
            String label;
            if (d instanceof AprilTagSingleDetection) {
                label = "Tag " + ((AprilTagSingleDetection) d).id;
            } else {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) d;
                label = "Cluster " + (cluster.metadata == null ? "?" : cluster.metadata.name);
            }
            if (d.ftcPose == null) {
                telemetry.addData(label, "seen, no pose");
            } else {
                telemetry.addData(label, "range %.1f in, bearing %.1f°, elev %.1f°",
                        d.ftcPose.range, d.ftcPose.bearing, d.ftcPose.elevation);
            }
        }
    }
}
