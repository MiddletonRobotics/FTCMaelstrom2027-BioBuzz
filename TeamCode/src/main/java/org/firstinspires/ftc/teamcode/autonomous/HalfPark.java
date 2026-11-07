package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

public abstract class HalfPark extends LinearOpMode {
    private static final double MAX_POWER = 0.5;
    private static final double MIN_POWER = 0.12;
    private static final double DRIVE_KP = 0.04;
    private static final double TURN_KP = 1.0;
    private static final double MAX_TURN = 0.3;
    private static final double DONE_IN = 1.0;
    private static final double DONE_DEG = 3.0;
    private static final double STUCK_S = 1.5;
    private static final double STUCK_GAIN_IN = 2.0;
    private static final double STOP_AT_S = 29.0;
    private static final double NO_PINPOINT_POWER = 0.4;
    private static final double NO_PINPOINT_S = 0.8;
    private static final double CHECK_IN = 3.0;

    private final List<String> warnings = new ArrayList<>();
    private final ElapsedTime match = new ElapsedTime();
    private List<LynxModule> hubs;
    private Drivetrain drivetrain;
    private double holdHeading;
    private boolean ok = true;
    private String step = "ready";
    private String stopped = null;

    protected abstract String title();
    protected abstract String placement();
    protected abstract double[] startPose();
    protected abstract double[][] legs();
    protected abstract String[] legNames();

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        drivetrain = new Drivetrain(hardwareMap, warnings);

        sleep(300);
        refresh();
        double x0 = drivetrain.x(), y0 = drivetrain.y();
        while (opModeInInit()) {
            refresh();
            showInit(drivetrain.x() - x0, drivetrain.y() - y0);
        }
        if (isStopRequested()) return;
        match.reset();

        driveForTime(NO_PINPOINT_POWER, 0, 1.25, "forward");
        driveForTime(0, -NO_PINPOINT_POWER, 1.875, "strafe left");
        driveForTime(-NO_PINPOINT_POWER, 0, 2.5, "backward");

        step = ok ? "PARKED" : "stopped";
        drivetrain.stop();
        while (opModeIsActive()) {
            refresh();
            show();
        }
    }

    private void goTo(double tx, double ty, String name) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        double checkTime = 0;
        double checkDist = Math.hypot(tx - drivetrain.x(), ty - drivetrain.y());
        double timeout = checkDist / 10.0 + 3.0;
        while (opModeIsActive() && match.seconds() < STOP_AT_S) {
            refresh();
            double x = drivetrain.x(), y = drivetrain.y(), h = drivetrain.heading();
            double ex = tx - x, ey = ty - y;
            double dist = Math.hypot(ex, ey);
            double headingError = wrap(holdHeading - h);
            if (dist < DONE_IN && Math.abs(Math.toDegrees(headingError)) < DONE_DEG) break;

            if (timer.seconds() - checkTime >= STUCK_S) {
                if (dist > 4 && checkDist - dist < STUCK_GAIN_IN) {
                    fail(String.format("stuck on %s, %.0f in short. Check pods / wheels.", name, dist));
                    return;
                }
                checkTime = timer.seconds();
                checkDist = dist;
            }
            if (timer.seconds() > timeout) {
                fail(String.format("timed out on %s, %.0f in short", name, dist));
                return;
            }

            double speed = dist < DONE_IN ? 0 : clamp(DRIVE_KP * dist, MIN_POWER, MAX_POWER);
            double vx = dist > 1e-6 ? ex / dist * speed : 0;
            double vy = dist > 1e-6 ? ey / dist * speed : 0;
            double forward = vx * Math.cos(h) + vy * Math.sin(h);
            double strafe = vx * Math.sin(h) - vy * Math.cos(h);
            double turn = clamp(-TURN_KP * headingError, -MAX_TURN, MAX_TURN);
            drivetrain.drive(forward, strafe, turn);
            telemetry.addData("To go", "%.1f in", dist);
            show();
        }
        drivetrain.drive(0, 0, 0);
    }

    private void driveForTime(double forward, double strafe, double seconds, String name) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            refresh();
            drivetrain.drive(forward, strafe, 0);
            show();
        }
        drivetrain.drive(0, 0, 0);
    }

    private void fail(String why) {
        drivetrain.drive(0, 0, 0);
        ok = false;
        stopped = why;
    }

    private void placeAt(double x, double y, double headingDeg) {
        for (int i = 0; i < 3; i++) {
            drivetrain.setPose(x, y, headingDeg);
            for (int k = 0; k < 3; k++) {
                sleep(20);
                refresh();
            }
            if (Math.hypot(drivetrain.x() - x, drivetrain.y() - y) < 1.0) return;
        }
    }

    private void refresh() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        drivetrain.update();
    }

    private void showInit(double dx, double dy) {
        telemetry.addLine("===== " + title() + " =====");
        telemetry.addLine("PLACE ROBOT: " + placement());
        telemetry.addLine();
        if (!drivetrain.hasPinpoint()) telemetry.addLine("! NO PINPOINT: will only drive forward a little");
        else if (drivetrain.pinpointProblem() != null) telemetry.addLine("! " + drivetrain.pinpointProblem());
        telemetry.addLine("Pod check (push robot by hand, then put it back):");
        telemetry.addData("  push FORWARD", "%+.1f in  %s", dx, verdict(dx, "POD_X_REVERSED"));
        telemetry.addData("  push LEFT", "%+.1f in  %s", dy, verdict(dy, "POD_Y_REVERSED"));
        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.update();
    }

    private static String verdict(double value, String fix) {
        if (value > CHECK_IN) return "OK";
        if (value < -CHECK_IN) return "WRONG WAY, flip " + fix;
        return "";
    }

    private void show() {
        telemetry.addData("Auto", "%s  %.1f s", step, match.seconds());
        if (stopped != null) telemetry.addLine("! " + stopped);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f deg", drivetrain.x(), drivetrain.y(),
                Math.toDegrees(drivetrain.heading()));
        telemetry.update();
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
