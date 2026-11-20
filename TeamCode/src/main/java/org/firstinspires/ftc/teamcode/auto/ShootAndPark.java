package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.FileWriter;
import java.io.IOException;

@Autonomous(name = "Shoot + Park", group = "Auto")
public class ShootAndPark extends ShootAuto {
    private static final double[][] PATH = {
            {57, 8},
            {57, 20},
            {30, 20},
            {30, 75},
            {10, 75}};
    private static final int LONG_LEG = 3;
    private static final double FORWARD_PERCENT = 72.3;

    private static final double PATH_POWER = 0.6;
    private static final double SLOW_IN = 10;
    private static final double SLOW_POWER = 0.25;
    private static final double SLOW_STRAFE_POWER = 0.35;
    private static final double HEADING_KP = 1.0;
    private static final double HEADING_MAX = 0.15;
    private static final double HEADING_OFF_DEG = 15;
    private static final double STUCK_S = 0.75;

    private static final double TIMED_POWER = 0.4;
    private static final double FORWARD_IN_PER_S = 16;
    private static final double STRAFE_IN_PER_S = 13;

    private static final String LOG = "/sdcard/FIRST/hive_park_log.csv";

    private final ElapsedTime autoTime = new ElapsedTime();
    private String leg = "";
    private FileWriter log;

    @Override
    public void runOpMode() {
        initHardware();
        String problem = drivetrain.pinpointProblem();
        telemetry.addLine("shoot, then path (57,8) > (57,20) > (30,20) > (30,75) > (10,75), facing 270");
        telemetry.addLine(problem == null ? "pinpoint OK, driving by inches" : "! " + problem + ", driving by time");
        telemetry.update();
        waitForStart();

        autoTime.reset();
        try {
            log = new FileWriter(LOG);
            log.write("t,step,x,y,headingDeg,traveled,target,odo\n");
        } catch (IOException e) {
            log = null;
        }

        shootPreload();

        for (int i = 1; i < PATH.length; i++) {
            double dx = PATH[i][0] - PATH[i - 1][0];
            double dy = PATH[i][1] - PATH[i - 1][1];
            double full = Math.hypot(dx, dy);
            double inches = i == LONG_LEG ? full * FORWARD_PERCENT / 100 : full;

            move(-dy / full * PATH_POWER, -dx / full * PATH_POWER, inches, full,
                    "path " + i + " to (" + (int) PATH[i][0] + ", " + (int) PATH[i][1] + ")");
        }

        stopAll();
        step = "done";
        write(String.format("%.2f,done,%.1f,%.1f,%.1f,0,0,true", autoTime.seconds(),
                drivetrain.x(), drivetrain.y(), Math.toDegrees(drivetrain.heading())));
        try {
            if (log != null) log.close();
        } catch (IOException ignored) {
        }

        while (opModeIsActive()) {
            show();
            idle();
        }
    }

    private void move(double forward, double strafe, double inches, double full, String name) {
        step = name;
        drivetrain.update();
        boolean odo = drivetrain.pinpointProblem() == null;
        double x0 = drivetrain.x(), y0 = drivetrain.y(), h0 = drivetrain.heading();
        double inPerS = (strafe != 0 ? STRAFE_IN_PER_S : FORWARD_IN_PER_S) * Math.hypot(forward, strafe) / TIMED_POWER;
        double timed = inches / inPerS;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime check = new ElapsedTime();
        double checked = 0;
        double traveled = 0;
        while (opModeIsActive()) {
            drivetrain.update();
            double t = timer.seconds();
            if (odo) {
                traveled = Math.hypot(drivetrain.x() - x0, drivetrain.y() - y0);
                double left = inches - traveled;
                if (left <= 0 || t > timed * 2 + 1) break;

                if (check.seconds() > STUCK_S) {
                    if (traveled < 1) odo = false;
                    else if (traveled - checked < 1) break;
                    checked = traveled;
                    check.reset();
                }

                double slow = strafe != 0 ? SLOW_STRAFE_POWER : SLOW_POWER;
                double scale = left < SLOW_IN ? slow / PATH_POWER : 1;
                double err = AngleUnit.normalizeRadians(drivetrain.heading() - h0);
                double turn = Math.abs(err) < Math.toRadians(HEADING_OFF_DEG)
                        ? Range.clip(HEADING_KP * err, -HEADING_MAX, HEADING_MAX) : 0;
                drivetrain.drive(forward * scale, strafe * scale, turn);
            } else {
                if (t >= timed) break;
                traveled = inches * t / timed;
                drivetrain.drive(forward, strafe, 0);
            }

            leg = String.format("%.1f / %.0f in  (%.0f of 100)  %s", traveled, inches, traveled / full * 100, odo ? "pinpoint" : "timed");
            write(String.format("%.2f,%s,%.1f,%.1f,%.1f,%.1f,%.0f,%b", autoTime.seconds(), name,
                    drivetrain.x(), drivetrain.y(), Math.toDegrees(drivetrain.heading()), traveled, inches, odo));
            show();
        }
        drivetrain.stop();
    }

    private void write(String line) {
        if (log == null) return;
        try {
            log.write(line + "\n");
        } catch (IOException e) {
            log = null;
        }
    }

    @Override
    protected void show() {
        telemetry.addData("Auto", "%s  (%.1f s)", step, autoTime.seconds());
        if (!leg.isEmpty()) telemetry.addData("Leg", leg);
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        servos.addTelemetry(telemetry);
        elevator.addTelemetry(telemetry);
        showBattery();
        telemetry.update();
    }
}
