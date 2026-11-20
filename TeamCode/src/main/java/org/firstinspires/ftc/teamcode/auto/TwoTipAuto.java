package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

public abstract class TwoTipAuto extends LinearOpMode {
    static final double START_X = 61, START_Y = 9, START_DEG = 90;
    static final double[][] TO_HIVE = {{61, 9}, {61, 34}};
    static final double[][] TO_SIDE_FLOWER = {{61, 34}, {43, 37}, {16, 48}};
    static final double[][] BACK_TO_HIVE = {{16, 48}, {43, 37}, {61, 34}};
    static final double[][] TO_FAR_FLOWER = {{61, 34}, {10, 25}, {30, 110}, {48, 128}};
    static final double[][] TO_PARK = {{48, 128}, {14, 93}};

    private static final double PATH_POWER = 0.6;
    private static final double PATH_TIMEOUT_S = 8.0;
    private static final double STOP_AT_S = 29.5;
    private static final double LIFT_S = 0.5;
    private static final double SPIN_UP_MAX_S = 2.0;
    private static final double SHOOT_S = 3.0;
    private static final double FLOWER_S = 1.0;

    private final List<String> warnings = new ArrayList<>();
    private final ElapsedTime match = new ElapsedTime();
    private List<LynxModule> hubs;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Servos servos;
    private Elevator elevator;
    private boolean canDrive = true;
    private String step = "ready";
    private String stopped = null;

    protected abstract boolean blue();

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        drivetrain = new Drivetrain(hardwareMap, warnings);
        intake = new Intake(hardwareMap, drivetrain.anchor(), warnings);
        shooter = new Shooter(hardwareMap, drivetrain.anchor(), warnings);
        servos = new Servos(hardwareMap, warnings);
        elevator = new Elevator(hardwareMap, drivetrain.anchor(), warnings);

        while (opModeInInit()) {
            refresh();
            telemetry.addLine(blue() ? "BLUE 2 TIP" : "RED 2 TIP");
            telemetry.addData("Start here", "x %.0f  y %.0f  facing %.0f deg",
                    sx(START_X), sy(START_Y), START_DEG + (blue() ? 180 : 0));
            String problem = drivetrain.pinpointProblem();
            if (!drivetrain.hasPinpoint()) telemetry.addLine("! NO PINPOINT: will shoot once, no driving");
            else if (problem != null) telemetry.addLine("! " + problem);
            for (String warning : warnings) telemetry.addLine("! " + warning);
            telemetry.update();
        }
        if (isStopRequested()) return;

        match.reset();
        canDrive = drivetrain.hasPinpoint();
        if (!canDrive) stopped = "no pinpoint, shooting only";
        placeAtStart();

        follow(TO_HIVE, START_DEG, START_DEG, "to hive", true, false);
        shoot("tip 1");
        follow(TO_SIDE_FLOWER, 90, 180, "to side flower", false, false);
        collect("side flower");
        follow(BACK_TO_HIVE, 180, 90, "back to hive", true, true);
        shoot("tip 2");
        follow(TO_FAR_FLOWER, 90, 90, "to far flower", false, false);
        collect("far flower");
        follow(TO_PARK, 90, 90, "park", false, false);

        step = "done";
        drivetrain.stop();
        intake.stop();
        shooter.stop();
        elevator.stop();
        while (opModeIsActive()) {
            refresh();
            drivetrain.drive(0, 0, 0);
            show();
        }
    }

    private void follow(double[][] points, double startDeg, double endDeg, String name, boolean spinUp, boolean intaking) {
        if (!canDrive || !opModeIsActive() || match.seconds() > STOP_AT_S) return;
        step = name;
        Curve curve = new Curve(points, startDeg, endDeg, blue(), PATH_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !curve.done() && !curve.stuck()
                && timer.seconds() < PATH_TIMEOUT_S && match.seconds() < STOP_AT_S) {
            refresh();
            double[] power = curve.command(drivetrain.x(), drivetrain.y(), drivetrain.heading(), match.seconds());
            drivetrain.drive(power[0], power[1], power[2]);
            shooter.auto(spinUp);
            if (intaking) intake.load();
            else intake.stop();
            servos.set(false, true);
            telemetry.addData("Left on path", "%.0f in", curve.left());
            show();
        }
        drivetrain.drive(0, 0, 0);
        if (curve.stuck()) {
            canDrive = false;
            stopped = String.format("stopped on %s: %.0f in off path, check pinpoint pods", name, curve.offPath());
        } else if (!curve.done() && opModeIsActive() && match.seconds() < STOP_AT_S) {
            canDrive = false;
            stopped = "path timed out on " + name;
        }
    }

    private void shoot(String name) {
        if (!opModeIsActive() || match.seconds() > STOP_AT_S) return;
        step = name + ": elevator up";
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < LIFT_S) {
            refresh();
            drivetrain.drive(0, 0, 0);
            shooter.auto(true);
            elevator.run(true, false);
            servos.set(false, true);
            show();
        }
        elevator.run(false, false);

        step = name + ": spinning up";
        timer.reset();
        while (opModeIsActive() && timer.seconds() < SPIN_UP_MAX_S && !shooter.ready()) {
            refresh();
            shooter.auto(true);
            elevator.run(false, false);
            servos.set(false, true);
            show();
        }

        step = name + ": shooting";
        timer.reset();
        while (opModeIsActive() && timer.seconds() < SHOOT_S && match.seconds() < STOP_AT_S) {
            refresh();
            shooter.auto(true);
            intake.load();
            elevator.run(false, false);
            servos.set(false, true);
            show();
        }
        shooter.auto(false);
        intake.stop();

        step = name + ": elevator down";
        timer.reset();
        while (opModeIsActive() && timer.seconds() < LIFT_S) {
            refresh();
            elevator.run(false, true);
            servos.set(false, true);
            show();
        }
        elevator.stop();
    }

    private void collect(String name) {
        if (!canDrive) return;
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < FLOWER_S && match.seconds() < STOP_AT_S) {
            refresh();
            drivetrain.drive(0, 0, 0);
            intake.load();
            servos.set(false, true);
            show();
        }
        intake.stop();
    }

    private double sx(double x) { return blue() ? Curve.FIELD - x : x; }
    private double sy(double y) { return blue() ? Curve.FIELD - y : y; }

    private void placeAtStart() {
        double x = sx(START_X), y = sy(START_Y), h = START_DEG + (blue() ? 180 : 0);
        for (int i = 0; i < 3; i++) {
            drivetrain.setPose(x, y, h);
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

    private void show() {
        telemetry.addData("Auto", "%s  (%.1f s)", step, match.seconds());
        if (stopped != null) telemetry.addLine("! " + stopped);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f deg", drivetrain.x(), drivetrain.y(),
                Math.toDegrees(drivetrain.heading()));
        shooter.addTelemetry(telemetry);
        elevator.addTelemetry(telemetry);
        telemetry.update();
    }
}
