package org.firstinspires.ftc.teamcode.testing.tuning.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Robot on a box, wheels off the ground.
 * cross    = spin each wheel alone at full power, then all four together, and measure them
 * triangle = after measuring, test speed control at 50% and 90% (all four should match)
 */
@TeleOp(name = "Drive Tuner", group = "Tuning")
public class DriveTuner extends LinearOpMode {
    private static final String[] NAMES = {"FRONT_LEFT", "FRONT_RIGHT", "BACK_LEFT", "BACK_RIGHT"};
    private static final boolean[] REVERSED = {
            RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
            RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};
    private static final double SPIN_TIME_S = 2.5;
    private static final double SAMPLE_TIME_S = 1.0;
    private static final double SPIN_DOWN_TIME_S = 1.0;
    private static final double MIN_GOOD_TPS = 100;

    private DcMotorEx[] wheels;
    private final List<String> notes = new ArrayList<>();
    private final List<String> results = new ArrayList<>();
    private final double[] maxTps = new double[4];
    private boolean measured = false;

    @Override
    public void runOpMode() {
        wheels = MotorFinder.drive(hardwareMap, notes);
        for (int i = 0; i < 4; i++) {
            wheels[i].setDirection(REVERSED[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (String note : notes) telemetry.addLine("! " + note);
        telemetry.addLine("Put the robot on a box so the wheels are OFF the ground. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.cross) measure();
            else if (gamepad1.triangle && measured) checkSpeedControl();

            for (String note : notes) telemetry.addLine("! " + note);
            telemetry.addLine("cross = measure all wheels" + (measured ? ", triangle = test speed control" : ""));
            telemetry.addLine();
            for (String line : results) telemetry.addLine(line);
            telemetry.update();
        }
        stopAll();
    }

    private void measure() {
        results.clear();
        double[] alone = new double[4];
        double[] aloneAmps = new double[4];
        for (int i = 0; i < 4; i++) {
            double[][] sample = spin(i, "Spinning " + NAMES[i] + " alone. It must turn the way that"
                    + " drives the robot FORWARD. If it goes backward, flip " + NAMES[i] + "_REVERSED.");
            if (sample == null) return;
            alone[i] = sample[0][i];
            aloneAmps[i] = sample[1][i];
        }
        double[][] together = spin(-1, "Spinning ALL FOUR. All must turn the way that drives FORWARD.");
        if (together == null) return;

        double medianTps = median(together[0]);
        double medianAmps = median(aloneAmps);
        for (int i = 0; i < 4; i++) {
            double tps = together[0][i];
            maxTps[i] = tps;
            String flag;
            if (Math.abs(tps) < MIN_GOOD_TPS) flag = "ENCODER NOT READING, check its encoder cable";
            else if (tps < 0) flag = "ENCODER BACKWARDS, check the motor power wires";
            else if (tps > 1.08 * medianTps) flag = "FASTER than the others, different motor/gearbox?";
            else if (tps < 0.92 * medianTps) flag = "SLOWER than the others, check for rubbing";
            else if (aloneAmps[i] > 1.5 * medianAmps) flag = "pulling extra current, check for binding";
            else flag = "ok";
            results.add(String.format("%s: %.0f tps (%.0f rpm), alone %.0f tps %.1f A  %s", NAMES[i], tps,
                    tps * 60 / RobotConstants.DRIVE_TICKS_PER_REV, alone[i], aloneAmps[i], flag));
        }
        results.add("");
        results.add("Copy into RobotConstants:");
        for (int i = 0; i < 4; i++) {
            results.add(maxTps[i] < MIN_GOOD_TPS ? NAMES[i] + "_MAX_TPS: fix the encoder first"
                    : String.format("%s_MAX_TPS = %.0f;", NAMES[i], maxTps[i]));
        }
        results.add("Currently: " + RobotConstants.FRONT_LEFT_MAX_TPS + " / " + RobotConstants.FRONT_RIGHT_MAX_TPS
                + " / " + RobotConstants.BACK_LEFT_MAX_TPS + " / " + RobotConstants.BACK_RIGHT_MAX_TPS);
        measured = true;
    }

    // Full power on one wheel (or all with -1); returns {average tps, average amps} over the last second
    private double[][] spin(int only, String message) {
        double[] tps = new double[4];
        double[] amps = new double[4];
        int samples = 0;
        ElapsedTime timer = new ElapsedTime();
        for (int i = 0; i < 4; i++) wheels[i].setPower(only < 0 || only == i ? 1 : 0);
        while (opModeIsActive() && timer.seconds() < SPIN_TIME_S) {
            if (timer.seconds() > SPIN_TIME_S - SAMPLE_TIME_S) {
                for (int i = 0; i < 4; i++) {
                    tps[i] += wheels[i].getVelocity();
                    amps[i] += wheels[i].getCurrent(CurrentUnit.AMPS);
                }
                samples++;
            }
            telemetry.addLine(message);
            telemetry.update();
        }
        stopAll();
        if (!waitFor(SPIN_DOWN_TIME_S) || samples == 0) return null;
        for (int i = 0; i < 4; i++) {
            tps[i] /= samples;
            amps[i] /= samples;
        }
        return new double[][]{tps, amps};
    }

    private void checkSpeedControl() {
        results.clear();
        double slowest = Double.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            if (maxTps[i] < MIN_GOOD_TPS) continue;
            double f = 32767 / maxTps[i];
            double p = 0.1 * f;
            wheels[i].setVelocityPIDFCoefficients(p, 0.1 * p, 0, f);
            wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slowest = Math.min(slowest, maxTps[i]);
        }
        if (slowest == Double.MAX_VALUE) {
            results.add("No working encoders, can't test speed control");
            return;
        }

        for (double fraction : new double[]{0.5, 0.9}) {
            double target = fraction * slowest;
            double[] tps = new double[4];
            int samples = 0;
            ElapsedTime timer = new ElapsedTime();
            for (int i = 0; i < 4; i++) {
                if (maxTps[i] >= MIN_GOOD_TPS) wheels[i].setVelocity(target);
            }
            while (opModeIsActive() && timer.seconds() < SPIN_TIME_S) {
                if (timer.seconds() > SPIN_TIME_S - SAMPLE_TIME_S) {
                    for (int i = 0; i < 4; i++) tps[i] += wheels[i].getVelocity();
                    samples++;
                }
                telemetry.addLine(String.format("Speed control at %.0f%% (%.0f tps)...", fraction * 100, target));
                telemetry.update();
            }
            stopAll();
            if (!waitFor(SPIN_DOWN_TIME_S) || samples == 0) break;

            results.add(String.format("At %.0f%% (target %.0f tps):", fraction * 100, target));
            for (int i = 0; i < 4; i++) {
                if (maxTps[i] < MIN_GOOD_TPS) {
                    results.add("  " + NAMES[i] + ": skipped, encoder not working");
                    continue;
                }
                double actual = tps[i] / samples;
                double error = 100 * (actual - target) / target;
                results.add(String.format("  %s: %.0f tps (%+.1f%%)  %s", NAMES[i], actual, error,
                        Math.abs(error) <= 3 ? "ok" : "OFF, run cross again"));
            }
        }
        for (DcMotorEx wheel : wheels) wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) idle();
        return opModeIsActive();
    }

    private void stopAll() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
    }

    private static double median(double[] values) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        return (sorted[1] + sorted[2]) / 2;
    }
}
