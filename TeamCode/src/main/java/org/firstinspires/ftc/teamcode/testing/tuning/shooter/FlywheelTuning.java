package org.firstinspires.ftc.teamcode.testing.tuning.shooter;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;
import org.firstinspires.ftc.teamcode.subsystems.Servos;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuning extends OpMode {
    private static final double[] FF_POWERS = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    private static final double[] KP_TRIES = {0.0001, 0.0002, 0.0004, 0.0007, 0.0012, 0.002, 0.0035, 0.006};
    private static final double FF_HOLD_S = 2.0;
    private static final double FF_SAMPLE_S = 0.8;
    private static final double DROP_S = 1.5;
    private static final double STEP_S = 2.5;
    private static final double STEADY_S = 1.0;
    private static final double DROP_RPM = 1000;
    private static final double SHOT_DROP_RPM = 150;
    private static final int MAX_LOG = 40000;
    private static final String[] NAMES = {"kS", "kV", "kP", "kI", "kD"};
    private static final double[] SEEDS = {0.02, 0.0001, 0.0001, 0.0001, 0.000005};

    private enum Phase { MANUAL, FF, DROP, STEP }

    private final List<LynxModule> hubs = new ArrayList<>();
    private final List<String> warnings = new ArrayList<>();
    private final List<String> results = new ArrayList<>();
    private final Gamepad previous = new Gamepad();
    private final ElapsedTime stepTime = new ElapsedTime();
    private final ElapsedTime runTime = new ElapsedTime();
    private final StringBuilder log = new StringBuilder("t,phase,target,rpm,power,volts\n");
    private int logRows = 0;

    private Flywheel flywheel;
    private DcMotorEx transfer;
    private Servos servos;

    private final double[] gains = {
            RobotConstants.SHOOTER_KS, RobotConstants.SHOOTER_KV, RobotConstants.SHOOTER_KP,
            RobotConstants.SHOOTER_KI, RobotConstants.SHOOTER_KD};
    private int selected = 2;
    private boolean boostOn = true;
    private boolean spinning = false;
    private double targetRpm = RobotConstants.SHOOTER_TARGET_RPM;

    private Phase phase = Phase.MANUAL;
    private int step = 0;
    private String autoMessage = "";
    private final double[] ffRpm = new double[FF_POWERS.length];
    private final double[] ffEffort = new double[FF_POWERS.length];
    private double sumRpm, sumSq, sumVolts;
    private int samples;
    private double noise = 0;
    private double maxRpm = 0;
    private double testRpm;
    private double peak, riseS;
    private final double[] kpOver = new double[KP_TRIES.length];
    private final double[] kpOsc = new double[KP_TRIES.length];
    private final double[] kpErr = new double[KP_TRIES.length];
    private final double[] kpRise = new double[KP_TRIES.length];

    private boolean inShot = false;
    private double shotStart, shotMin, lastDip = 0, lastRecovery = 0, totalRecovery = 0;
    private int shots = 0;

    @Override
    public void init() {
        hubs.addAll(hardwareMap.getAll(LynxModule.class));
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER);
        shooter.setDirection(MotorFinder.direction(RobotConstants.SHOOTER_REVERSED));
        flywheel = new Flywheel(shooter, hardwareMap.voltageSensor.iterator().next());
        applyGains();

        transfer = hardwareMap.tryGet(DcMotorEx.class, RobotConstants.FEEDER);
        if (transfer != null) {
            transfer.setDirection(MotorFinder.direction(RobotConstants.FEEDER_REVERSED));
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            warnings.add("no transfer motor, RT does nothing");
        }
        servos = new Servos(hardwareMap, warnings);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("FLYWHEEL TUNING. Shooter mounted, NO balls in it, hands clear.");
        telemetry.addLine("Battery should be charged (12.5 V+). Press START.");
        for (String w : warnings) telemetry.addLine("! " + w);
        telemetry.update();
    }

    @Override
    public void start() {
        runTime.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        Gamepad g = gamepad1;

        if (phase == Phase.MANUAL) manualButtons(g);
        else if (g.share && !previous.share) abort("stopped by you");
        previous.copy(g);

        if (phase == Phase.MANUAL) {
            flywheel.setRpm(spinning ? targetRpm : 0);
        } else {
            runAuto();
        }
        flywheel.update();

        if (phase != Phase.MANUAL && !flywheel.hasEncoder()) abort("encoder reads 0. Plug the shooter encoder cable in (same port number as the motor).");
        if (phase != Phase.MANUAL && flywheel.backwards()) abort("spinning backwards. Flip SHOOTER_REVERSED, push, run again.");

        if (transfer != null) transfer.setPower(phase == Phase.MANUAL && g.right_trigger > 0.1 ? RobotConstants.FEEDER_SPEED : 0);
        servos.set(g.circle, false, g.dpad_right ? RobotConstants.SPIN_POWER : 0);
        watchShots();
        record();
        show();
    }

    private void manualButtons(Gamepad g) {
        if (g.triangle && !previous.triangle) startAuto();
        if (g.cross && !previous.cross) spinning = !spinning;
        if (g.right_bumper && !previous.right_bumper) targetRpm = Math.min(RobotConstants.LAUNCHER_FREE_RPM, targetRpm + 100);
        if (g.left_bumper && !previous.left_bumper) targetRpm = Math.max(0, targetRpm - 100);
        if (g.square && !previous.square) selected = (selected + 1) % gains.length;
        if (g.dpad_up && !previous.dpad_up) nudge(1);
        if (g.dpad_down && !previous.dpad_down) nudge(-1);
        if (g.dpad_left && !previous.dpad_left) {
            boostOn = !boostOn;
            flywheel.setBoost(boostOn ? RobotConstants.SHOOTER_BOOST_RPM : 0);
        }
        if (g.options && !previous.options) save("saved by you");
    }

    private void nudge(int dir) {
        if (selected == 0) {
            gains[0] = Math.max(0, gains[0] + dir * 0.005);
        } else if (gains[selected] == 0) {
            if (dir > 0) gains[selected] = SEEDS[selected];
        } else {
            gains[selected] *= dir > 0 ? 1.15 : 1 / 1.15;
            if (gains[selected] < SEEDS[selected] / 4) gains[selected] = 0;
        }
        applyGains();
    }

    private void startAuto() {
        results.clear();
        autoMessage = "step 1/2: measuring kS + kV";
        spinning = false;
        phase = Phase.FF;
        step = 0;
        noise = 0;
        maxRpm = 0;
        resetSamples();
        stepTime.reset();
    }

    private void runAuto() {
        double t = stepTime.seconds();
        switch (phase) {
            case FF:
                flywheel.setOpenPower(FF_POWERS[step]);
                if (t > FF_HOLD_S - FF_SAMPLE_S) sample();
                if (t > FF_HOLD_S) {
                    double mean = sumRpm / samples;
                    ffRpm[step] = mean;
                    ffEffort[step] = FF_POWERS[step] * (sumVolts / samples) / RobotConstants.NOMINAL_VOLTAGE;
                    noise = Math.max(noise, std());
                    maxRpm = Math.max(maxRpm, mean);
                    next();
                    if (step == FF_POWERS.length) finishFeedforward();
                }
                break;
            case DROP:
                flywheel.setRpm(testRpm - DROP_RPM);
                if (t > DROP_S) {
                    phase = Phase.STEP;
                    peak = 0;
                    riseS = Double.NaN;
                    resetSamples();
                    stepTime.reset();
                }
                break;
            case STEP:
                flywheel.setRpm(testRpm);
                double rpm = flywheel.rpm();
                peak = Math.max(peak, rpm);
                if (Double.isNaN(riseS) && rpm >= testRpm - RobotConstants.SHOOTER_READY_TOLERANCE_RPM) riseS = t;
                if (t > STEP_S - STEADY_S) sample();
                if (t > STEP_S) {
                    kpOver[step] = peak - testRpm;
                    kpOsc[step] = std();
                    kpErr[step] = testRpm - sumRpm / samples;
                    kpRise[step] = riseS;
                    results.add(String.format("kP %.4f: over %.0f  wobble %.0f  err %.0f  rise %.2fs",
                            KP_TRIES[step], kpOver[step], kpOsc[step], kpErr[step], riseS));
                    if (unstable(step) || step == KP_TRIES.length - 1) {
                        finishKp();
                    } else {
                        step++;
                        phase = Phase.DROP;
                        stepTime.reset();
                    }
                }
                break;
        }
    }

    private void finishFeedforward() {
        int n = FF_POWERS.length;
        double mx = 0, my = 0;
        for (int i = 0; i < n; i++) {
            mx += ffRpm[i];
            my += ffEffort[i];
        }
        mx /= n;
        my /= n;
        double sxy = 0, sxx = 0, syy = 0;
        for (int i = 0; i < n; i++) {
            sxy += (ffRpm[i] - mx) * (ffEffort[i] - my);
            sxx += (ffRpm[i] - mx) * (ffRpm[i] - mx);
            syy += (ffEffort[i] - my) * (ffEffort[i] - my);
        }
        if (sxx < 1) {
            abort("flywheel barely moved. Check motor, belt, encoder.");
            return;
        }
        double kV = sxy / sxx;
        double kS = Math.max(0, my - kV * mx);
        double r2 = sxy * sxy / (sxx * syy);
        gains[0] = kS;
        gains[1] = kV;
        gains[2] = 0;
        gains[3] = 0;
        gains[4] = 0;
        applyGains();
        flywheel.setBoost(0);

        results.add(String.format("kS %.4f  kV %.7f  fit %.3f  top %.0f rpm  noise %.0f rpm", kS, kV, r2, maxRpm, noise));
        if (r2 < 0.98) results.add("! fit is messy: rubbing, loose belt or bad encoder. Fix then re-run.");

        testRpm = Math.min(RobotConstants.SHOOTER_TARGET_RPM, 0.85 * maxRpm);
        autoMessage = String.format("step 2/2: finding kP at %.0f rpm", testRpm);
        step = 0;
        gains[2] = KP_TRIES[0];
        applyGains();
        phase = Phase.DROP;
        stepTime.reset();
    }

    private boolean unstable(int i) {
        return kpOver[i] > Math.max(0.03 * testRpm, 3 * noise) || kpOsc[i] > 2 * noise + 15;
    }

    private void finishKp() {
        int good = -1;
        for (int i = 0; i <= step; i++) if (!unstable(i)) good = i;
        double kP = good < 0 ? KP_TRIES[0] / 2 : KP_TRIES[good] * 0.75;
        double kI = 0;
        if (good >= 0 && Math.abs(kpErr[good]) > 25) kI = kP;
        gains[2] = kP;
        gains[3] = kI;
        gains[4] = 0;
        applyGains();
        boostOn = true;
        flywheel.setBoost(RobotConstants.SHOOTER_BOOST_RPM);

        results.add(String.format("PICKED kP %.5f (75%% of last stable)%s", kP, kI > 0 ? ", small kI for leftover error" : ""));
        phase = Phase.MANUAL;
        spinning = false;
        targetRpm = testRpm;
        save("auto tune done");
        autoMessage = "DONE. Now press cross and feed balls with RT to check recovery.";
    }

    private void abort(String why) {
        phase = Phase.MANUAL;
        spinning = false;
        flywheel.stop();
        autoMessage = "STOPPED: " + why;
        saveLog();
    }

    private void watchShots() {
        if (phase != Phase.MANUAL || !spinning) {
            inShot = false;
            return;
        }
        double now = runTime.seconds();
        double rpm = flywheel.rpm();
        if (!inShot && flywheel.atSpeed() && rpm < targetRpm - SHOT_DROP_RPM) {
            inShot = true;
            shotStart = now;
            shotMin = rpm;
        }
        if (inShot) {
            shotMin = Math.min(shotMin, rpm);
            if (Math.abs(targetRpm - rpm) < RobotConstants.SHOOTER_READY_TOLERANCE_RPM) {
                inShot = false;
                lastDip = targetRpm - shotMin;
                lastRecovery = now - shotStart;
                totalRecovery += lastRecovery;
                shots++;
            }
        }
    }

    private void show() {
        telemetry.addLine("triangle AUTO TUNE   share stop   cross spin on/off   RB/LB +-100 rpm");
        telemetry.addLine("square pick gain   dpad up/down change it   dpad left boost   options save");
        telemetry.addLine("RT transfer   circle gate   dpad right spinner");
        telemetry.addLine();
        if (!autoMessage.isEmpty()) telemetry.addLine(autoMessage);
        if (phase == Phase.FF) telemetry.addData("Power", "%.1f  (%d/%d)", FF_POWERS[step], step + 1, FF_POWERS.length);
        if (phase == Phase.DROP || phase == Phase.STEP) telemetry.addData("Trying kP", "%.4f  (%d/%d)", KP_TRIES[step], step + 1, KP_TRIES.length);
        telemetry.addData("RPM", "%.0f / %.0f   error %.0f", flywheel.rpm(), flywheel.target(), flywheel.target() - flywheel.rpm());
        telemetry.addData("Power / battery", "%.2f / %.2f V", flywheel.power(), flywheel.volts());
        telemetry.addData("Ready", flywheel.atSpeed() ? "YES" : "no");
        telemetry.addData("Boost", boostOn ? "on (" + RobotConstants.SHOOTER_BOOST_RPM + " rpm)" : "off");
        if (!flywheel.hasEncoder()) telemetry.addLine("! ENCODER READS 0, plug it in");
        if (flywheel.backwards()) telemetry.addLine("! BACKWARDS, flip SHOOTER_REVERSED");
        if (shots > 0) {
            telemetry.addData("Shots", "%d   last dip %.0f rpm   last recovery %.2fs   avg %.2fs",
                    shots, lastDip, lastRecovery, totalRecovery / shots);
        }
        telemetry.addLine();
        for (int i = 0; i < gains.length; i++) {
            telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.7f", gains[i]);
        }
        telemetry.addLine();
        for (String r : results) telemetry.addLine(r);
        for (String w : warnings) telemetry.addLine("! " + w);
        telemetry.update();
    }

    private void record() {
        if (logRows >= MAX_LOG) return;
        if (phase == Phase.MANUAL && !spinning) return;
        log.append(String.format("%.3f,%s,%.0f,%.0f,%.3f,%.2f\n", runTime.seconds(), phase,
                flywheel.target(), flywheel.rpm(), flywheel.power(), flywheel.volts()));
        logRows++;
    }

    private void save(String why) {
        StringBuilder s = new StringBuilder(why + "\n\n");
        s.append(String.format("    public static final double SHOOTER_KS = %.4f;\n", gains[0]));
        s.append(String.format("    public static final double SHOOTER_KV = %.7f;\n", gains[1]));
        s.append(String.format("    public static final double SHOOTER_KP = %.5f;\n", gains[2]));
        s.append(String.format("    public static final double SHOOTER_KI = %.5f;\n", gains[3]));
        s.append(String.format("    public static final double SHOOTER_KD = %.6f;\n\n", gains[4]));
        for (String r : results) s.append(r).append('\n');
        if (shots > 0) s.append(String.format("shots %d, avg recovery %.2fs\n", shots, totalRecovery / shots));
        write("flywheel_tune.txt", s.toString());
        saveLog();
    }

    private void saveLog() {
        write("flywheel_log.csv", log.toString());
    }

    private void write(String name, String text) {
        try (FileWriter w = new FileWriter(new File(AppUtil.FIRST_FOLDER, name))) {
            w.write(text);
        } catch (Exception e) {
            warnings.add("couldn't save " + name);
        }
    }

    private void applyGains() {
        flywheel.setGains(gains[0], gains[1], gains[2], gains[3], gains[4]);
    }

    private void sample() {
        double r = flywheel.rpm();
        sumRpm += r;
        sumSq += r * r;
        sumVolts += flywheel.volts();
        samples++;
    }

    private double std() {
        if (samples < 2) return 0;
        double mean = sumRpm / samples;
        return Math.sqrt(Math.max(0, sumSq / samples - mean * mean));
    }

    private void resetSamples() {
        sumRpm = 0;
        sumSq = 0;
        sumVolts = 0;
        samples = 0;
    }

    private void next() {
        step++;
        resetSamples();
        stepTime.reset();
    }

    @Override
    public void stop() {
        flywheel.stop();
        if (transfer != null) transfer.setPower(0);
        servos.stop();
        saveLog();
    }
}
