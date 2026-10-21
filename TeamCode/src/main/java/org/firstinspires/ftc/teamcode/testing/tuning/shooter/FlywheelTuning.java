package org.firstinspires.ftc.teamcode.testing.tuning.shooter;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

@Disabled
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuning extends OpMode {
    private static final String[] GAIN_NAMES = {"P", "I", "D", "F"};
    private static final double[] STEPS = {10, 1, 0.1, 0.01};
    private static final double MAX_RPM = 6000;

    private DcMotorEx shooter;
    private final double[] gains = {
            RobotConstants.SHOOTER_P, RobotConstants.SHOOTER_I,
            RobotConstants.SHOOTER_D, RobotConstants.SHOOTER_F};
    private double targetRpm = RobotConstants.SHOOTER_TARGET_RPM;
    private int selectedGain = 3;
    private int stepIndex = 1;
    private boolean spinning = false;
    private double peakRpm = 0;
    private final Gamepad previous = new Gamepad();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        shooter = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER);
        shooter.setDirection(RobotConstants.SHOOTER_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyGains();
        telemetry.addLine("Flywheel tuner ready. Keep hands clear. Press START.");
    }

    @Override
    public void loop() {
        Gamepad g = gamepad1;
        boolean gainsChanged = false;

        if (g.triangle && !previous.triangle) {
            spinning = !spinning;
            peakRpm = 0;
        }
        if (g.dpad_up && !previous.dpad_up) targetRpm = Math.min(MAX_RPM, targetRpm + 100);
        if (g.dpad_down && !previous.dpad_down) targetRpm = Math.max(0, targetRpm - 100);
        if (g.square && !previous.square) selectedGain = (selectedGain + 1) % gains.length;
        if (g.cross && !previous.cross) stepIndex = (stepIndex + 1) % STEPS.length;
        if (g.dpad_right && !previous.dpad_right) {
            gains[selectedGain] += STEPS[stepIndex];
            gainsChanged = true;
        }
        if (g.dpad_left && !previous.dpad_left) {
            gains[selectedGain] = Math.max(0, gains[selectedGain] - STEPS[stepIndex]);
            gainsChanged = true;
        }
        previous.copy(g);

        if (gainsChanged) applyGains();

        if (spinning) shooter.setVelocity(targetRpm * RobotConstants.SHOOTER_TICKS_PER_REV / 60.0);
        else shooter.setPower(0);

        double rpm = shooter.getVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60.0;
        peakRpm = Math.max(peakRpm, rpm);

        telemetry.addData("Flywheel", spinning ? "SPINNING" : "off");
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Actual RPM", "%.0f", rpm);
        if (spinning && rpm < -50) {
            telemetry.addLine("SPINNING BACKWARDS: flip SHOOTER_REVERSED in RobotConstants");
        }
        telemetry.addData("Error RPM", "%.0f", targetRpm - rpm);
        telemetry.addData("Peak RPM (overshoot check)", "%.0f", peakRpm);
        telemetry.addData("Current (A)", "%.2f", shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
        for (int i = 0; i < gains.length; i++) {
            telemetry.addData((i == selectedGain ? "> " : "  ") + GAIN_NAMES[i], "%.3f", gains[i]);
        }
        telemetry.addData("Step", STEPS[stepIndex]);
        telemetry.update();
    }

    private void applyGains() {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(gains[0], gains[1], gains[2], gains[3]));
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}
