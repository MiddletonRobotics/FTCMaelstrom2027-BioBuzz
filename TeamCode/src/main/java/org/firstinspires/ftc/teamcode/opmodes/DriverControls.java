package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriverControls {
    private final Drivetrain drivetrain;
    private final Gamepad previous = new Gamepad();
    private boolean fieldCentric = false;
    private boolean reverseDrive = false;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastForward, lastStrafe, lastTurn;

    private final ElapsedTime sinceTurning = new ElapsedTime();
    private boolean holdingHeading = false;
    private double targetHeading = 0;

    public DriverControls(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void update(Gamepad gamepad, Telemetry telemetry) {
        drivetrain.update();

        if (gamepad.share && !previous.share && drivetrain.hasImu()) {
            fieldCentric = !fieldCentric;
            gamepad.rumble(200);
        }
        if (gamepad.triangle && !previous.triangle) {
            reverseDrive = !reverseDrive;
            gamepad.rumble(200);
        }
        if (gamepad.options && !previous.options) {
            drivetrain.resetHeading();
            holdingHeading = false;
            gamepad.rumble(100);
        }
        previous.copy(gamepad);

        double forward = shape(-gamepad.left_stick_y);
        double strafe = shape(gamepad.left_stick_x);
        double turn = shape(gamepad.right_stick_x) * RobotConstants.TURN_SCALE;

        if (reverseDrive && !fieldCentric) {
            forward = -forward;
            strafe = -strafe;
        }

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (RobotConstants.DRIVE_ACCEL_LIMIT > 0) {
            double maxStep = RobotConstants.DRIVE_ACCEL_LIMIT * dt;
            forward = approach(lastForward, forward, maxStep);
            strafe = approach(lastStrafe, strafe, maxStep);
            turn = approach(lastTurn, turn, maxStep);
        }
        lastForward = forward;
        lastStrafe = strafe;
        lastTurn = turn;

        turn = applyHeadingHold(forward, strafe, turn);

        if (fieldCentric) drivetrain.driveFieldCentric(forward, strafe, turn);
        else drivetrain.drive(forward, strafe, turn);

        telemetry.addData("Drive mode", (fieldCentric ? "FIELD-centric" : "Robot-centric")
                + (reverseDrive && !fieldCentric ? "  [REVERSED]" : "")
                + (holdingHeading ? "  [heading hold]" : ""));
        if (!drivetrain.hasImu()) {
            telemetry.addLine("WARNING: no IMU named \"" + RobotConstants.IMU
                    + "\" in config. Field-centric and heading hold disabled.");
        }
        drivetrain.addTelemetry(telemetry);
    }

    private double applyHeadingHold(double forward, double strafe, double turn) {
        if (!RobotConstants.HEADING_HOLD_ENABLED || !drivetrain.hasImu()) {
            holdingHeading = false;
            return turn;
        }
        if (turn != 0) {
            sinceTurning.reset();
            holdingHeading = false;
            return turn;
        }
        if (sinceTurning.seconds() < RobotConstants.HEADING_HOLD_DELAY_S) return turn;
        if (!holdingHeading) {
            targetHeading = drivetrain.getHeading();
            holdingHeading = true;
        }
        if (forward == 0 && strafe == 0) return 0;

        double error = AngleUnit.normalizeRadians(drivetrain.getHeading() - targetHeading);
        return Range.clip(error * RobotConstants.HEADING_HOLD_KP, -1, 1);
    }

    private static double shape(double value) {
        if (Math.abs(value) < RobotConstants.STICK_DEADBAND) return 0;
        return Math.signum(value) * Math.pow(Math.abs(value), RobotConstants.DRIVE_INPUT_EXPONENT);
    }

    private static double approach(double current, double target, double maxStep) {
        return current + Range.clip(target - current, -maxStep, maxStep);
    }
}
