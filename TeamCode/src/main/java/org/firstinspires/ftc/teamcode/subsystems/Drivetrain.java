package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Iterator;

public class Drivetrain {
    public enum Wheel { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT }

    private static final double POWER_EPSILON = 0.001;
    private static final double VOLTAGE_READ_INTERVAL = 0.5;

    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final boolean[] reversed = {
            RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
            RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};
    private final double[] lastPower = new double[4];

    private final IMU imu;
    private double heading = 0;

    private final VoltageSensor voltageSensor;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltageScale = 1.0;

    public final Odometry odometry = new Odometry();

    public Drivetrain(HardwareMap hardwareMap) {
        motors[0] = findMotor(hardwareMap, RobotConstants.FRONT_LEFT, RobotConstants.FRONT_LEFT_PORT);
        motors[1] = findMotor(hardwareMap, RobotConstants.FRONT_RIGHT, RobotConstants.FRONT_RIGHT_PORT);
        motors[2] = findMotor(hardwareMap, RobotConstants.BACK_LEFT, RobotConstants.BACK_LEFT_PORT);
        motors[3] = findMotor(hardwareMap, RobotConstants.BACK_RIGHT, RobotConstants.BACK_RIGHT_PORT);

        for (Wheel wheel : Wheel.values()) applyDirection(wheel);
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(RobotConstants.DRIVE_BRAKE
                    ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.tryGet(IMU.class, RobotConstants.IMU);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RobotConstants.HUB_LOGO, RobotConstants.HUB_USB)));
            imu.resetYaw();
        }

        Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        voltageSensor = sensors.hasNext() ? sensors.next() : null;
    }

    private static DcMotorEx findMotor(HardwareMap hardwareMap, String name, int port) {
        DcMotorEx byName = hardwareMap.tryGet(DcMotorEx.class, name);
        if (byName != null) return byName;
        for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
            if (motor.getPortNumber() == port) return motor;
        }
        throw new IllegalStateException("No drive motor \"" + name + "\" and nothing configured on"
                + " motor port " + port + ". On the Driver Hub: Configure Robot > add all 4 drive"
                + " motors (port 0 backRight, 1 backLeft, 2 frontLeft, 3 frontRight) > Save > Activate.");
    }

    public void drive(double forward, double strafe, double turn) {
        strafe *= RobotConstants.STRAFE_CORRECTION;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        double scale = RobotConstants.DRIVE_MAX_POWER * voltageScale / max;

        setPower(Wheel.FRONT_LEFT, fl * scale);
        setPower(Wheel.FRONT_RIGHT, fr * scale);
        setPower(Wheel.BACK_LEFT, bl * scale);
        setPower(Wheel.BACK_RIGHT, br * scale);
    }

    public void driveFieldCentric(double forward, double strafe, double turn) {
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        drive(strafe * sin + forward * cos, strafe * cos - forward * sin, turn);
    }

    public void stop() {
        for (Wheel wheel : Wheel.values()) setPower(wheel, 0);
    }

    public void setPower(Wheel wheel, double power) {
        power = Range.clip(power, -1, 1);
        int i = wheel.ordinal();
        if (Math.abs(power - lastPower[i]) > POWER_EPSILON || (power == 0 && lastPower[i] != 0)) {
            motors[i].setPower(power);
            lastPower[i] = power;
        }
    }

    public void setReversed(Wheel wheel, boolean isReversed) {
        reversed[wheel.ordinal()] = isReversed;
        applyDirection(wheel);
    }

    public boolean isReversed(Wheel wheel) {
        return reversed[wheel.ordinal()];
    }

    private void applyDirection(Wheel wheel) {
        motors[wheel.ordinal()].setDirection(isReversed(wheel)
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        if (imu != null) heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        updateVoltageScale();
        odometry.update(getPosition(Wheel.FRONT_LEFT), getPosition(Wheel.FRONT_RIGHT),
                getPosition(Wheel.BACK_LEFT), getPosition(Wheel.BACK_RIGHT), heading);
    }

    private void updateVoltageScale() {
        if (!RobotConstants.VOLTAGE_COMPENSATION || voltageSensor == null) {
            voltageScale = 1.0;
            return;
        }
        if (voltageTimer.seconds() < VOLTAGE_READ_INTERVAL) return;
        voltageTimer.reset();
        double volts = voltageSensor.getVoltage();
        if (volts > 1) voltageScale = Math.min(1.0, RobotConstants.NOMINAL_VOLTAGE / volts);
    }

    public boolean hasImu() {
        return imu != null;
    }

    public double getHeading() {
        return heading;
    }

    public void resetHeading() {
        if (imu != null) imu.resetYaw();
        heading = 0;
    }

    public int getPosition(Wheel wheel) {
        return motors[wheel.ordinal()].getCurrentPosition();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Pose (in)", "x %.1f  y %.1f", odometry.getX(), odometry.getY());
        telemetry.addData("Encoders FL/FR/BL/BR", "%d / %d / %d / %d",
                getPosition(Wheel.FRONT_LEFT), getPosition(Wheel.FRONT_RIGHT),
                getPosition(Wheel.BACK_LEFT), getPosition(Wheel.BACK_RIGHT));
        if (RobotConstants.VOLTAGE_COMPENSATION) {
            telemetry.addData("Voltage scale", "%.2f", voltageScale);
        }
    }
}
