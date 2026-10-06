package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Every hardware name and tunable number for BIOBUZZ lives here.
 * The names MUST match the Driver Hub configuration exactly (case-sensitive).
 */
public final class RobotConstants {
    private RobotConstants() {}

    // ---------------- Hardware names (Driver Hub -> Configure Robot) ----------------
    // Drivetrain
    public static final String FRONT_LEFT  = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT   = "LeftRear";
    public static final String BACK_RIGHT  = "RightRear";
    public static final String IMU         = "imu";  // Control Hub built-in IMU, I2C bus 0

    // Intake
    public static final String INTAKE = "intake";

    // Transfer
    public static final String ELEVATOR     = "elevator";
    public static final String FEEDER       = "transfer";      // motor that pushes into the shooter
    public static final String FLOWER_SERVO = "flowerServo";
    public static final String GATE_SERVO   = "gateServo";     // opens access to shooter

    // Shooter
    public static final String SHOOTER       = "shooter";
    public static final String SHOOTER_SERVO = "shooterServo"; // optional, ok if not configured

    // ---------------- Drivetrain ----------------
    // Which way the Control Hub is mounted on the robot. Only matters for field-centric + odometry heading.
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static final double STICK_DEADBAND  = 0.05;
    // Mecanum strafes slower than it drives; >1 boosts strafe so it feels even.
    public static final double STRAFE_CORRECTION = 1.1;

    // goBILDA 5203 312 RPM motor + goBILDA 104 mm mecanum wheel.
    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_IN   = 104.0 / 25.4;
    public static final double DRIVE_GEAR_RATIO    = 1.0; // wheel revs per motor rev
    // Measured strafe distance / encoder strafe distance. Tune by strafing 48 in and comparing.
    public static final double ODOMETRY_STRAFE_MULTIPLIER = 1.0;

    // ---------------- Intake ----------------
    public static final double INTAKE_POWER = 1.0;

    // ---------------- Elevator (encoder ticks, 0 = bottom at init) ----------------
    // Placeholder values: tune on the robot by reading "Elevator ticks" in telemetry.
    public static final int    ELEVATOR_DOWN     = 0;
    public static final int    ELEVATOR_TRANSFER = 400;
    public static final int    ELEVATOR_UP       = 1200;
    public static final int    ELEVATOR_MAX      = 1300;
    public static final double ELEVATOR_POWER    = 0.8;
    public static final double ELEVATOR_MANUAL_TICKS_PER_LOOP = 15;

    // ---------------- Transfer ----------------
    public static final double FEEDER_POWER = 1.0;
    public static final double FLOWER_CLOSED = 0.0;
    public static final double FLOWER_OPEN   = 1.0;
    public static final double GATE_CLOSED   = 0.0;
    public static final double GATE_OPEN     = 1.0;

    // ---------------- Shooter ----------------
    public static final double SHOOTER_TICKS_PER_REV = 28.0; // bare goBILDA/REV motor encoder at the motor shaft
    public static final double SHOOTER_TARGET_RPM    = 4500;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 150;
    public static final double SHOOTER_SERVO_A = 0.0;
    public static final double SHOOTER_SERVO_B = 1.0;
}
