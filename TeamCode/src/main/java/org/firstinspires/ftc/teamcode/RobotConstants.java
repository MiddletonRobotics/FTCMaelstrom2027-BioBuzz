package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class RobotConstants {
    private RobotConstants() {}

    // Hardware names (must match the Driver Hub configuration)
    public static final String FRONT_LEFT = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backLeft";
    public static final String BACK_RIGHT = "backRight";
    public static final int BACK_RIGHT_PORT = 0;
    public static final int BACK_LEFT_PORT = 1;
    public static final int FRONT_LEFT_PORT = 2;
    public static final int FRONT_RIGHT_PORT = 3;
    public static final String IMU = "imu";
    public static final String INTAKE = "intake";
    public static final String ELEVATOR = "elevator";
    public static final String FEEDER = "transfer";
    public static final String FLOWER_SERVO = "flowerServo";
    public static final String GATE_SERVO = "gateServo";
    public static final String SHOOTER = "shooter";
    public static final String SHOOTER_SERVO = "shooterServo";
    public static final String WEBCAM = "Webcam 1";

    // Camera, inches from robot center (+x right, +y forward, +z up).
    // Pitch: 0 = straight up, -90 = level.
    public static final double CAMERA_X_IN = 0;
    public static final double CAMERA_Y_IN = 0;
    public static final double CAMERA_Z_IN = 8;
    public static final double CAMERA_YAW_DEG = 0;
    public static final double CAMERA_PITCH_DEG = -45;
    public static final double CAMERA_ROLL_DEG = 0;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;

    // Drivetrain
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static final boolean FRONT_LEFT_REVERSED = true;
    public static final boolean FRONT_RIGHT_REVERSED = false;
    public static final boolean BACK_LEFT_REVERSED = false;
    public static final boolean BACK_RIGHT_REVERSED = false;

    public static final boolean DRIVE_BRAKE = true;
    public static final double DRIVE_MAX_POWER = 1.0;
    public static final double TURN_SCALE = 1.0;
    public static final double STICK_DEADBAND = 0.05;
    public static final double DRIVE_INPUT_EXPONENT = 1.0;
    public static final double STRAFE_CORRECTION = 1.1;
    public static final double DRIVE_ACCEL_LIMIT = 0; // power per second, 0 = off

    public static final boolean HEADING_HOLD_ENABLED = false;
    public static final double HEADING_HOLD_KP = 1.0;
    public static final double HEADING_HOLD_DELAY_S = 0.25;

    public static final boolean VOLTAGE_COMPENSATION = false;
    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_IN = 104.0 / 25.4;
    public static final double DRIVE_GEAR_RATIO = 1.0;
    public static final double ODOMETRY_STRAFE_MULTIPLIER = 1.0;

    // Park auto
    public static final double AUTO_PARK_FORWARD_IN = 24;
    public static final double AUTO_PARK_STRAFE_IN = 0;
    public static final double AUTO_DRIVE_KP = 0.05;
    public static final double AUTO_HEADING_KP = 1.0;
    public static final double AUTO_MAX_POWER = 0.6;
    public static final double AUTO_MIN_POWER = 0.12;
    public static final double AUTO_TOLERANCE_IN = 1.0;

    // Intake
    public static final boolean INTAKE_REVERSED = false;
    public static final double INTAKE_POWER = 1.0;
    public static final boolean INTAKE_JAM_DETECTION = false;
    public static final double INTAKE_JAM_AMPS = 7.0;
    public static final double INTAKE_JAM_TIME_S = 0.25;
    public static final double INTAKE_UNJAM_TIME_S = 0.3;

    // Elevator (encoder ticks, 0 = bottom at init)
    public static final int ELEVATOR_DOWN = 0;
    public static final int ELEVATOR_TRANSFER = 400;
    public static final int ELEVATOR_UP = 1200;
    public static final int ELEVATOR_MAX = 1300;
    public static final double ELEVATOR_POWER = 0.8;
    public static final double ELEVATOR_MANUAL_TICKS_PER_LOOP = 15;
    public static final boolean ELEVATOR_REVERSED = false;
    public static final int ELEVATOR_TOLERANCE = 15;
    public static final boolean ELEVATOR_RELAX_AT_BOTTOM = true;

    // Transfer
    public static final boolean FEEDER_REVERSED = false;
    public static final double FEEDER_POWER = 1.0;
    public static final double FLOWER_CLOSED = 0.0;
    public static final double FLOWER_OPEN = 1.0;
    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN = 1.0;

    // Shooter
    public static final boolean SHOOTER_REVERSED = false;
    public static final double SHOOTER_RPM_STEP = 100;
    public static final double SHOOTER_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_TARGET_RPM = 4500;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 150;
    public static final double SHOOTER_P = 10.0;
    public static final double SHOOTER_I = 3.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 12.0;
    public static final double SHOOTER_SERVO_A = 0.0;
    public static final double SHOOTER_SERVO_B = 1.0;
}
