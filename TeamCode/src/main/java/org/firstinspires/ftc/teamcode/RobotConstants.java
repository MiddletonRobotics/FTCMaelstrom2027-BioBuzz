package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

public final class RobotConstants {
    private RobotConstants() {}

    public static final String FRONT_LEFT = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backRight";
    public static final String BACK_RIGHT = "backLeft";
    public static final int FRONT_LEFT_PORT = 2;
    public static final int FRONT_RIGHT_PORT = 3;
    public static final int BACK_LEFT_PORT = 0;
    public static final int BACK_RIGHT_PORT = 1;

    public static final String IMU = "imu";
    public static final String PINPOINT = "pinpoint";
    public static final String INTAKE = "intake";
    public static final String ELEVATOR = "elevator";
    public static final String FEEDER = "transfer";
    public static final String SHOOTER = "shooter";
    public static final String GATE_SERVO = "gateServo";
    public static final String DEFLOWER_SERVO = "deFlower";
    public static final String RAMP_SERVO = "rampServo";

    public static final boolean FRONT_LEFT_REVERSED = false;
    public static final boolean FRONT_RIGHT_REVERSED = true;
    public static final boolean BACK_LEFT_REVERSED = false;
    public static final boolean BACK_RIGHT_REVERSED = false;

    public static final double TURN_SCALE = 1.0;
    public static final double STICK_DEADBAND = 0.05;
    public static final double STRAFE_CORRECTION = 1.1;
    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final double AUTO_PARK_FORWARD_IN = 24;
    public static final double AUTO_PARK_STRAFE_IN = 0;
    public static final double AUTO_DRIVE_KP = 0.05;
    public static final double AUTO_HEADING_KP = 1.0;
    public static final double AUTO_MAX_POWER = 0.6;
    public static final double AUTO_MIN_POWER = 0.12;
    public static final double AUTO_TOLERANCE_IN = 1.0;

    public static final boolean INTAKE_REVERSED = false;
    public static final boolean ELEVATOR_REVERSED = false;
    public static final boolean FEEDER_REVERSED = false;
    public static final double INTAKE_MAX_TPS = 2600;
    public static final double FEEDER_MAX_TPS = 2600;
    public static final double INTAKE_SPEED = 0.85;
    public static final double FEEDER_SPEED = 0.85;

    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN = 0.7;
    public static final double DEFLOWER_CLOSED = 0.0;
    public static final double DEFLOWER_OPEN = 0.7;
    public static final double RAMP_DOWN = 0.55;
    public static final double RAMP_UP = 0.4;

    public static final boolean SHOOTER_REVERSED = true;
    public static final double SHOOTER_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_TARGET_RPM = 4400;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 200;
    public static final double SHOOTER_READY_S = 0.1;
    public static final double SHOOTER_KS = 0.0;
    public static final double SHOOTER_KV = 0.000167;
    public static final double SHOOTER_KP = 0.0003;
    public static final double SHOOTER_KI = 0.0005;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_I_LIMIT = 0.3;
    public static final double SHOOTER_BOOST_RPM = 300;
    public static final double LAUNCHER_FREE_RPM = 6000;
    public static final double LAUNCHER_SPINUP_S = 1.5;

    public static final LogoFacingDirection HUB_LOGO = LogoFacingDirection.UP;
    public static final UsbFacingDirection HUB_USB = UsbFacingDirection.FORWARD;
    public static final double POD_X_OFFSET = 0;
    public static final double POD_Y_OFFSET = 0;
    public static final boolean POD_X_REVERSED = false;
    public static final boolean POD_Y_REVERSED = false;
}
