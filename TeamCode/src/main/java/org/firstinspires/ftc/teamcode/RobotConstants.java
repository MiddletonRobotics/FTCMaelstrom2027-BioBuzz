package org.firstinspires.ftc.teamcode;

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
    public static final double STRAFE_CORRECTION = 1.1;
    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final boolean INTAKE_REVERSED = false;
    public static final boolean ELEVATOR_REVERSED = false;
    public static final boolean FEEDER_REVERSED = false;
    public static final double INTAKE_MAX_TPS = 2600;
    public static final double FEEDER_MAX_TPS = 2600;
    public static final double INTAKE_SPEED = 0.75;
    public static final double TRANSFER_POWER = 0.45;

    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN = 0.7;
    public static final double DEFLOWER_CLOSED = 0.0;
    public static final double DEFLOWER_OPEN = 0.889;
    public static final double RAMP_DOWN = 0.55;
    public static final double RAMP_UP = 0.4;

    public static final boolean SHOOTER_REVERSED = true;
    public static final double SHOOTER_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_TARGET_RPM = 3645;

    public static final double SHOOTER_IDLE_RPM = 225;
    public static final double SHOOTER_IDLE_KI = 0.00015;
    public static final double SHOOTER_IDLE_START_POWER = 0.06;
    public static final double SHOOTER_IDLE_MAX_POWER = 0.25;
    public static final double SHOOTER_AVG_WINDOW_S = 0.25;
    public static final double SHOOTER_LOW_RPM = 1000;
    public static final double SHOOTER_MIN_RPM = 3595;
    public static final double AUTO_SHOOTER_RPM = 4283;
    public static final double SHOOTER_MAX_RPM = 3695;
    public static final double SHOOTER_GUARD_RPM = 25;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 100;

    public static final double SHOOTER_READY_S = 0.1;
    public static final double SHOOTER_KS = 0.0;
    public static final double SHOOTER_KV = 0.000167;
    public static final double SHOOTER_KP = 0.0003;
    public static final double SHOOTER_KI = 0.0005;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_I_LIMIT = 0.3;
    public static final double SHOOTER_BOOST_RPM = 300;
    public static final double LAUNCHER_SPINUP_S = 1.5;
    public static final double POD_X_OFFSET = 0;
    public static final double POD_Y_OFFSET = 0;
    public static final boolean POD_X_REVERSED = false;
    public static final boolean POD_Y_REVERSED = false;
}
