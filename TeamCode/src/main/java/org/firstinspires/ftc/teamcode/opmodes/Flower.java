package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;

import java.util.ArrayList;
import java.util.List;

/*
 * gamepad1:
 *   left stick up/down = drive forward/back, left stick left/right = strafe
 *   right stick = rotate only: the robot turns to face where the stick points, all 360
 *     (up = the way the robot faced at START), let go = stops turning
 *   L2 = intake, full speed    L1 = reverse intake, full speed
 *   hold triangle = elevator up, hold cross = elevator down, let go = holds in place
 *   R1 / R2 = nothing (saved for the shooter)
 * gamepad2 (optional): same L1/L2/triangle/cross
 * either gamepad: hold dpad up/down = elevator at half power, ignoring the limits
 *   holding dpad down into the bottom sets that as the new bottom
 *
 * Elevator must be all the way DOWN when INIT is pressed.
 */
@TeleOp(name = "flower", group = "BioBuzz")
public class Flower extends OpMode {
    private static final int INTAKE_PORT = 0;
    private static final int ELEVATOR_PORT = 1;

    private static final double STICK_DEADBAND = 0.1;
    // Encoder speed matching for the wheels; off until the encoder wiring is checked
    private static final boolean SPEED_CONTROL = false;

    private static final double AIM_STICK_MIN = 0.5;
    private static final double AIM_KP = 1.0;
    private static final double AIM_KD = 0.05;
    private static final double AIM_TOLERANCE = Math.toRadians(1.5);

    private static final String[] WHEELS = {"FL", "FR", "BL", "BR"};
    private static final boolean[] REVERSED = {
            RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
            RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};
    private static final double[] MAX_TPS = {
            RobotConstants.FRONT_LEFT_MAX_TPS, RobotConstants.FRONT_RIGHT_MAX_TPS,
            RobotConstants.BACK_LEFT_MAX_TPS, RobotConstants.BACK_RIGHT_MAX_TPS};

    private static final double ELEVATOR_UP_POWER = 1.0;
    private static final double ELEVATOR_DOWN_POWER = 0.5;
    private static final double ELEVATOR_UP_SLOW_POWER = 0.5;
    private static final double ELEVATOR_DPAD_POWER = 0.5;
    private static final double ELEVATOR_HOLD_POWER = 0.8;
    private static final double ELEVATOR_DOWN_SLOW_POWER = 0.3;
    private static final int ELEVATOR_SLOW_ZONE = 200;
    private static final int ELEVATOR_REST_TICKS = 20;
    private static final int ELEVATOR_HOLD_SLIP = 150;
    private static final double ELEVATOR_STALL_AMPS = 8.0;
    private static final double ELEVATOR_STALL_TIME_S = 1.0;
    private static final double ELEVATOR_HOME_TIME_S = 0.3;

    private List<LynxModule> hubs;
    private final List<String> warnings = new ArrayList<>();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private DcMotorEx[] wheels;
    private final double[] wheelVelocity = new double[4];
    private final boolean[] openLoop = new boolean[4];
    private final ElapsedTime[] wheelCheck = {
            new ElapsedTime(), new ElapsedTime(), new ElapsedTime(), new ElapsedTime()};
    private double driveSpeed = Double.MAX_VALUE;

    private IMU imu;
    private boolean aimWorks = true;
    private double heading = 0;
    private final ElapsedTime aimCheck = new ElapsedTime();

    private DcMotorEx intake;

    private DcMotorEx elevator;
    private boolean elevatorEncoderOk = true;
    private boolean elevatorEverMoved = false;
    private double elevatorStuckTime = 0;
    private boolean holding = false;
    private int holdTarget = 0;
    private boolean stalled = false;
    private boolean homed = false;
    private String elevatorState = "";
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime homeTimer = new ElapsedTime();
    private final ElapsedTime backwardsTimer = new ElapsedTime();

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        wheels = MotorFinder.drive(hardwareMap, warnings);
        for (int i = 0; i < 4; i++) {
            DcMotorEx wheel = wheels[i];
            wheel.setDirection(REVERSED[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            openLoop[i] = !SPEED_CONTROL;
            wheel.setMode(SPEED_CONTROL ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // REV's velocity tuning: F = full output at max speed, P = F / 10, I = P / 10
            double f = 32767 / MAX_TPS[i];
            double p = 0.1 * f;
            wheel.setVelocityPIDFCoefficients(p, 0.1 * p, 0, f);
            driveSpeed = Math.min(driveSpeed, MAX_TPS[i]);
        }

        imu = hardwareMap.tryGet(IMU.class, RobotConstants.IMU);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RobotConstants.HUB_LOGO, RobotConstants.HUB_USB)));
            imu.resetYaw();
        } else {
            warnings.add("no \"imu\" in config, right stick left/right just spins the robot");
        }

        intake = MotorFinder.expansion(hardwareMap, RobotConstants.INTAKE, INTAKE_PORT, wheels[0], warnings);
        intake.setDirection(RobotConstants.INTAKE_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        elevator = MotorFinder.expansion(hardwareMap, RobotConstants.ELEVATOR, ELEVATOR_PORT, wheels[0], warnings);
        elevator.setDirection(RobotConstants.ELEVATOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetElevator();
        elevator.setPower(0);

        MotorFinder.checkUnique(wheels[0], wheels[1], wheels[2], wheels[3], intake, elevator);

        telemetry.addLine("flower ready. Elevator must be all the way DOWN. Press START.");
        for (String warning : warnings) telemetry.addLine("! " + warning);
    }

    @Override
    public void start() {
        loopTimer.reset();
        if (imu != null) imu.resetYaw();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        for (LynxModule hub : hubs) hub.clearBulkCache();

        drive();
        runIntake();
        runElevator(dt);

        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addData("Heading", "%.0f deg", Math.toDegrees(-heading));
        telemetry.addData("Wheel speed FL/FR/BL/BR", "%.0f / %.0f / %.0f / %.0f",
                wheelVelocity[0], wheelVelocity[1], wheelVelocity[2], wheelVelocity[3]);
        telemetry.addData("Intake", "%.2f", intake.getPower());
        telemetry.addData("Elevator", "%d / %d ticks  %s", elevator.getCurrentPosition(),
                RobotConstants.ELEVATOR_MAX, elevatorState);
        telemetry.update();
    }

    private void drive() {
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x) * RobotConstants.STRAFE_CORRECTION;
        double turn = aimTurn();

        double[] power = {
                forward + strafe + turn,
                forward - strafe - turn,
                forward - strafe + turn,
                forward + strafe - turn};
        double max = 1;
        for (double p : power) max = Math.max(max, Math.abs(p));
        double scale = 1 / max;

        double fastest = 0;
        for (int i = 0; i < 4; i++) {
            wheelVelocity[i] = wheels[i].getVelocity();
            fastest = Math.max(fastest, Math.abs(wheelVelocity[i]));
        }
        for (int i = 0; i < 4; i++) {
            power[i] *= scale;
            if (!openLoop[i]) checkWheelEncoder(i, power[i], wheelVelocity[i], fastest);
            // Speed control keeps all four wheels matched; full stick asks for more than
            // the slowest wheel can do loaded, so every motor still gets full power
            if (openLoop[i]) wheels[i].setPower(power[i]);
            else wheels[i].setVelocity(power[i] * driveSpeed);
        }
    }

    // Turn toward where the right stick points. Positive turn = clockwise; the gyro's yaw is counterclockwise
    private double aimTurn() {
        double x = gamepad1.right_stick_x;
        double y = -gamepad1.right_stick_y;
        if (imu == null || !aimWorks) return deadband(x) * RobotConstants.TURN_SCALE;

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double yawRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        if (Math.hypot(x, y) < AIM_STICK_MIN) {
            aimCheck.reset();
            return 0;
        }
        double target = Math.atan2(-x, y);
        double error = AngleUnit.normalizeRadians(target - heading);
        if (Math.abs(error) < AIM_TOLERANCE) {
            aimCheck.reset();
            return 0;
        }
        double turn = Range.clip(-error * AIM_KP + yawRate * AIM_KD, -1, 1);

        // Gyro set up wrong: the robot turns away from the stick and would spin forever
        if (Math.abs(turn) > 0.3 && turn * yawRate > 0 && Math.abs(yawRate) > 0.5) {
            if (aimCheck.seconds() > 0.5) {
                aimWorks = false;
                warnings.add("gyro turns the wrong way (check HUB_LOGO/HUB_USB), right stick left/right just spins");
                gamepad1.rumble(500);
                return 0;
            }
        } else {
            aimCheck.reset();
        }
        return turn;
    }

    // A dead or backwards encoder would make speed control run the wheel wild, so drop that wheel to plain power
    private void checkWheelEncoder(int i, double power, double velocity, double fastest) {
        boolean dead = Math.abs(power) > 0.3 && Math.abs(velocity) < 30 && fastest > 600;
        boolean runaway = power == 0 ? Math.abs(velocity) > 1000
                : power * velocity < 0 && Math.abs(velocity) > 300;
        if (!dead && !runaway) {
            wheelCheck[i].reset();
            return;
        }
        if (wheelCheck[i].seconds() < (dead ? 0.5 : 1.0)) return;
        openLoop[i] = true;
        wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        warnings.add(WHEELS[i] + (dead ? " encoder not reading" : " encoder backwards")
                + ", wheel running without speed control");
        gamepad1.rumble(500);
    }

    private void runIntake() {
        boolean in = gamepad1.left_trigger > STICK_DEADBAND || gamepad2.left_trigger > STICK_DEADBAND;
        boolean out = gamepad1.left_bumper || gamepad2.left_bumper;
        double power = in == out ? 0 : in ? 1 : -1;
        intake.setPower(power);
    }

    private void runElevator(double dt) {
        int position = elevator.getCurrentPosition();
        double velocity = elevator.getVelocity();
        if (Math.abs(position) > 50) elevatorEverMoved = true;

        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean dpad = dpadUp != dpadDown;

        double power;
        if (dpad) {
            power = dpadUp ? ELEVATOR_DPAD_POWER : -ELEVATOR_DPAD_POWER;
            elevatorState = "dpad, no limits";
        } else {
            boolean up = gamepad1.triangle || gamepad2.triangle;
            boolean down = gamepad1.cross || gamepad2.cross;
            power = up == down ? 0 : up ? ELEVATOR_UP_POWER : -ELEVATOR_DOWN_POWER;
            if (elevatorEncoderOk) power = applyLimits(power, position);
        }

        // Holding dpad down into the hard stop: that's the real bottom
        if (dpad && dpadDown && Math.abs(velocity) < 20) {
            if (!homed && homeTimer.seconds() > ELEVATOR_HOME_TIME_S) {
                resetElevator();
                position = 0;
                homed = true;
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
        } else {
            homeTimer.reset();
        }
        if (!(dpad && dpadDown)) homed = false;
        if (homed) {
            power = 0;
            elevatorState = "bottom set";
        }

        if (elevatorEncoderOk && !dpad) checkElevatorEncoder(power, velocity, dt);

        if (power == 0) {
            stalled = false;
            stallTimer.reset();
        } else if (elevator.getCurrent(CurrentUnit.AMPS) < ELEVATOR_STALL_AMPS) {
            stallTimer.reset();
        } else if (!stalled && stallTimer.seconds() > ELEVATOR_STALL_TIME_S) {
            stalled = true;
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }
        if (stalled) power = 0;

        if (power != 0) {
            stopHolding();
            elevator.setPower(power);
            if (!dpad) elevatorState = elevatorEncoderOk ? "moving" : "moving, NO LIMITS";
        } else if (elevatorEncoderOk && !stalled && position > ELEVATOR_REST_TICKS) {
            if (!holding) {
                holding = true;
                holdTarget = position;
                elevator.setTargetPosition(holdTarget);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(ELEVATOR_HOLD_POWER);
            } else if (Math.abs(position - holdTarget) > ELEVATOR_HOLD_SLIP) {
                elevatorProblem("elevator slipped while holding");
            }
            if (holding) elevatorState = "holding";
        } else {
            stopHolding();
            elevator.setPower(0);
            if (stalled) elevatorState = "STALLED, let go";
            else if (!homed) elevatorState = elevatorEncoderOk ? "at bottom" : "stopped, NO LIMITS";
        }
    }

    private static double applyLimits(double power, int position) {
        if (power > 0) {
            int room = RobotConstants.ELEVATOR_MAX - position;
            if (room <= 0) return 0;
            return room < ELEVATOR_SLOW_ZONE ? Math.min(power, ELEVATOR_UP_SLOW_POWER) : power;
        }
        if (power < 0) {
            int room = position - RobotConstants.ELEVATOR_DOWN;
            if (room <= 0) return 0;
            return room < ELEVATOR_SLOW_ZONE ? Math.max(power, -ELEVATOR_DOWN_SLOW_POWER) : power;
        }
        return 0;
    }

    // Limits and hold need a working encoder that counts up when the elevator goes up
    private void checkElevatorEncoder(double power, double velocity, double dt) {
        if (!elevatorEverMoved && Math.abs(power) >= ELEVATOR_DOWN_POWER) elevatorStuckTime += dt;
        if (elevatorStuckTime > 0.75) {
            elevatorProblem("elevator encoder not reading (or elevator not moving)");
            return;
        }
        if (Math.abs(power) >= ELEVATOR_DOWN_POWER && power * velocity < 0 && Math.abs(velocity) > 200) {
            if (backwardsTimer.seconds() > 0.5) {
                elevatorProblem("elevator moving opposite to the stick (encoder backwards or too heavy)");
            }
        } else {
            backwardsTimer.reset();
        }
    }

    private void elevatorProblem(String message) {
        elevatorEncoderOk = false;
        stopHolding();
        elevator.setPower(0);
        warnings.add(message + ", limits and hold are OFF");
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }

    private void stopHolding() {
        if (!holding) return;
        holding = false;
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetElevator() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        holding = false;
    }

    private static double deadband(double value) {
        if (Math.abs(value) < STICK_DEADBAND) return 0;
        return Math.signum(value) * (Math.abs(value) - STICK_DEADBAND) / (1 - STICK_DEADBAND);
    }

    @Override
    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
        intake.setPower(0);
        elevator.setPower(0);
    }
}
