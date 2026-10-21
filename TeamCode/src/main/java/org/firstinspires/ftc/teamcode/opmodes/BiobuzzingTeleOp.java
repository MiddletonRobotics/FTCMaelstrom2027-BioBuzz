package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "BiobuzzingTeleOp", group = "BioBuzz")
public class BiobuzzingTeleOp extends OpMode {
    private static final double DEADBAND = 0.1;

    private static final double UP_POWER = 1.0;
    private static final double DOWN_POWER = 0.5;
    private static final double UP_SLOW_POWER = 0.5;
    private static final double DOWN_SLOW_POWER = 0.3;
    private static final double DPAD_POWER = 0.5;
    private static final double HOLD_POWER = 0.8;
    private static final int SLOW_ZONE = 200;
    private static final int REST_TICKS = 20;
    private static final double STALL_AMPS = 8.0;
    private static final double STALL_TIME = 1.0;
    private static final double HOME_TIME = 0.3;

    private static final boolean[] REVERSED = {
            RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
            RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};

    private final List<String> warnings = new ArrayList<>();

    private DcMotorEx[] wheels;
    private DcMotorEx intake;
    private DcMotorEx elevator;
    private boolean holding = false;
    private boolean stalled = false;
    private boolean homed = false;
    private String elevatorState = "";
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime homeTimer = new ElapsedTime();

    @Override
    public void init() {
        wheels = MotorFinder.drive(hardwareMap, warnings);
        for (int i = 0; i < 4; i++) {
            wheels[i].setDirection(direction(REVERSED[i]));
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = MotorFinder.expansion(hardwareMap, RobotConstants.INTAKE, 0, wheels[0], warnings);
        intake.setDirection(direction(RobotConstants.INTAKE_REVERSED));
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevator = MotorFinder.expansion(hardwareMap, RobotConstants.ELEVATOR, 1, wheels[0], warnings);
        elevator.setDirection(direction(RobotConstants.ELEVATOR_REVERSED));
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetElevator();

        telemetry.addLine("BiobuzzingTeleOp ready. Elevator must be all the way DOWN.");
        for (String warning : warnings) telemetry.addLine("! " + warning);
    }

    @Override
    public void loop() {
        drive();
        runIntake();
        runElevator();

        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addData("Intake", "%.2f", intake.getPower());
        telemetry.addData("Elevator", "%d / %d ticks  %s", elevator.getCurrentPosition(),
                RobotConstants.ELEVATOR_MAX, elevatorState);
        telemetry.update();
    }

    private void drive() {
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x) * RobotConstants.STRAFE_CORRECTION;
        double turn = deadband(gamepad1.right_stick_x) * RobotConstants.TURN_SCALE;
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        wheels[0].setPower((forward + strafe + turn) / max);
        wheels[1].setPower((forward - strafe - turn) / max);
        wheels[2].setPower((forward - strafe + turn) / max);
        wheels[3].setPower((forward + strafe - turn) / max);
    }

    private void runIntake() {
        boolean in = gamepad1.left_trigger > DEADBAND || gamepad2.left_trigger > DEADBAND;
        boolean out = gamepad1.left_bumper || gamepad2.left_bumper;
        intake.setPower(in == out ? 0 : in ? 1 : -1);
    }

    private void runElevator() {
        int position = elevator.getCurrentPosition();
        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean dpad = dpadUp != dpadDown;
        boolean up = gamepad1.triangle || gamepad2.triangle;
        boolean down = gamepad1.cross || gamepad2.cross;

        double power;
        if (dpad) power = dpadUp ? DPAD_POWER : -DPAD_POWER;
        else power = limit(up == down ? 0 : up ? UP_POWER : -DOWN_POWER, position);

        boolean pushingDown = dpad && dpadDown;
        if (!pushingDown) homed = false;
        if (!pushingDown || Math.abs(elevator.getVelocity()) >= 20) {
            homeTimer.reset();
        } else if (!homed && homeTimer.seconds() > HOME_TIME) {
            resetElevator();
            position = 0;
            homed = true;
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }
        if (homed) power = 0;

        if (power == 0) {
            stalled = false;
            stallTimer.reset();
        } else if (elevator.getCurrent(CurrentUnit.AMPS) < STALL_AMPS) {
            stallTimer.reset();
        } else if (!stalled && stallTimer.seconds() > STALL_TIME) {
            stalled = true;
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }
        if (stalled) power = 0;

        if (power != 0) {
            stopHolding();
            elevator.setPower(power);
        } else if (!stalled && position > REST_TICKS) {
            if (!holding) {
                holding = true;
                elevator.setTargetPosition(position);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(HOLD_POWER);
            }
        } else {
            stopHolding();
            elevator.setPower(0);
        }

        elevatorState = stalled ? "STALLED, let go"
                : homed ? "bottom set"
                : power != 0 ? (dpad ? "dpad, no limits" : "moving")
                : holding ? "holding" : "at bottom";
    }

    private static double limit(double power, int position) {
        if (power == 0) return 0;
        int room = power > 0 ? RobotConstants.ELEVATOR_MAX - position : position - RobotConstants.ELEVATOR_DOWN;
        if (room <= 0) return 0;
        if (room >= SLOW_ZONE) return power;
        return power > 0 ? Math.min(power, UP_SLOW_POWER) : Math.max(power, -DOWN_SLOW_POWER);
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

    private static DcMotorSimple.Direction direction(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static double deadband(double value) {
        if (Math.abs(value) < DEADBAND) return 0;
        return Math.signum(value) * (Math.abs(value) - DEADBAND) / (1 - DEADBAND);
    }

    @Override
    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
        intake.setPower(0);
        elevator.setPower(0);
    }
}
