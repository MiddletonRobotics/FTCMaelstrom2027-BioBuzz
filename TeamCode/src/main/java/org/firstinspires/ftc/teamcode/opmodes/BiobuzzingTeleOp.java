package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "BiobuzzingTeleOp", group = "BioBuzz")
public class BiobuzzingTeleOp extends OpMode {
    private static final double DEADBAND = 0.1;

    private static final double UP_POWER = 1.0;
    private static final double DOWN_POWER = 0.5;
    private static final double HOLD_POWER = 0.3;
    private static final double PRESET_POWER = 0.8;
    private static final int REST_TICKS = 20;
    private static final int ONE_ROTATION = 385;

    private static final boolean[] REVERSED = {
            RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
            RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};

    private final List<String> warnings = new ArrayList<>();

    private DcMotorEx[] wheels;
    private DcMotorEx intake;
    private DcMotorEx elevator;
    private boolean holding = false;
    private boolean presetHeld = false;
    private boolean dropping = false;
    private String elevatorState = "";

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
        boolean up = gamepad1.triangle || gamepad2.triangle;
        boolean down = gamepad1.cross || gamepad2.cross;

        if (gamepad1.dpad_up) {
            if (!presetHeld) {
                presetHeld = true;
                dropping = false;
                holding = true;
                elevator.setTargetPosition(ONE_ROTATION);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(PRESET_POWER);
            }
            elevatorState = "going to 1 rotation";
            return;
        }
        if (presetHeld) {
            presetHeld = false;
            dropping = true;
            stopHolding();
            elevator.setPower(0);
        }

        if (up != down) {
            dropping = false;
            stopHolding();
            elevator.setPower(up ? UP_POWER : -DOWN_POWER);
            elevatorState = up ? "up" : "down";
        } else if (!dropping && position > REST_TICKS) {
            if (!holding) {
                holding = true;
                elevator.setTargetPosition(position);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(HOLD_POWER);
            }
            elevatorState = "holding";
        } else {
            if (position <= REST_TICKS) dropping = false;
            stopHolding();
            elevator.setPower(0);
            elevatorState = dropping ? "dropping" : "at bottom";
        }
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
