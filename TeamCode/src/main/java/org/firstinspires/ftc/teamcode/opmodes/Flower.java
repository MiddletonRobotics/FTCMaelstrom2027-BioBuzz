package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;
import org.firstinspires.ftc.teamcode.subsystems.Servos;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "flower", group = "BioBuzz")
public class Flower extends OpMode {
    private static final double DEADBAND = 0.1;

    private static final double UP_POWER = 1.0;
    private static final double DOWN_POWER = 0.5;
    private static final double UP_SLOW_POWER = 0.5;
    private static final double DOWN_SLOW_POWER = 0.3;
    private static final double DPAD_POWER = 0.5;
    private static final double HOLD_POWER = 0.8;
    private static final int SLOW_ZONE = 200;
    private static final int REST_TICKS = 20;

    private static final boolean[] REVERSED = {
            RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
            RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};

    private List<LynxModule> hubs;
    private final List<String> warnings = new ArrayList<>();

    private DcMotorEx[] wheels;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private DcMotorEx elevator;
    private boolean holding = false;
    private String elevatorState = "";

    private Buttons buttons;
    private Servos servos;

    @Override
    public void init() {
        buttons = new Buttons(gamepad1, gamepad2);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        wheels = MotorFinder.drive(hardwareMap, warnings);
        for (int i = 0; i < 4; i++) {
            wheels[i].setDirection(MotorFinder.direction(REVERSED[i]));
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = MotorFinder.expansion(hardwareMap, RobotConstants.INTAKE, 0, wheels[0], warnings);
        intake.setDirection(MotorFinder.direction(RobotConstants.INTAKE_REVERSED));
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevator = MotorFinder.expansion(hardwareMap, RobotConstants.ELEVATOR, 1, wheels[0], warnings);
        elevator.setDirection(MotorFinder.direction(RobotConstants.ELEVATOR_REVERSED));
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetElevator();

        transfer = MotorFinder.expansion(hardwareMap, RobotConstants.FEEDER, 2, wheels[0], warnings);
        transfer.setDirection(MotorFinder.direction(RobotConstants.FEEDER_REVERSED));
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorFinder.checkUnique(wheels[0], wheels[1], wheels[2], wheels[3], intake, elevator, transfer);

        servos = new Servos(hardwareMap, warnings);

        telemetry.addLine("flower ready. Elevator must be all the way DOWN. Press START.");
        for (String warning : warnings) telemetry.addLine("! " + warning);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        drive();
        runIntake();
        runTransfer();
        runElevator();
        servos.update(buttons.gate(), buttons.ramp());

        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addData("Intake", "%.2f", intake.getPower());
        telemetry.addData("Transfer", "%.2f", transfer.getPower());
        telemetry.addData("Elevator", "%d / %d ticks  %s", elevator.getCurrentPosition(),
                RobotConstants.ELEVATOR_MAX, elevatorState);
        servos.addTelemetry(telemetry);
        telemetry.update();
    }

    private void drive() {
        double forward = deadband(buttons.forward());
        double strafe = deadband(buttons.strafe()) * RobotConstants.STRAFE_CORRECTION;
        double turn = deadband(buttons.turn()) * RobotConstants.TURN_SCALE;
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        wheels[0].setPower((forward + strafe + turn) / max);
        wheels[1].setPower((forward - strafe - turn) / max);
        wheels[2].setPower((forward - strafe + turn) / max);
        wheels[3].setPower((forward + strafe - turn) / max);
    }

    private void runIntake() {
        boolean in = buttons.intakeIn();
        boolean out = buttons.intakeOut();
        intake.setPower(in == out ? 0 : in ? 1 : -1);
    }

    private void runTransfer() {
        boolean in = buttons.transferIn();
        boolean out = buttons.transferOut();
        transfer.setPower(in == out ? 0 : in ? RobotConstants.FEEDER_POWER : -RobotConstants.FEEDER_POWER);
    }

    private void runElevator() {
        int position = elevator.getCurrentPosition();
        boolean dpadUp = buttons.elevatorNudgeUp();
        boolean dpadDown = buttons.elevatorNudgeDown();
        boolean dpad = dpadUp != dpadDown;
        boolean up = buttons.elevatorUp();
        boolean down = buttons.elevatorDown();

        double power;
        if (dpad) power = dpadUp ? DPAD_POWER : -DPAD_POWER;
        else power = limit(up == down ? 0 : up ? UP_POWER : -DOWN_POWER, position);

        if (power != 0) {
            stopHolding();
            elevator.setPower(power);
        } else if (position > REST_TICKS) {
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

        elevatorState = power != 0 ? (dpad ? "dpad, no limits" : "moving")
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

    private static double deadband(double value) {
        if (Math.abs(value) < DEADBAND) return 0;
        return Math.signum(value) * (Math.abs(value) - DEADBAND) / (1 - DEADBAND);
    }

    @Override
    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
        elevator.setPower(0);
    }
}
