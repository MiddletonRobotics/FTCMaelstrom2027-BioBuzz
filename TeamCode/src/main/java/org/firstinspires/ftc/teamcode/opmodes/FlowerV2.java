package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.VelocityMotor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "flowerv2", group = "BioBuzz")
public class FlowerV2 extends OpMode {
    private static final double DEADZONE = 0.1;

    private static final double LIFT_UP = 1.0;
    private static final double LIFT_DOWN = 0.5;
    private static final double LIFT_UP_SLOW = 0.5;
    private static final double LIFT_DOWN_SLOW = 0.3;
    private static final double LIFT_NUDGE = 0.5;
    private static final double LIFT_HOLD = 0.8;
    private static final int SLOW_ZONE = 200;
    private static final int BOTTOM = 20;

    private final List<String> warnings = new ArrayList<>();
    private List<LynxModule> hubs;
    private Buttons buttons;
    private Servos servos;

    private DcMotorEx[] wheels;
    private DcMotorEx elevator;
    private VelocityMotor intake;
    private VelocityMotor transfer;

    private boolean holding = false;
    private String liftState = "";

    @Override
    public void init() {
        buttons = new Buttons(gamepad1, gamepad2);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        boolean[] reversed = {
                RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
                RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};
        wheels = MotorFinder.drive(hardwareMap, warnings);
        for (int i = 0; i < 4; i++) {
            wheels[i].setDirection(MotorFinder.direction(reversed[i]));
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotorEx intakeMotor = expansionMotor(RobotConstants.INTAKE, 0,
                RobotConstants.INTAKE_REVERSED, DcMotor.ZeroPowerBehavior.FLOAT);
        intake = new VelocityMotor(intakeMotor, "intake", RobotConstants.INTAKE_MAX_TPS);

        elevator = expansionMotor(RobotConstants.ELEVATOR, 1,
                RobotConstants.ELEVATOR_REVERSED, DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx transferMotor = expansionMotor(RobotConstants.FEEDER, 2,
                RobotConstants.FEEDER_REVERSED, DcMotor.ZeroPowerBehavior.FLOAT);
        transfer = new VelocityMotor(transferMotor, "transfer", RobotConstants.FEEDER_MAX_TPS);

        MotorFinder.checkUnique(wheels[0], wheels[1], wheels[2], wheels[3],
                intakeMotor, elevator, transferMotor);

        servos = new Servos(hardwareMap, warnings);

        telemetry.addLine("flowerv2 ready. Elevator must be all the way DOWN. Press START.");
        for (String warning : warnings) telemetry.addLine("! " + warning);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        drive();
        intake.set(pick(buttons.intakeIn(), buttons.intakeOut()) * RobotConstants.INTAKE_SPEED, warnings);
        transfer.set(pick(buttons.transferIn(), buttons.transferOut()) * RobotConstants.FEEDER_SPEED, warnings);
        runElevator();
        servos.update(buttons.gate(), buttons.ramp());

        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addData("Intake", "%.0f tps %s", intake.tps(), intake.usingVelocity() ? "" : "(power)");
        telemetry.addData("Transfer", "%.0f tps %s", transfer.tps(), transfer.usingVelocity() ? "" : "(power)");
        telemetry.addData("Elevator", "%d / %d ticks  %s", elevator.getCurrentPosition(),
                RobotConstants.ELEVATOR_MAX, liftState);
        servos.addTelemetry(telemetry);
        telemetry.update();
    }

    private void drive() {
        double forward = deadzone(buttons.forward());
        double strafe = deadzone(buttons.strafe()) * RobotConstants.STRAFE_CORRECTION;
        double turn = deadzone(buttons.turn()) * RobotConstants.TURN_SCALE;
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        wheels[0].setPower((forward + strafe + turn) / max);
        wheels[1].setPower((forward - strafe - turn) / max);
        wheels[2].setPower((forward - strafe + turn) / max);
        wheels[3].setPower((forward + strafe - turn) / max);
    }

    private void runElevator() {
        int position = elevator.getCurrentPosition();
        int nudge = pick(buttons.elevatorNudgeUp(), buttons.elevatorNudgeDown());
        int move = pick(buttons.elevatorUp(), buttons.elevatorDown());
        double power = nudge != 0 ? nudge * LIFT_NUDGE : liftPower(move, position);

        if (power != 0) {
            stopHolding();
            elevator.setPower(power);
            liftState = nudge != 0 ? "dpad, no limits" : "moving";
        } else if (position > BOTTOM) {
            if (!holding) {
                holding = true;
                elevator.setTargetPosition(position);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(LIFT_HOLD);
            }
            liftState = "holding";
        } else {
            stopHolding();
            elevator.setPower(0);
            liftState = "at bottom";
        }
    }

    private static double liftPower(int move, int position) {
        if (move == 0) return 0;
        int room = move > 0 ? RobotConstants.ELEVATOR_MAX - position : position - RobotConstants.ELEVATOR_DOWN;
        if (room <= 0) return 0;
        boolean slow = room < SLOW_ZONE;
        if (move > 0) return slow ? LIFT_UP_SLOW : LIFT_UP;
        return slow ? -LIFT_DOWN_SLOW : -LIFT_DOWN;
    }

    private void stopHolding() {
        if (!holding) return;
        holding = false;
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private DcMotorEx expansionMotor(String name, int port, boolean reversed, DcMotor.ZeroPowerBehavior zero) {
        DcMotorEx motor = MotorFinder.expansion(hardwareMap, name, port, wheels[0], warnings);
        motor.setDirection(MotorFinder.direction(reversed));
        motor.setZeroPowerBehavior(zero);
        return motor;
    }

    private static int pick(boolean forward, boolean back) {
        return forward == back ? 0 : forward ? 1 : -1;
    }

    private static double deadzone(double value) {
        if (Math.abs(value) < DEADZONE) return 0;
        return Math.signum(value) * (Math.abs(value) - DEADZONE) / (1 - DEADZONE);
    }

    @Override
    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
        intake.stop();
        transfer.stop();
        elevator.setPower(0);
    }
}
