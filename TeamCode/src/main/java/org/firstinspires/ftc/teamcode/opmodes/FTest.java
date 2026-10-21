package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/*
 * Drive (gamepad1, same as Drive Only):
 *   left stick = drive/strafe, right stick X = turn
 *   triangle = reverse drive, share = field-centric, options = reset heading
 *
 * Mechanisms (gamepad1 OR gamepad2):
 *   right trigger = intake in, left trigger = intake out (analog)
 *   elevator = gamepad1 right stick Y, or gamepad2 left stick Y
 *     push up = go up, pull down = go down, small push = slow, max 0.5 power
 *     let go = elevator stops and holds (brake)
 *     slows near the top/bottom and stops at them
 *
 * Elevator must be at the bottom when INIT is pressed (encoder zero = bottom).
 */
@TeleOp(name = "ftest", group = "BioBuzz")
public class FTest extends OpMode {
    // Expansion hub ports, used if "intake"/"elevator" aren't in the config
    private static final int INTAKE_PORT = 0;     // 435 rpm
    private static final int ELEVATOR_PORT = 1;   // 312 rpm

    private static final double TRIGGER_DEADBAND = 0.1;
    private static final double STICK_DEADBAND = 0.08;

    private static final double ELEVATOR_POWER = 0.5;
    private static final double ELEVATOR_STALL_AMPS = 3.5; // 312 rpm stalls ~4.6 A at 0.5
    private static final double ELEVATOR_STALL_TIME_S = 0.4;
    private static final int ELEVATOR_SLOW_ZONE = 200;
    private static final double ELEVATOR_MIN_SCALE = 0.25;

    private Drivetrain drivetrain;
    private DriverControls driverControls;
    private DcMotorEx intake;
    private DcMotorEx elevator;

    private final ElapsedTime stallTimer = new ElapsedTime();
    private boolean stalled = false;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Drivetrain(hardwareMap);
        driverControls = new DriverControls(drivetrain);

        intake = findExpansionMotor(RobotConstants.INTAKE, INTAKE_PORT);
        intake.setDirection(RobotConstants.INTAKE_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        elevator = findExpansionMotor(RobotConstants.ELEVATOR, ELEVATOR_PORT);
        elevator.setDirection(RobotConstants.ELEVATOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetElevatorZero();
        elevator.setPower(0);

        telemetry.addLine("ftest ready. Elevator must be all the way DOWN. Press START.");
    }

    private DcMotorEx findExpansionMotor(String name, int port) {
        DcMotorEx byName = hardwareMap.tryGet(DcMotorEx.class, name);
        if (byName != null) return byName;
        // Same port number exists on both hubs, so skip the drive motors' hub
        DcMotorEx driveMotor = hardwareMap.tryGet(DcMotorEx.class, RobotConstants.FRONT_LEFT);
        if (driveMotor != null) {
            for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
                if (motor.getPortNumber() == port && motor.getController() != driveMotor.getController()) {
                    return motor;
                }
            }
        }
        throw new IllegalStateException("Can't find " + name + " (expansion hub port " + port
                + "). Name it \"" + name + "\" in the robot configuration.");
    }

    @Override
    public void loop() {
        driverControls.update(gamepad1, telemetry);
        updateIntake();
        updateElevator();

        telemetry.addData("Intake power", "%.2f", intake.getPower());
        telemetry.addData("Elevator", "%d / %d ticks, power %.2f%s", elevator.getCurrentPosition(),
                RobotConstants.ELEVATOR_MAX, elevator.getPower(),
                stalled ? "  STALLED, let go of stick" : "");
        telemetry.update();
    }

    private void updateIntake() {
        double in = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        double out = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        double power = in - out;
        if (Math.abs(power) < TRIGGER_DEADBAND) power = 0;
        intake.setPower(power * RobotConstants.INTAKE_POWER);
    }

    private void updateElevator() {
        double stick = Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad2.left_stick_y)
                ? gamepad1.right_stick_y : gamepad2.left_stick_y;
        if (Math.abs(stick) < STICK_DEADBAND) stick = 0;
        // Squared so small pushes move it slowly
        double power = -stick * Math.abs(stick) * ELEVATOR_POWER;

        // Soft limits: slow down near the ends, stop at them
        int position = elevator.getCurrentPosition();
        int room = power > 0 ? RobotConstants.ELEVATOR_MAX - position
                : position - RobotConstants.ELEVATOR_DOWN;
        if (room <= 0) power = 0;
        else if (room < ELEVATOR_SLOW_ZONE) {
            power *= Math.max(ELEVATOR_MIN_SCALE, (double) room / ELEVATOR_SLOW_ZONE);
        }

        if (power == 0) {
            stalled = false;
            stallTimer.reset();
        } else if (elevator.getCurrent(CurrentUnit.AMPS) < ELEVATOR_STALL_AMPS) {
            stallTimer.reset();
        } else if (!stalled && stallTimer.seconds() > ELEVATOR_STALL_TIME_S) {
            // Hit a hard stop: cut power until the stick is released
            stalled = true;
            if (power < 0) resetElevatorZero();
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }

        elevator.setPower(stalled ? 0 : power);
    }

    private void resetElevatorZero() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.setPower(0);
        elevator.setPower(0);
    }
}
