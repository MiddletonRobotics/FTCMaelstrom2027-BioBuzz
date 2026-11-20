package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

public abstract class ShootAuto extends LinearOpMode {
    private static final double INTAKE_S = 1.0;
    private static final double LIFT_S = 0.5;
    private static final double SPIN_UP_MAX_S = 5.0;
    private static final double AT_RPM_WAIT_S = 1.0;
    private static final double SHOOT_S = 5.0;

    protected final List<String> warnings = new ArrayList<>();
    protected Drivetrain drivetrain;
    protected Intake intake;
    protected Shooter shooter;
    protected Servos servos;
    protected Elevator elevator;
    protected String step = "ready";

    protected abstract void show();

    protected void initHardware() {
        drivetrain = new Drivetrain(hardwareMap, warnings);
        intake = new Intake(hardwareMap, drivetrain.anchor(), warnings);
        shooter = new Shooter(hardwareMap, drivetrain.anchor(), warnings);
        servos = new Servos(hardwareMap, warnings);
        elevator = new Elevator(hardwareMap, drivetrain.anchor(), warnings);
        for (String warning : warnings) telemetry.addLine("! " + warning);
    }

    protected void shootPreload() {
        step = "intake in";
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < INTAKE_S) {
            drivetrain.stop();
            intake.load();
            servos.set(false, true);
            show();
        }
        intake.stop();

        step = "elevator up";
        timer.reset();
        while (opModeIsActive() && timer.seconds() < LIFT_S) {
            elevator.run(true, false);
            servos.set(false, true);
            show();
        }
        elevator.run(false, false);

        step = "spinning up";
        timer.reset();
        ElapsedTime atRpm = null;
        while (opModeIsActive() && timer.seconds() < SPIN_UP_MAX_S
                && (atRpm == null || atRpm.seconds() < AT_RPM_WAIT_S)) {
            shooter.auto(true);
            elevator.run(false, false);
            servos.set(false, true);
            if (atRpm == null && shooter.ready()) {
                atRpm = new ElapsedTime();
                step = "at rpm, waiting";
            }
            show();
        }

        step = "shooting";
        timer.reset();
        while (opModeIsActive() && timer.seconds() < SHOOT_S) {
            shooter.auto(true);
            intake.load();
            elevator.run(false, false);
            servos.set(false, true);
            show();
        }
        shooter.auto(false);
        intake.stop();

        step = "elevator down";
        timer.reset();
        while (opModeIsActive() && timer.seconds() < LIFT_S) {
            elevator.run(false, true);
            servos.set(false, true);
            show();
        }
        elevator.stop();
        servos.set(false, true);
    }

    protected void stopAll() {
        drivetrain.stop();
        intake.stop();
        shooter.stop();
        elevator.stop();
    }

    protected void showBattery() {
        telemetry.addData("Battery", "%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
    }
}
