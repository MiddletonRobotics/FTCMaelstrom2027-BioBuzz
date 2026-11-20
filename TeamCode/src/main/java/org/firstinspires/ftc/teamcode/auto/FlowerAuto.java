package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Servos;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Flower", group = "Auto")
public class FlowerAuto extends LinearOpMode {
    private static final double POWER = 0.4;
    private static final double STRAFE_RIGHT_S = 0.3;
    private static final double RAISE_S = 0.5;
    private static final double FORWARD_1_S = 1.4;
    private static final double GATE_WAIT_S = 5.0;
    private static final double FORWARD_2_S = 1.0;

    private final List<String> warnings = new ArrayList<>();
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Servos servos;
    private String step = "ready";

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap, warnings);
        elevator = new Elevator(hardwareMap, drivetrain.anchor(), warnings);
        servos = new Servos(hardwareMap, warnings);
        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addLine("strafe right 0.3 s, raise 0.5 s, forwards 1.4 s, open gate, wait 5 s, forwards 1 s");
        telemetry.update();
        waitForStart();

        drive(0, POWER, STRAFE_RIGHT_S, "strafe right");
        raise();
        drive(POWER, 0, FORWARD_1_S, "forwards");

        servos.set(true, false);
        servos.set(false, false);
        hold(GATE_WAIT_S, "gate open, waiting");
        drive(POWER, 0, FORWARD_2_S, "forwards");

        drivetrain.stop();
        step = "done";
        while (opModeIsActive()) {
            elevator.run(false, false);
            show();
            idle();
        }
    }

    private void drive(double forward, double strafe, double seconds, String name) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drivetrain.drive(forward, strafe, 0);
            elevator.run(false, false);
            show();
        }
        drivetrain.stop();
    }

    private void raise() {
        step = "raising elevator";
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < RAISE_S) {
            drivetrain.stop();
            elevator.run(true, false);
            show();
        }
        elevator.run(false, false);
    }

    private void hold(double seconds, String name) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drivetrain.stop();
            elevator.run(false, false);
            show();
        }
    }

    private void show() {
        telemetry.addData("Auto", step);
        elevator.addTelemetry(telemetry);
        servos.addTelemetry(telemetry);
        telemetry.addData("Battery", "%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.update();
    }
}
