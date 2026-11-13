package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Servos;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "flower cycle", group = "BioBuzz")
public class FlowerCycle extends LinearOpMode {
    private static final double DRIVE_POWER = 0.4;
    private static final double BACKWARD_1_S = 1.5;
    private static final double RAISE_S = 0.5;
    private static final double GATE_OPEN_S = 5.0;
    private static final double BACKWARD_2_S = 1.85;

    private Drivetrain drivetrain;
    private Elevator elevator;
    private Servos servos;
    private String step = "ready";

    @Override
    public void runOpMode() {
        List<String> warnings = new ArrayList<>();
        drivetrain = new Drivetrain(hardwareMap, warnings);
        elevator = new Elevator(hardwareMap, drivetrain.anchor(), warnings);
        servos = new Servos(hardwareMap, warnings);
        while (opModeInInit()) {
            for (String warning : warnings) telemetry.addLine("! " + warning);
            telemetry.addLine("flower cycle: backward, raise, gate open, gate close, backward");
            drivetrain.addWheelTelemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) return;

        drive(BACKWARD_1_S, "backwards");
        lift(true, false, RAISE_S, "raising elevator");

        toggleGate();
        lift(false, false, GATE_OPEN_S, "gate open, holding");
        toggleGate();

        drive(BACKWARD_2_S, "backwards");

        drivetrain.stop();
        servos.stop();
        step = "done";
        while (opModeIsActive()) show();
    }

    private void toggleGate() {
        servos.set(true, false);
        servos.set(false, false);
    }

    private void drive(double seconds, String name) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drivetrain.drive(-DRIVE_POWER, 0, 0);
            elevator.run(false, false);
            show();
        }
        drivetrain.stop();
    }

    private void lift(boolean up, boolean down, double seconds, String name) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drivetrain.stop();
            elevator.run(up, down);
            show();
        }
        elevator.run(false, false);
    }

    private void show() {
        telemetry.addData("Auto", step);
        drivetrain.addWheelTelemetry(telemetry);
        elevator.addTelemetry(telemetry);
        servos.addTelemetry(telemetry);
        telemetry.update();
    }
}
