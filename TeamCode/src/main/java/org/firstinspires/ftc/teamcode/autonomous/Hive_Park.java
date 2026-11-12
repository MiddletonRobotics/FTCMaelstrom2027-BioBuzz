package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Hive Auto", group = "BioBuzz")
public class Hive_Park extends LinearOpMode {
    private static final double POWER = 0.4;
    private static final double FORWARD_1_S = 0.4;
    private static final double SHOOT_S = 5.0;
    private static final double STRAFE_LEFT_S = 1.0;
    private static final double FINAL_S = 2.5;
    // final leg: 1 unit left for every 10 units forward
    private static final double FINAL_FORWARD = POWER;
    private static final double FINAL_STRAFE = -POWER / 10.0;

    private final List<String> warnings = new ArrayList<>();
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Servos servos;
    private String step = "ready";

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap, warnings);
        intake = new Intake(hardwareMap, drivetrain.anchor(), warnings);
        shooter = new Shooter(hardwareMap, drivetrain.anchor(), warnings);
        servos = new Servos(hardwareMap, warnings);
        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addLine("forwards 0.4 s, shoot 5 s, strafe left 1 s, forwards 2.5 s with slight left");
        telemetry.update();
        waitForStart();

        drive(POWER, 0, FORWARD_1_S, "forwards");
        shoot();
        drive(0, -POWER, STRAFE_LEFT_S, "strafe left");
        drive(FINAL_FORWARD, FINAL_STRAFE, FINAL_S, "forwards, drifting left");

        drivetrain.stop();
        intake.stop();
        shooter.stop();
        servos.stop();
        step = "done";
        while (opModeIsActive()) {
            show();
            idle();
        }
    }

    private void drive(double forward, double strafe, double seconds, String name) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drivetrain.drive(forward, strafe, 0);
            show();
        }
        drivetrain.stop();
    }

    private void shoot() {
        step = "shooting";
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SHOOT_S) {
            drivetrain.stop();
            shooter.run(true);
            intake.run(true, false, false, false, true, false, warnings);
            servos.set(false, true, RobotConstants.SPIN_POWER);
            show();
        }
        shooter.run(false);
        intake.stop();
        servos.set(false, true, 0);
    }

    private void show() {
        telemetry.addData("Auto", step);
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        servos.addTelemetry(telemetry);
        telemetry.addData("Battery", "%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.update();
    }
}