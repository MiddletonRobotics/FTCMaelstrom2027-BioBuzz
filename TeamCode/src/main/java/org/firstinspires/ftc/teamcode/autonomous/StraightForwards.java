package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "straight forwards", group = "BioBuzz")
public class StraightForwards extends LinearOpMode {
    private static final double POWER = 0.4;
    private static final double SECONDS = 0.5;

    @Override
    public void runOpMode() {
        List<String> warnings = new ArrayList<>();
        Drivetrain drivetrain = new Drivetrain(hardwareMap, warnings);
        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addLine("straight forwards: 1.25 s at 40%");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SECONDS) drivetrain.drive(POWER, 0, 0);
        drivetrain.stop();
    }
}
