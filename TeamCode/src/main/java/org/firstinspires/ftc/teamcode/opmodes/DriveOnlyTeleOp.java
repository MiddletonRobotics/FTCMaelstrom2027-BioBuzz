package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "BioBuzz Drive Only", group = "BioBuzz")
public class DriveOnlyTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private DriverControls driverControls;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Drivetrain(hardwareMap);
        driverControls = new DriverControls(drivetrain);
        telemetry.addLine("Drivetrain ready. Press START.");
    }

    @Override
    public void loop() {
        driverControls.update(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
