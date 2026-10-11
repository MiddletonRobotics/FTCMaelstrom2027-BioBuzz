package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "AprilTag Test", group = "Testing")
public class AprilTagTest extends OpMode {
    private Vision vision;
    private boolean blue = false;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.square) blue = true;
        if (gamepad1.circle) blue = false;

        telemetry.addData("Alliance", blue ? "BLUE (tags 38-45)" : "RED (tags 30-37)");
        vision.addTelemetry(telemetry, blue);
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.close();
    }
}
