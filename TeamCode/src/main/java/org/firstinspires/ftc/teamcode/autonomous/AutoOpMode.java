package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BioBuzz Auto", group = "BioBuzz")
public class AutoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Auto auto = new Auto(this);

        while (opModeInInit()) {
            if (gamepad1.cross || gamepad1.square) auto.setBlue(true);
            if (gamepad1.circle) auto.setBlue(false);
            telemetry.addData("Alliance", auto.isBlue() ? "BLUE" : "RED");
            telemetry.addLine("X / Square = blue, B / Circle = red");
            telemetry.addData("Start pose", auto.startPose());
            telemetry.update();
        }
        if (isStopRequested()) return;

        auto.run();
    }
}
