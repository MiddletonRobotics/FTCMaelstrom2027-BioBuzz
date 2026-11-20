package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Just Shoot", group = "Auto")
public class JustShoot extends ShootAuto {
    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addLine("shoot the preload, no driving");
        telemetry.update();
        waitForStart();

        shootPreload();
        stopAll();

        step = "done";
        while (opModeIsActive()) {
            show();
            idle();
        }
    }

    @Override
    protected void show() {
        telemetry.addData("Auto", step);
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        servos.addTelemetry(telemetry);
        elevator.addTelemetry(telemetry);
        showBattery();
        telemetry.update();
    }
}
