package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Slider;

@TeleOp(name = "Elevator", group = "BioBuzz")
public class ElevatorTeleOp extends OpMode {
    private Slider slider;

    @Override
    public void init() {
        slider = new Slider(hardwareMap);
        telemetry.addLine("Elevator must be fully DOWN. Press START.");
    }

    @Override
    public void loop() {
        if (gamepad1.cross) slider.up();
        else if (gamepad1.circle) slider.down();
        else slider.hold();

        slider.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        slider.stop();
    }
}
