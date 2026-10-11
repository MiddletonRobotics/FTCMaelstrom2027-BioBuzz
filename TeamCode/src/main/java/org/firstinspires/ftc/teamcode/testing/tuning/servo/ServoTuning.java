package org.firstinspires.ftc.teamcode.testing.tuning.servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@Disabled
@TeleOp(name = "Servo Tuning", group = "Tuning")
public class ServoTuning extends OpMode {
    private static final double[] STEPS = {0.1, 0.01, 0.001};

    private final List<String> names = new ArrayList<>();
    private final List<Servo> servos = new ArrayList<>();
    private final List<Double> positions = new ArrayList<>();
    private int selected = 0;
    private int stepIndex = 1;
    private final Gamepad previous = new Gamepad();

    @Override
    public void init() {
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            names.add(entry.getKey());
            servos.add(entry.getValue());
            positions.add(Double.NaN);
        }
    }

    @Override
    public void loop() {
        if (servos.isEmpty()) {
            telemetry.addLine("No servos in the configuration.");
            telemetry.update();
            return;
        }
        Gamepad g = gamepad1;
        if (g.square && !previous.square) selected = (selected + 1) % servos.size();
        if (g.cross && !previous.cross) stepIndex = (stepIndex + 1) % STEPS.length;

        int direction = 0;
        if (g.dpad_right && !previous.dpad_right) direction = 1;
        if (g.dpad_left && !previous.dpad_left) direction = -1;
        previous.copy(g);

        if (direction != 0) {
            double current = positions.get(selected);
            double next = Double.isNaN(current) ? 0.5 : Range.clip(current + direction * STEPS[stepIndex], 0, 1);
            positions.set(selected, next);
            servos.get(selected).setPosition(next);
        }

        for (int i = 0; i < servos.size(); i++) {
            double p = positions.get(i);
            telemetry.addData((i == selected ? "> " : "  ") + names.get(i),
                    Double.isNaN(p) ? "(not moved)" : String.format(Locale.ROOT, "%.3f", p));
        }
        telemetry.addData("Step", STEPS[stepIndex]);
        telemetry.update();
    }
}
