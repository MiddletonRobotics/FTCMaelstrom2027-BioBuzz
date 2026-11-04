package org.firstinspires.ftc.teamcode.testing.tuning.servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

@TeleOp(name = "Gate Test", group = "Tuning")
public class GateTest extends OpMode {
    private Servo gate;
    private boolean open = false;

    @Override
    public void init() {
        gate = hardwareMap.tryGet(Servo.class, RobotConstants.GATE_SERVO);
        telemetry.addLine(gate == null
                ? "! no \"" + RobotConstants.GATE_SERVO + "\" in config"
                : "Gate Test ready. X = open, O = close");
    }

    @Override
    public void loop() {
        if (gate == null) {
            telemetry.addLine("! no \"" + RobotConstants.GATE_SERVO + "\" in config");
            telemetry.update();
            return;
        }
        if (gamepad1.cross) open = true;
        if (gamepad1.circle) open = false;
        double position = open ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED;
        gate.setPosition(position);

        telemetry.addData("Gate", open ? "OPEN" : "closed");
        telemetry.addData("Position", "%.2f", position);
        telemetry.addLine("X = open   O = close");
        telemetry.update();
    }
}
