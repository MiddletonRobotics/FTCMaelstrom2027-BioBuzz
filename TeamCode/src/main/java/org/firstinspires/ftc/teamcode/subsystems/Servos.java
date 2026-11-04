package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Servos {
    private final Servo gate;
    private final Servo ramp;
    private boolean gateOpen = false;
    private boolean rampUp = false;
    private boolean lastGate = false;
    private boolean lastRamp = false;

    public Servos(HardwareMap hardwareMap, List<String> warnings) {
        gate = hardwareMap.tryGet(Servo.class, RobotConstants.GATE_SERVO);
        if (gate == null) warnings.add("no \"" + RobotConstants.GATE_SERVO + "\" in config, circle does nothing");
        ramp = hardwareMap.tryGet(Servo.class, RobotConstants.RAMP_SERVO);
        if (ramp == null) warnings.add("no \"" + RobotConstants.RAMP_SERVO + "\" in config, square does nothing");
        move();
    }

    public void update(boolean gateButton, boolean rampButton) {
        if (gateButton && !lastGate) gateOpen = !gateOpen;
        if (rampButton && !lastRamp) rampUp = !rampUp;
        lastGate = gateButton;
        lastRamp = rampButton;
        move();
    }

    private void move() {
        if (gate != null) gate.setPosition(gateOpen ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED);
        if (ramp != null) ramp.setPosition(rampUp ? RobotConstants.RAMP_UP : RobotConstants.RAMP_DOWN);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate", gateOpen ? "OPEN" : "closed");
        telemetry.addData("Ramp", rampUp ? "UP" : "down");
    }
}
