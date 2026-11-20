package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Servos {
    private final Servo gate;
    private final Servo ramp;
    private final Servo deFlower;

    private boolean hive = false;
    private boolean gateOpen = false;
    private boolean deFlowerOpen = false;
    private boolean lastGate = false;
    private boolean lastDeFlower = false;
    private boolean started = false;

    public Servos(HardwareMap hardwareMap, List<String> warnings) {
        gate = find(hardwareMap, RobotConstants.GATE_SERVO, "gate", warnings);
        ramp = find(hardwareMap, RobotConstants.RAMP_SERVO, "ramp", warnings);
        deFlower = find(hardwareMap, RobotConstants.DEFLOWER_SERVO, "deFlower", warnings);
        move();
    }

    public void set(boolean gateButton, boolean hive) {
        set(gateButton, hive, false);
    }

    public void set(boolean gateButton, boolean hive, boolean deFlowerButton) {
        if (gateButton && !lastGate) gateOpen = !gateOpen;
        if (deFlowerButton && !lastDeFlower) deFlowerOpen = !deFlowerOpen;
        lastGate = gateButton;
        lastDeFlower = deFlowerButton;
        this.hive = hive;
        started = true;
        move();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate", "%s (%.2f)", gateOpen ? "OPEN" : "closed", gatePosition());
        telemetry.addData("DeFlower", "%s (%.2f)", deFlowerOpen ? "OPEN" : "closed", deFlowerPosition());
        telemetry.addData("Ramp", hive ? "DOWN (hive)" : "UP (flower)");
    }

    private void move() {
        if (gate != null && started) gate.setPosition(gatePosition());
        if (deFlower != null && started) deFlower.setPosition(deFlowerPosition());
        if (ramp != null) ramp.setPosition(hive ? RobotConstants.RAMP_DOWN : RobotConstants.RAMP_UP);
    }

    private double gatePosition() {
        return gateOpen ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED;
    }

    private double deFlowerPosition() {
        return deFlowerOpen ? RobotConstants.DEFLOWER_OPEN : RobotConstants.DEFLOWER_CLOSED;
    }

    private static Servo find(HardwareMap hardwareMap, String name, String what, List<String> warnings) {
        Servo servo = hardwareMap.tryGet(Servo.class, name);
        if (servo == null) warnings.add("no \"" + name + "\" in config, " + what + " won't move");
        return servo;
    }
}
