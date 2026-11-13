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
    private boolean gateOpen = false;
    private boolean lastGate = false;
    private boolean gateStarted = false;
    private boolean hive = false;
    private boolean deFlowerOpen = false;
    private boolean lastDeFlower = false;

    public Servos(HardwareMap hardwareMap, List<String> warnings) {
        gate = hardwareMap.tryGet(Servo.class, RobotConstants.GATE_SERVO);
        if (gate == null) warnings.add("no \"" + RobotConstants.GATE_SERVO + "\" in config, gate won't move");
        ramp = hardwareMap.tryGet(Servo.class, RobotConstants.RAMP_SERVO);
        if (ramp == null) warnings.add("no \"" + RobotConstants.RAMP_SERVO + "\" in config, ramp won't move");
        deFlower = hardwareMap.tryGet(Servo.class, RobotConstants.DEFLOWER_SERVO);
        if (deFlower == null) warnings.add("no \"" + RobotConstants.DEFLOWER_SERVO + "\" in config, deFlower won't move");
        move();
    }

    public void set(boolean gateButton, boolean hive) {
        set(gateButton, hive, false);
    }

    public void set(boolean gateButton, boolean hive, boolean deFlowerButton) {
        if (deFlowerButton && !lastDeFlower) deFlowerOpen = !deFlowerOpen;
        lastDeFlower = deFlowerButton;
        if (gateButton && !lastGate) gateOpen = !gateOpen;
        lastGate = gateButton;
        gateStarted = true;
        this.hive = hive;
        move();
    }

    private void move() {
        if (gate != null && gateStarted) gate.setPosition(gatePosition());
        if (deFlower != null && gateStarted) deFlower.setPosition(deFlowerPosition());
        if (ramp != null) ramp.setPosition(hive ? RobotConstants.RAMP_DOWN : RobotConstants.RAMP_UP);
    }

    private double gatePosition() {
        return gateOpen ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED;
    }

    private double deFlowerPosition() {
        return deFlowerOpen ? RobotConstants.DEFLOWER_OPEN : RobotConstants.DEFLOWER_CLOSED;
    }

    public void stop() { }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate", "%s (%.2f)", gateOpen ? "OPEN" : "closed", gatePosition());
        telemetry.addData("DeFlower", "%s (%.2f)", deFlowerOpen ? "OPEN" : "closed", deFlowerPosition());
        telemetry.addData("Ramp", hive ? "DOWN (hive)" : "UP (flower)");
    }
}
