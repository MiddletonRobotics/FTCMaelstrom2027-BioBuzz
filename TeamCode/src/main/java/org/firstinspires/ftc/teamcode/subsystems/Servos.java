package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Servos {
    private final Servo gate;
    private final Servo ramp;
    private final CRServo spin;
    private boolean gateOpen = false;
    private boolean lastGate = false;
    private boolean gateStarted = false;
    private boolean rampUp = false;
    private double spinPower = 0;

    public Servos(HardwareMap hardwareMap, List<String> warnings) {
        gate = hardwareMap.tryGet(Servo.class, RobotConstants.GATE_SERVO);
        if (gate == null) warnings.add("no \"" + RobotConstants.GATE_SERVO + "\" in config, gate won't move");
        ramp = hardwareMap.tryGet(Servo.class, RobotConstants.RAMP_SERVO);
        if (ramp == null) warnings.add("no \"" + RobotConstants.RAMP_SERVO + "\" in config, ramp won't move");
        spin = hardwareMap.tryGet(CRServo.class, RobotConstants.SPIN_SERVO);
        if (spin == null) warnings.add("no \"" + RobotConstants.SPIN_SERVO + "\" (continuous) in config, roller won't spin");
        else spin.setDirection(MotorFinder.direction(RobotConstants.SPIN_REVERSED));
        move();
    }

    public void set(boolean gateButton, boolean rampUp, double roller) {
        if (gateButton && !lastGate) gateOpen = !gateOpen;
        lastGate = gateButton;
        gateStarted = true;
        this.rampUp = rampUp;
        spinPower = roller;
        move();
    }

    private void move() {
        if (gate != null && gateStarted) gate.setPosition(gatePosition());
        if (ramp != null) ramp.setPosition(rampUp ? RobotConstants.RAMP_UP : RobotConstants.RAMP_DOWN);
        if (spin != null) spin.setPower(spinPower);
    }

    private double gatePosition() {
        return gateOpen ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED;
    }

    public void stop() {
        if (spin != null) spin.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate", "%s (%.2f)", gateOpen ? "OPEN" : "closed", gatePosition());
        telemetry.addData("Ramp", rampUp ? "UP" : "down");
        telemetry.addData("Roller", spinPower > 0 ? "FEEDING" : spinPower < 0 ? "REVERSE" : "stopped");
    }
}
