package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Servos {
    private static final double SPIN_OFF_S = 0.5;

    private final Servo gate;
    private final Servo ramp;
    private final CRServo spin;
    private boolean gateOpen = false;
    private boolean lastGate = false;
    private boolean gateStarted = false;
    private boolean hive = false;
    private double spinPower = 0;
    private boolean spun = false;
    private final ElapsedTime spinIdle = new ElapsedTime();

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

    public void set(boolean gateButton, boolean hive, double roller) {
        if (gateButton && !lastGate) gateOpen = !gateOpen;
        lastGate = gateButton;
        gateStarted = true;
        this.hive = hive;
        spinPower = roller;
        move();
    }

    private void move() {
        if (gate != null && gateStarted) gate.setPosition(gatePosition());
        if (ramp != null) ramp.setPosition(hive ? RobotConstants.RAMP_DOWN : RobotConstants.RAMP_UP);
        if (spin != null) spinTo(spinPower);
    }

    private void spinTo(double power) {
        if (power != 0) {
            spun = true;
            spinIdle.reset();
        }
        boolean off = power == 0 && (!spun || spinIdle.seconds() > SPIN_OFF_S);
        if (spin instanceof CRServoImplEx) {
            CRServoImplEx ex = (CRServoImplEx) spin;
            if (off) {
                spun = false;
                ex.setPwmDisable();
                return;
            }
            ex.setPwmEnable();
        }
        spin.setPower(power);
    }

    private double gatePosition() {
        return gateOpen ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED;
    }

    public void stop() {
        spun = false;
        if (spin != null) spinTo(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate", "%s (%.2f)", gateOpen ? "OPEN" : "closed", gatePosition());
        telemetry.addData("Ramp", hive ? "DOWN (hive)" : "UP (flower)");
        telemetry.addData("Roller", "%s (%.1f)", spinPower > 0 ? "FEEDING" : spinPower < 0 ? "REVERSE" : "stopped", spinPower);
    }
}
