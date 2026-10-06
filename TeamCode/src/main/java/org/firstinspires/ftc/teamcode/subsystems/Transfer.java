package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Transfer between intake and shooter: the flower servo, the gate servo that
 * opens access to the shooter, and the feeder motor that pushes into the shooter.
 * The elevator is its own subsystem ({@link Elevator}).
 */
public class Transfer {
    private final DcMotor feeder;
    private final Servo flower;
    private final Servo gate;
    private boolean flowerOpen = false;
    private boolean gateOpen = false;

    public Transfer(HardwareMap hardwareMap) {
        feeder = hardwareMap.get(DcMotor.class, RobotConstants.FEEDER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flower = hardwareMap.get(Servo.class, RobotConstants.FLOWER_SERVO);
        gate   = hardwareMap.get(Servo.class, RobotConstants.GATE_SERVO);
        setFlowerOpen(false);
        setGateOpen(false);
    }

    public void feed()        { feeder.setPower(RobotConstants.FEEDER_POWER); }
    public void reverseFeed() { feeder.setPower(-RobotConstants.FEEDER_POWER); }
    public void stopFeed()    { feeder.setPower(0); }

    public void setFlowerOpen(boolean open) {
        flowerOpen = open;
        flower.setPosition(open ? RobotConstants.FLOWER_OPEN : RobotConstants.FLOWER_CLOSED);
    }

    public void setGateOpen(boolean open) {
        gateOpen = open;
        gate.setPosition(open ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED);
    }

    public void toggleFlower() { setFlowerOpen(!flowerOpen); }
    public void toggleGate()   { setGateOpen(!gateOpen); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower", flowerOpen ? "OPEN" : "closed");
        telemetry.addData("Gate", gateOpen ? "OPEN" : "closed");
    }
}
