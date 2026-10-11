package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Transfer {
    private enum FeedState { STOPPED, FEEDING, REVERSING }

    private final DcMotor feeder;
    private final Servo flower;
    private final Servo gate;
    private FeedState feedState = FeedState.STOPPED;
    private boolean flowerOpen = false;
    private boolean gateOpen = false;

    public Transfer(HardwareMap hardwareMap) {
        feeder = hardwareMap.get(DcMotor.class, RobotConstants.FEEDER);
        feeder.setDirection(RobotConstants.FEEDER_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flower = hardwareMap.get(Servo.class, RobotConstants.FLOWER_SERVO);
        gate = hardwareMap.get(Servo.class, RobotConstants.GATE_SERVO);
        setFlowerOpen(false);
        setGateOpen(false);
    }

    public void feed() { setFeedState(FeedState.FEEDING); }
    public void reverseFeed() { setFeedState(FeedState.REVERSING); }
    public void stopFeed() { setFeedState(FeedState.STOPPED); }

    private void setFeedState(FeedState newState) {
        if (newState == feedState) return;
        feedState = newState;
        switch (feedState) {
            case FEEDING: feeder.setPower(RobotConstants.FEEDER_POWER); break;
            case REVERSING: feeder.setPower(-RobotConstants.FEEDER_POWER); break;
            default: feeder.setPower(0); break;
        }
    }

    private void setFlowerOpen(boolean open) {
        flowerOpen = open;
        flower.setPosition(open ? RobotConstants.FLOWER_OPEN : RobotConstants.FLOWER_CLOSED);
    }

    private void setGateOpen(boolean open) {
        gateOpen = open;
        gate.setPosition(open ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED);
    }

    public void toggleFlower() { setFlowerOpen(!flowerOpen); }
    public void toggleGate() { setGateOpen(!gateOpen); }

    public void reset() {
        stopFeed();
        setFlowerOpen(false);
        setGateOpen(false);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Feeder", feedState);
        telemetry.addData("Flower", flowerOpen ? "OPEN" : "closed");
        telemetry.addData("Gate", gateOpen ? "OPEN" : "closed");
    }
}
