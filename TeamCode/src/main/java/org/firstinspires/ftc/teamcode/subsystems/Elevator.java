package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Elevator {
    private static final double UP = 1.0;
    private static final double DOWN = 0.5;
    private static final double HOLD = 0.1;

    private final DcMotorEx motor;

    public Elevator(HardwareMap hardwareMap, DcMotorEx anchor, List<String> warnings) {
        motor = MotorFinder.expansion(hardwareMap, RobotConstants.ELEVATOR, 1, anchor, warnings);
        motor.setDirection(MotorFinder.direction(RobotConstants.ELEVATOR_REVERSED));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run(boolean up, boolean down) {
        if (up && !down) motor.setPower(UP);
        else if (down && !up) motor.setPower(-DOWN);
        else motor.setPower(HOLD);
    }

    public void stop() { motor.setPower(0); }

    public int position() { return motor.getCurrentPosition(); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Elevator", "%d ticks  power %.1f", motor.getCurrentPosition(), motor.getPower());
    }
}
