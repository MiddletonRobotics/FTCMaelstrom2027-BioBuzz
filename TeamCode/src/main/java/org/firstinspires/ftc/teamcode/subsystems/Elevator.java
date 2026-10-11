package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Elevator {
    private final DcMotor motor;
    private int target = RobotConstants.ELEVATOR_DOWN;
    private boolean relaxed = false;

    public Elevator(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConstants.ELEVATOR);
        motor.setDirection(RobotConstants.ELEVATOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(RobotConstants.ELEVATOR_POWER);
    }

    public void goToDown() { setTarget(RobotConstants.ELEVATOR_DOWN); }
    public void goToTransfer() { setTarget(RobotConstants.ELEVATOR_TRANSFER); }
    public void goToUp() { setTarget(RobotConstants.ELEVATOR_UP); }

    public void manual(double input) {
        if (input == 0) return;
        setTarget((int) Math.round(target + input * RobotConstants.ELEVATOR_MANUAL_TICKS_PER_LOOP));
    }

    private void setTarget(int ticks) {
        target = Range.clip(ticks, RobotConstants.ELEVATOR_DOWN, RobotConstants.ELEVATOR_MAX);
        motor.setTargetPosition(target);
    }

    public void update() {
        boolean shouldRelax = RobotConstants.ELEVATOR_RELAX_AT_BOTTOM
                && target == RobotConstants.ELEVATOR_DOWN
                && motor.getCurrentPosition() <= RobotConstants.ELEVATOR_DOWN + RobotConstants.ELEVATOR_TOLERANCE;
        if (shouldRelax != relaxed) {
            relaxed = shouldRelax;
            motor.setPower(relaxed ? 0 : RobotConstants.ELEVATOR_POWER);
        }
    }

    public void resetZero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        target = 0;
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relaxed = false;
        motor.setPower(RobotConstants.ELEVATOR_POWER);
    }

    public void addTelemetry(Telemetry telemetry) {
        int position = motor.getCurrentPosition();
        boolean atTarget = Math.abs(position - target) <= RobotConstants.ELEVATOR_TOLERANCE;
        telemetry.addData("Elevator ticks", "%d (target %d)%s%s", position, target,
                atTarget ? "  at target" : "", relaxed ? "  [relaxed]" : "");
    }
}
