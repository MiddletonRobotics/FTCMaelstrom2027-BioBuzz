package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Slider {
    private enum State { UP, DOWN, HOLDING, STOPPED }

    private static final double TICKS_PER_REV = 537.7;
    private static final double UP_RPM = 25;
    private static final double DOWN_RPM = 25;
    private static final double HOLD_POWER = 0.3;
    private static final int HOLD_MIN_TICKS = 20;
    private static final boolean REVERSED = false;

    private final DcMotorEx motor;
    private State state = State.STOPPED;

    public Slider(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.ELEVATOR);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stop();
    }

    public void up() {
        state = State.UP;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(UP_RPM * TICKS_PER_REV / 60.0);
    }

    public void down() {
        state = State.DOWN;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(-DOWN_RPM * TICKS_PER_REV / 60.0);
    }

    public void hold() {
        if (state == State.HOLDING || state == State.STOPPED) return;

        int position = motor.getCurrentPosition();
        if (position < HOLD_MIN_TICKS) {
            stop();
            return;
        }
        state = State.HOLDING;
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(HOLD_POWER);
    }

    public void stop() {
        state = State.STOPPED;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Elevator state", state);
        telemetry.addData("Elevator ticks", motor.getCurrentPosition());
        telemetry.addData("Elevator RPM", "%.1f", motor.getVelocity() * 60.0 / TICKS_PER_REV);
    }
}
