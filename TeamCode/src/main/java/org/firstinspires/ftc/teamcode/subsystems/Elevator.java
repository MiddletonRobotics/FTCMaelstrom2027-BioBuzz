package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Vertical elevator on the rear towers. Holds position with RUN_TO_POSITION.
 * The elevator must be all the way down when the OpMode is initialized (that becomes tick 0).
 */
public class Elevator {
    private final DcMotor motor;
    private int target = RobotConstants.ELEVATOR_DOWN;

    public Elevator(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConstants.ELEVATOR);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(RobotConstants.ELEVATOR_POWER);
    }

    public void goToDown()     { setTarget(RobotConstants.ELEVATOR_DOWN); }
    public void goToTransfer() { setTarget(RobotConstants.ELEVATOR_TRANSFER); }
    public void goToUp()       { setTarget(RobotConstants.ELEVATOR_UP); }

    /** Nudges the target up/down from a joystick. @param input -1 to 1, positive = up. */
    public void manual(double input) {
        setTarget((int) Math.round(target + input * RobotConstants.ELEVATOR_MANUAL_TICKS_PER_LOOP));
    }

    private void setTarget(int ticks) {
        target = Range.clip(ticks, RobotConstants.ELEVATOR_DOWN, RobotConstants.ELEVATOR_MAX);
        motor.setTargetPosition(target);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Elevator ticks", "%d (target %d)", motor.getCurrentPosition(), target);
    }
}
