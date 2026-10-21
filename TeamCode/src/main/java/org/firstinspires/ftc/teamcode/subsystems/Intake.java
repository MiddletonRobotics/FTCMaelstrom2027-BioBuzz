package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Locale;

public class Intake {
    private enum State { STOPPED, INTAKING, REVERSING, UNJAMMING }

    private final DcMotorEx motor;
    private State state = State.STOPPED;
    private final ElapsedTime overCurrentTimer = new ElapsedTime();
    private final ElapsedTime unjamTimer = new ElapsedTime();
    private double current = 0;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE);
        motor.setDirection(RobotConstants.INTAKE_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void in() {
        if (state == State.UNJAMMING) return;
        if (state != State.INTAKING) overCurrentTimer.reset();
        setState(State.INTAKING);
    }

    public void reverse() { setState(State.REVERSING); }
    public void stop() { setState(State.STOPPED); }

    public void update() {
        if (state == State.UNJAMMING) {
            if (unjamTimer.seconds() > RobotConstants.INTAKE_UNJAM_TIME_S) {
                setState(State.INTAKING);
                overCurrentTimer.reset();
            }
            return;
        }
        if (!RobotConstants.INTAKE_JAM_DETECTION || state != State.INTAKING) return;

        current = motor.getCurrent(CurrentUnit.AMPS);
        if (current < RobotConstants.INTAKE_JAM_AMPS) {
            overCurrentTimer.reset();
        } else if (overCurrentTimer.seconds() > RobotConstants.INTAKE_JAM_TIME_S) {
            unjamTimer.reset();
            setState(State.UNJAMMING);
        }
    }

    private void setState(State newState) {
        if (newState == state) return;
        state = newState;
        switch (state) {
            case INTAKING: motor.setPower(RobotConstants.INTAKE_POWER); break;
            case REVERSING:
            case UNJAMMING: motor.setPower(-RobotConstants.INTAKE_POWER); break;
            default: motor.setPower(0); break;
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", state + (RobotConstants.INTAKE_JAM_DETECTION
                ? String.format(Locale.ROOT, "  (%.1f A)", current) : ""));
    }
}
