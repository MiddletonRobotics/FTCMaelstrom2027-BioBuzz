package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Shooter {
    private static final double MAX_RPM = 6000;

    private final DcMotorEx motor;
    private final Servo servo;
    private boolean spinning = false;
    private boolean servoAtB = false;
    private double targetRpm = RobotConstants.SHOOTER_TARGET_RPM;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER);
        motor.setDirection(RobotConstants.SHOOTER_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                RobotConstants.SHOOTER_P, RobotConstants.SHOOTER_I,
                RobotConstants.SHOOTER_D, RobotConstants.SHOOTER_F));

        servo = hardwareMap.tryGet(Servo.class, RobotConstants.SHOOTER_SERVO);
        if (servo != null) servo.setPosition(RobotConstants.SHOOTER_SERVO_A);
    }

    public void spinUp() {
        spinning = true;
        motor.setVelocity(targetRpm * RobotConstants.SHOOTER_TICKS_PER_REV / 60.0);
    }

    public void stop() {
        spinning = false;
        motor.setPower(0);
    }

    public void toggle() {
        if (spinning) stop();
        else spinUp();
    }

    public void adjustTargetRpm(double deltaRpm) {
        targetRpm = Range.clip(targetRpm + deltaRpm, 0, MAX_RPM);
        if (spinning) spinUp();
    }

    public double getRpm() {
        return motor.getVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60.0;
    }

    public boolean isReady() {
        return spinning && Math.abs(getRpm() - targetRpm) < RobotConstants.SHOOTER_READY_TOLERANCE_RPM;
    }

    public void toggleServo() {
        if (servo == null) return;
        servoAtB = !servoAtB;
        servo.setPosition(servoAtB ? RobotConstants.SHOOTER_SERVO_B : RobotConstants.SHOOTER_SERVO_A);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM", "%.0f / %.0f %s", getRpm(),
                spinning ? targetRpm : 0.0, isReady() ? "READY" : "");
        if (!spinning) telemetry.addData("Shooter set to", "%.0f RPM", targetRpm);
    }
}
