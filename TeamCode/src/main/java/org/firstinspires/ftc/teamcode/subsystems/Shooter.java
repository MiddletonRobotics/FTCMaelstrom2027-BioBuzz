package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Flywheel shooter with velocity control (needs the motor encoder cable plugged in),
 * plus an optional servo that is skipped if it isn't in the configuration.
 */
public class Shooter {
    private final DcMotorEx motor;
    /** Null if "shooterServo" isn't configured. */
    private final Servo servo;
    private boolean spinning = false;
    private boolean servoAtB = false;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.tryGet(Servo.class, RobotConstants.SHOOTER_SERVO);
        if (servo != null) servo.setPosition(RobotConstants.SHOOTER_SERVO_A);
    }

    public void spinUp() {
        spinning = true;
        motor.setVelocity(rpmToTicksPerSec(RobotConstants.SHOOTER_TARGET_RPM));
    }

    public void stop() {
        spinning = false;
        motor.setPower(0);
    }

    public void toggle() {
        if (spinning) stop(); else spinUp();
    }

    public double getRpm() {
        return motor.getVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60.0;
    }

    /** True once the flywheel is close enough to target speed to shoot. */
    public boolean isReady() {
        return spinning && Math.abs(getRpm() - RobotConstants.SHOOTER_TARGET_RPM)
                < RobotConstants.SHOOTER_READY_TOLERANCE_RPM;
    }

    public void toggleServo() {
        if (servo == null) return;
        servoAtB = !servoAtB;
        servo.setPosition(servoAtB ? RobotConstants.SHOOTER_SERVO_B : RobotConstants.SHOOTER_SERVO_A);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * RobotConstants.SHOOTER_TICKS_PER_REV / 60.0;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM", "%.0f / %.0f %s", getRpm(),
                spinning ? RobotConstants.SHOOTER_TARGET_RPM : 0.0, isReady() ? "READY" : "");
    }
}
