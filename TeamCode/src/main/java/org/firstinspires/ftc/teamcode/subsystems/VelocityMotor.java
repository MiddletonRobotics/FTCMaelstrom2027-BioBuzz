package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class VelocityMotor {
    private static final double DEAD_TPS = 20;
    private static final double CHECK_S = 0.5;

    private final DcMotorEx motor;
    private final String name;
    private final double maxTps;
    private final ElapsedTime onTime = new ElapsedTime();
    private boolean velocity = true;
    private boolean wasOn = false;

    public VelocityMotor(DcMotorEx motor, String name, double maxTps) {
        this.motor = motor;
        this.name = name;
        this.maxTps = maxTps;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void set(double speed, List<String> warnings) {
        boolean on = speed != 0;
        if (on && !wasOn) onTime.reset();
        wasOn = on;

        if (velocity && on && onTime.seconds() > CHECK_S && Math.abs(motor.getVelocity()) < DEAD_TPS) {
            velocity = false;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            warnings.add(name + " encoder reads 0, switched to plain power");
        }

        if (velocity) motor.setVelocity(speed * maxTps);
        else motor.setPower(speed);
    }

    public void stop() { motor.setPower(0); }
    public double tps() { return motor.getVelocity(); }
    public boolean usingVelocity() { return velocity; }
}
