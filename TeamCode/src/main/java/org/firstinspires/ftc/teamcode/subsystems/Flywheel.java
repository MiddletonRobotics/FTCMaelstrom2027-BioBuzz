package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayDeque;
import java.util.Iterator;

public class Flywheel {
    private static final double DEAD_TPS = 20;
    private static final double DEAD_POWER = 0.3;
    private static final double DEAD_S = 0.6;

    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime deadTime = new ElapsedTime(   );
    private final ElapsedTime backTime = new ElapsedTime();
    private final ElapsedTime inBand = new ElapsedTime();
    private final ElapsedTime onTime = new ElapsedTime();
    private final ArrayDeque<double[]> window = new ArrayDeque<>();

    private double target = 0;
    private double rpm = 0;
    private double power = 0;
    private double volts;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double idlePower = SHOOTER_IDLE_START_POWER;
    private int ticks = 0;
    private boolean encoder = true;

    public Flywheel(DcMotorEx motor, VoltageSensor battery) {
        this.motor = motor;
        this.battery = battery;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        volts = readVolts();
    }

    public void setRpm(double rpm) {
        if (target <= SHOOTER_IDLE_RPM && rpm > SHOOTER_IDLE_RPM) onTime.reset();
        target = Math.max(0, rpm);
    }

    public void update() {
        double now = clock.seconds();
        double dt = Math.max(now - lastTime, 0.001);
        lastTime = now;
        double tps = motor.getVelocity();
        rpm = readRpm(now, tps);

        if (rpm > -100) backTime.reset();
        if (encoder && backTime.seconds() > 0.3) encoder = false;

        if (!encoder && tps > 200) encoder = true;

        boolean idle = target > 0 && target <= SHOOTER_IDLE_RPM;
        boolean bandTarget = target >= SHOOTER_MIN_RPM && target <= SHOOTER_MAX_RPM;
        volts += 0.2 * (readVolts() - volts);

        double p;
        if (target <= 0) {
            p = 0;
            integral = 0;
        } else {
            double error = target - rpm;
            double ff = (SHOOTER_KS + SHOOTER_KV * target) * NOMINAL_VOLTAGE / volts;

            if (idle && !encoder) {
                p = SHOOTER_IDLE_START_POWER;
            } else if (idle) {
                if (rpm > target * 2) {
                    p = 0;
                } else {
                    idlePower = Range.clip(idlePower + SHOOTER_IDLE_KI * error * dt, 0, SHOOTER_IDLE_MAX_POWER);
                    p = idlePower;
                }
                integral = 0;
            } else if (!encoder) {
                p = ff;
            } else if (bandTarget && rpm < SHOOTER_MIN_RPM + SHOOTER_GUARD_RPM) {
                p = 1;
            } else if (bandTarget && rpm > SHOOTER_MAX_RPM - SHOOTER_GUARD_RPM) {
                p = 0;
            } else if (SHOOTER_BOOST_RPM > 0 && error > SHOOTER_BOOST_RPM) {
                p = 1;
                integral = 0;
            } else {
                if (SHOOTER_KI > 0) {
                    double cap = SHOOTER_I_LIMIT / SHOOTER_KI;
                    integral = Range.clip(integral + error * dt, -cap, cap);
                }
                p = ff + SHOOTER_KP * error + SHOOTER_KI * integral + SHOOTER_KD * (error - lastError) / dt;
            }
            lastError = error;
        }

        if (encoder && !idle && p > DEAD_POWER && Math.abs(tps) < DEAD_TPS) {
            if (deadTime.seconds() > DEAD_S) encoder = false;
        } else {
            deadTime.reset();
        }

        power = Range.clip(p, 0, 1);
        motor.setPower(power);

        boolean inRange = bandTarget
                ? rpm >= SHOOTER_MIN_RPM && rpm <= SHOOTER_MAX_RPM
                : Math.abs(target - rpm) <= SHOOTER_READY_TOLERANCE_RPM;
        if (!inRange) inBand.reset();
    }

    public boolean atSpeed() {
        if (target <= 0) return false;
        if (!encoder) return onTime.seconds() > LAUNCHER_SPINUP_S;
        return inBand.seconds() > SHOOTER_READY_S;
    }

    public void stop() {
        target = 0;
        integral = 0;
        power = 0;
        motor.setPower(0);
    }

    public int ticks() { return ticks; }
    public double rpm() { return rpm; }
    public double target() { return target; }
    public double power() { return power; }
    public double volts() { return volts; }
    public boolean hasEncoder() { return encoder; }

    private double readRpm(double now, double tps) {
        ticks = motor.getCurrentPosition();
        window.addLast(new double[]{now, ticks});
        while (window.size() > 2) {
            Iterator<double[]> it = window.iterator();
            it.next();
            if (now - it.next()[0] < SHOOTER_AVG_WINDOW_S) break;
            window.removeFirst();
        }

        double fast = tps / SHOOTER_TICKS_PER_REV * 60;
        double[] first = window.peekFirst();
        double span = now - first[0];
        if (span < 0.05 || Math.abs(fast) > SHOOTER_LOW_RPM) return fast;
        return (ticks - first[1]) / span / SHOOTER_TICKS_PER_REV * 60;
    }

    private double readVolts() {
        double v = battery.getVoltage();
        return v > 6 ? v : NOMINAL_VOLTAGE;
    }
}
