package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Flywheel {
    private static final double DEAD_TPS = 20;
    private static final double DEAD_POWER = 0.3;
    private static final double DEAD_S = 0.6;

    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime deadTime = new ElapsedTime();
    private final ElapsedTime inBand = new ElapsedTime();
    private final ElapsedTime onTime = new ElapsedTime();

    private double kS = RobotConstants.SHOOTER_KS;
    private double kV = RobotConstants.SHOOTER_KV;
    private double kP = RobotConstants.SHOOTER_KP;
    private double kI = RobotConstants.SHOOTER_KI;
    private double kD = RobotConstants.SHOOTER_KD;
    private double boost = RobotConstants.SHOOTER_BOOST_RPM;

    private double target = 0;
    private double open = Double.NaN;
    private double rpm = 0;
    private double power = 0;
    private double volts;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private boolean encoder = true;

    public Flywheel(DcMotorEx motor, VoltageSensor battery) {
        this.motor = motor;
        this.battery = battery;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        volts = readVolts();
    }

    public void setGains(double kS, double kV, double kP, double kI, double kD) {
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        integral = 0;
    }

    public void setBoost(double rpm) { boost = rpm; }

    public void setRpm(double rpm) {
        if (target <= 0 && rpm > 0) onTime.reset();
        target = Math.max(0, rpm);
        open = Double.NaN;
    }

    public void setOpenPower(double p) {
        open = p;
        target = 0;
    }

    public void update() {
        double now = clock.seconds();
        double dt = Math.max(now - lastTime, 0.001);
        lastTime = now;
        rpm = motor.getVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60;
        if (encoder && rpm < -100) encoder = false;
        volts += 0.2 * (readVolts() - volts);

        double p;
        if (!Double.isNaN(open)) {
            p = open;
        } else if (target <= 0) {
            p = 0;
            integral = 0;
        } else {
            double error = target - rpm;
            double ff = (kS + kV * target) * RobotConstants.NOMINAL_VOLTAGE / volts;
            if (!encoder) {
                p = ff;
            } else if (boost > 0 && error > boost) {
                p = 1;
                integral = 0;
            } else {
                if (kI > 0) {
                    double cap = RobotConstants.SHOOTER_I_LIMIT / kI;
                    integral = Range.clip(integral + error * dt, -cap, cap);
                }
                p = ff + kP * error + kI * integral + kD * (error - lastError) / dt;
            }
            lastError = error;
        }

        if (encoder && p > DEAD_POWER && Math.abs(motor.getVelocity()) < DEAD_TPS) {
            if (deadTime.seconds() > DEAD_S) encoder = false;
        } else {
            deadTime.reset();
        }

        power = Range.clip(p, 0, 1);
        motor.setPower(power);
        if (Math.abs(target - rpm) > RobotConstants.SHOOTER_READY_TOLERANCE_RPM) inBand.reset();
    }

    public boolean atSpeed() {
        if (target <= 0) return false;
        if (!encoder) return onTime.seconds() > RobotConstants.LAUNCHER_SPINUP_S;
        return inBand.seconds() > RobotConstants.SHOOTER_READY_S;
    }

    public void stop() {
        target = 0;
        open = Double.NaN;
        integral = 0;
        power = 0;
        motor.setPower(0);
    }

    public double rpm() { return rpm; }
    public double target() { return target; }
    public double power() { return power; }
    public double volts() { return volts; }
    public boolean hasEncoder() { return encoder; }
    public boolean backwards() { return encoder && rpm < -100; }

    private double readVolts() {
        double v = battery.getVoltage();
        return v > 6 ? v : RobotConstants.NOMINAL_VOLTAGE;
    }
}
