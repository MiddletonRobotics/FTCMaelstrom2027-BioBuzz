package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Shooter {
    private final Flywheel flywheel;
    private boolean shooting = false;

    public Shooter(HardwareMap hardwareMap, DcMotorEx anchor, List<String> warnings) {
        DcMotorEx motor = MotorFinder.expansion(hardwareMap, RobotConstants.SHOOTER, 3, anchor, warnings);
        motor.setDirection(MotorFinder.direction(RobotConstants.SHOOTER_REVERSED));
        flywheel = new Flywheel(motor, hardwareMap.voltageSensor.iterator().next());
    }

    public void run(boolean shooting, boolean idle) {
        this.shooting = shooting;
        flywheel.setRpm(shooting ? RobotConstants.SHOOTER_TARGET_RPM : idle ? RobotConstants.SHOOTER_IDLE_RPM : 0);
        flywheel.update();
    }

    public void auto(boolean on) {
        shooting = on;
        flywheel.setRpm(on ? RobotConstants.AUTO_SHOOTER_RPM : 0);
        flywheel.update();
    }

    public boolean ready() { return shooting && flywheel.atSpeed(); }

    public void stop() { flywheel.stop(); }

    public void addTelemetry(Telemetry telemetry) {
        String state;
        if (flywheel.target() <= 0) state = "off";
        else if (!shooting) state = "IDLE";
        else state = flywheel.atSpeed() ? "IN RANGE" : "OUT OF RANGE";

        telemetry.addData("Shooter", "%.0f rpm (%.0f-%.0f)  %.0f%% power  %.1f V  %s", flywheel.rpm(),
                RobotConstants.SHOOTER_MIN_RPM, RobotConstants.SHOOTER_MAX_RPM, flywheel.power() * 100, flywheel.volts(), state);
        if (!flywheel.hasEncoder()) telemetry.addLine("! shooter encoder reads 0, using plain power");
        telemetry.addData("Shooter enc", "%d ticks = %.2f revs", flywheel.ticks(),
                flywheel.ticks() / RobotConstants.SHOOTER_TICKS_PER_REV);
    }
}
