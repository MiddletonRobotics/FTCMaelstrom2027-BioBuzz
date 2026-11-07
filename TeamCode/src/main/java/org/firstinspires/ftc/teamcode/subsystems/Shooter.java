package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Shooter {
    private final Flywheel flywheel;
    private boolean feeding = false;

    public Shooter(HardwareMap hardwareMap, DcMotorEx anchor, List<String> warnings) {
        DcMotorEx motor = MotorFinder.expansion(hardwareMap, RobotConstants.SHOOTER, 3, anchor, warnings);
        motor.setDirection(MotorFinder.direction(RobotConstants.SHOOTER_REVERSED));
        flywheel = new Flywheel(motor, hardwareMap.voltageSensor.iterator().next());
    }

    public double run(boolean shooting, boolean reverse) {
        flywheel.setRpm(shooting ? RobotConstants.SHOOTER_TARGET_RPM : 0);
        flywheel.update();
        feeding = shooting && flywheel.atSpeed();
        if (feeding) return RobotConstants.SPIN_POWER;
        if (!shooting && reverse) return -RobotConstants.SPIN_POWER;
        return 0;
    }

    public void stop() { flywheel.stop(); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter", "%.0f / %.0f rpm  %s", flywheel.rpm(), flywheel.target(),
                flywheel.target() <= 0 ? "off" : feeding ? "AT SPEED, feeding" : "spinning up, roller waiting");
        if (!flywheel.hasEncoder()) telemetry.addLine("! shooter encoder reads 0, using plain power");
    }
}
