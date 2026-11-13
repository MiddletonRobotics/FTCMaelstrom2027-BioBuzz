package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Intake {
    private final VelocityMotor intake;
    private final VelocityMotor transfer;

    public Intake(HardwareMap hardwareMap, DcMotorEx anchor, List<String> warnings) {
        intake = new VelocityMotor(motor(hardwareMap, RobotConstants.INTAKE, 0,
                RobotConstants.INTAKE_REVERSED, anchor, warnings), "intake", RobotConstants.INTAKE_MAX_TPS);
        transfer = new VelocityMotor(motor(hardwareMap, RobotConstants.FEEDER, 2,
                RobotConstants.FEEDER_REVERSED, anchor, warnings), "transfer", RobotConstants.FEEDER_MAX_TPS);
    }

    public void run(boolean hive, boolean in, boolean spitOut, boolean reverse, boolean shooting, boolean transferFull, boolean transferReverse, List<String> warnings) {
        double speed = RobotConstants.INTAKE_SPEED;

        boolean toShooter = hive && transferFull && !reverse;

        double front = 0;
        if (spitOut || reverse || toShooter) front = -speed;
        else if (in) front = speed;

        intake.set(front, warnings);
        if (reverse) transfer.set(-speed, warnings);
        else if (hive && transferReverse) transfer.full(1.0);
        else if (shooting || toShooter) transfer.full(-1.0);
        else if (in) transfer.set(hive ? -speed : speed, warnings);
        else transfer.set(0, warnings);
    }

    public void stop() {
        intake.stop();
        transfer.stop();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", "%.0f tps %s", intake.tps(), intake.usingVelocity() ? "" : "(power)");
        telemetry.addData("Transfer", "%.0f tps %s", transfer.tps(), transfer.usingVelocity() ? "" : "(power)");
    }

    private static DcMotorEx motor(HardwareMap hardwareMap, String name, int port, boolean reversed,
                                   DcMotorEx anchor, List<String> warnings) {
        DcMotorEx motor = MotorFinder.expansion(hardwareMap, name, port, anchor, warnings);
        motor.setDirection(MotorFinder.direction(reversed));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return motor;
    }
}
