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
        intake = new VelocityMotor(motor(hardwareMap, RobotConstants.INTAKE, 0, RobotConstants.INTAKE_REVERSED, anchor, warnings),
                "intake", RobotConstants.INTAKE_MAX_TPS, warnings);
        transfer = new VelocityMotor(motor(hardwareMap, RobotConstants.FEEDER, 2, RobotConstants.FEEDER_REVERSED, anchor, warnings),
                "transfer", RobotConstants.FEEDER_MAX_TPS, warnings);
    }

    public void run(boolean hive, boolean in, boolean spitOut, boolean unjam,
                    boolean transferFull, boolean transferReverse, boolean transferIn) {
        double speed = RobotConstants.INTAKE_SPEED;
        double power = RobotConstants.TRANSFER_POWER;
        boolean toShooter = hive && transferFull && !unjam;

        if (spitOut || unjam || toShooter) intake.setSpeed(-speed);
        else if (in) intake.setSpeed(speed);
        else intake.setSpeed(0);

        if (unjam) transfer.setPower(-power);
        else if (hive && transferReverse) transfer.setPower(power);
        else if (toShooter || (hive && transferIn)) transfer.setPower(-power);
        else if (in) transfer.setPower(hive ? -power : power);
        else transfer.setPower(0);
    }

    public void load() {
        run(true, true, false, false, false, false, false);
    }

    public void feedShooter() {
        intake.setPower(1.0);
        transfer.setPower(-RobotConstants.TRANSFER_POWER);
    }

    public void stop() {
        intake.stop();
        transfer.stop();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", "%.0f tps %s", intake.tps(), intake.usingEncoder() ? "" : "(power)");
        telemetry.addData("Transfer", "%.0f tps (%.0f%% power)", transfer.tps(), RobotConstants.TRANSFER_POWER * 100);
    }

    private static DcMotorEx motor(HardwareMap hardwareMap, String name, int port, boolean reversed,
                                   DcMotorEx anchor, List<String> warnings) {
        DcMotorEx motor = MotorFinder.expansion(hardwareMap, name, port, anchor, warnings);
        motor.setDirection(MotorFinder.direction(reversed));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return motor;
    }
}
