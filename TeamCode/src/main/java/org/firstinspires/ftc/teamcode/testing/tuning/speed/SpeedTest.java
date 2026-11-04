package org.firstinspires.ftc.teamcode.testing.tuning.speed;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;

@TeleOp(name = "Speed Test", group = "Tuning")
public class SpeedTest extends OpMode {
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private VoltageSensor battery;
    private double intakePeak = 0;
    private double transferPeak = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE);
        transfer = hardwareMap.get(DcMotorEx.class, RobotConstants.FEEDER);
        intake.setDirection(MotorFinder.direction(RobotConstants.INTAKE_REVERSED));
        transfer.setDirection(MotorFinder.direction(RobotConstants.FEEDER_REVERSED));
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        battery = hardwareMap.voltageSensor.iterator().next();
        telemetry.addLine("Hold LT = intake full power, RT = transfer full power. Y resets peaks.");
        telemetry.addLine("Use a FULL battery. Put the peaks into INTAKE_MAX_TPS / FEEDER_MAX_TPS.");
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.left_trigger > 0.1 ? 1 : 0);
        transfer.setPower(gamepad1.right_trigger > 0.1 ? 1 : 0);
        if (gamepad1.y) {
            intakePeak = 0;
            transferPeak = 0;
        }
        intakePeak = Math.max(intakePeak, Math.abs(intake.getVelocity()));
        transferPeak = Math.max(transferPeak, Math.abs(transfer.getVelocity()));

        telemetry.addData("Battery", "%.2f V", battery.getVoltage());
        telemetry.addData("Intake", "%.0f tps  peak %.0f", intake.getVelocity(), intakePeak);
        telemetry.addData("Transfer", "%.0f tps  peak %.0f", transfer.getVelocity(), transferPeak);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);
        transfer.setPower(0);
    }
}
