package org.firstinspires.ftc.teamcode.testing.tuning.elevator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Disabled
@TeleOp(name = "Elevator Tuning", group = "Tuning")
public class ElevatorTuning extends OpMode {
    private static final double MAX_POWER = 0.6;
    private DcMotor elevator;
    private boolean previousCross = false;
    private int highest = 0;

    @Override
    public void init() {
        elevator = hardwareMap.get(DcMotor.class, RobotConstants.ELEVATOR);
        elevator.setDirection(RobotConstants.ELEVATOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetZero();
        telemetry.addLine("Elevator must be fully DOWN. Press START.");
    }

    @Override
    public void loop() {
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) < RobotConstants.STICK_DEADBAND) stick = 0;
        elevator.setPower(stick * MAX_POWER);

        if (gamepad1.cross && !previousCross) resetZero();
        previousCross = gamepad1.cross;

        int ticks = elevator.getCurrentPosition();
        highest = Math.max(highest, ticks);

        telemetry.addData("Elevator ticks", ticks);
        telemetry.addData("Highest seen", highest);
        if (stick > 0.3 && ticks < -20) {
            telemetry.addLine("Going UP but ticks are NEGATIVE: flip ELEVATOR_REVERSED in RobotConstants");
        }
        telemetry.addLine();
        telemetry.addLine("Write down ticks at DOWN, TRANSFER, UP, and the very top (MAX).");
        telemetry.addLine("Cross = re-zero here");
        telemetry.update();
    }

    private void resetZero() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        highest = 0;
    }

    @Override
    public void stop() {
        elevator.setPower(0);
    }
}
