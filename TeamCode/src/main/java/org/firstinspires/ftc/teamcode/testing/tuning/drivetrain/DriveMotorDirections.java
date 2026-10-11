package org.firstinspires.ftc.teamcode.testing.tuning.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.Wheel;

@TeleOp(name = "Drive Motor Directions", group = "Tuning")
public class DriveMotorDirections extends OpMode {
    private Drivetrain drivetrain;
    private final Gamepad previous = new Gamepad();

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        telemetry.addLine("Put the robot on a box so the wheels are off the ground. Press START.");
    }

    @Override
    public void loop() {
        Gamepad g = gamepad1;
        double[] power = g.square
                ? new double[]{1, 1, 1, 1}
                : new double[]{g.left_bumper ? 1 : 0, g.right_bumper ? 1 : 0, g.left_trigger, g.right_trigger};

        if (g.cross && !previous.cross && !g.square) {
            for (Wheel wheel : Wheel.values()) {
                if (power[wheel.ordinal()] > 0.5) {
                    drivetrain.setReversed(wheel, !drivetrain.isReversed(wheel));
                    g.rumble(150);
                }
            }
        }
        previous.copy(g);

        for (Wheel wheel : Wheel.values()) {
            drivetrain.setPower(wheel, power[wheel.ordinal()]);
        }

        telemetry.addLine("Hold L1/R1/L2/R2 to spin a wheel. Cross flips it.");
        telemetry.addLine("Each wheel should spin to roll the robot FORWARD.");
        telemetry.addLine();
        for (Wheel wheel : Wheel.values()) {
            telemetry.addData(wheel.name(), "%s%s",
                    drivetrain.isReversed(wheel) ? "REVERSED" : "forward",
                    power[wheel.ordinal()] > 0 ? "   <- spinning" : "");
        }
        telemetry.addLine();
        telemetry.addLine("Copy into RobotConstants:");
        telemetry.addLine("FRONT_LEFT_REVERSED  = " + drivetrain.isReversed(Wheel.FRONT_LEFT) + ";");
        telemetry.addLine("FRONT_RIGHT_REVERSED = " + drivetrain.isReversed(Wheel.FRONT_RIGHT) + ";");
        telemetry.addLine("BACK_LEFT_REVERSED   = " + drivetrain.isReversed(Wheel.BACK_LEFT) + ";");
        telemetry.addLine("BACK_RIGHT_REVERSED  = " + drivetrain.isReversed(Wheel.BACK_RIGHT) + ";");
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
