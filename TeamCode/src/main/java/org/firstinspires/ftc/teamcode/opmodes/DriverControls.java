package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Gamepad 1 driving, shared by every TeleOp so the driver gets the same controls everywhere.
 *   Left stick      drive + strafe
 *   Right stick X   turn
 *   Share / Back    toggle field-centric / robot-centric
 *   Options / Start reset heading (point robot away from you first)
 */
public class DriverControls {
    private final Drivetrain drivetrain;
    private final Gamepad previous = new Gamepad();
    private boolean fieldCentric = false;

    public DriverControls(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void update(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.share && !previous.share && drivetrain.hasImu()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad.options && !previous.options) {
            drivetrain.resetHeading();
        }
        previous.copy(gamepad);

        double forward = deadband(-gamepad.left_stick_y);
        double strafe  = deadband(gamepad.left_stick_x);
        double turn    = deadband(gamepad.right_stick_x);

        if (fieldCentric) {
            drivetrain.driveFieldCentric(forward, strafe, turn);
        } else {
            drivetrain.drive(forward, strafe, turn);
        }
        drivetrain.update();

        telemetry.addData("Drive mode", fieldCentric ? "FIELD-centric" : "Robot-centric");
        if (!drivetrain.hasImu()) {
            telemetry.addLine("WARNING: no IMU named \"" + RobotConstants.IMU
                    + "\" in config. Field-centric disabled.");
        }
        drivetrain.addTelemetry(telemetry);
    }

    private static double deadband(double value) {
        return Math.abs(value) < RobotConstants.STICK_DEADBAND ? 0 : value;
    }
}
