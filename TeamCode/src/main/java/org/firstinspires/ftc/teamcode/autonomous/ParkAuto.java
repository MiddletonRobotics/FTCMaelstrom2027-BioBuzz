package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Disabled
@Autonomous(name = "BioBuzz Auto - Park", group = "BioBuzz")
public class ParkAuto extends LinearOpMode {
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Drivetrain(hardwareMap);

        int delaySeconds = 0;
        boolean strafeRight = true;
        boolean prevUp = false, prevDown = false;

        while (opModeInInit()) {
            if (gamepad1.dpad_up && !prevUp) delaySeconds = Math.min(25, delaySeconds + 1);
            if (gamepad1.dpad_down && !prevDown) delaySeconds = Math.max(0, delaySeconds - 1);
            if (gamepad1.dpad_left) strafeRight = false;
            if (gamepad1.dpad_right) strafeRight = true;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            telemetry.addData("Start delay (s)", delaySeconds);
            telemetry.addData("Strafe side", strafeRight ? "RIGHT" : "LEFT");
            telemetry.addData("Park target", "%.0f in forward, %.0f in %s",
                    RobotConstants.AUTO_PARK_FORWARD_IN, RobotConstants.AUTO_PARK_STRAFE_IN,
                    strafeRight ? "right" : "left");
            if (!drivetrain.hasImu()) telemetry.addLine("WARNING: no IMU, heading hold disabled");
            telemetry.update();
        }
        if (isStopRequested()) return;

        drivetrain.resetHeading();
        sleep(delaySeconds * 1000L);

        double targetY = (strafeRight ? -1 : 1) * RobotConstants.AUTO_PARK_STRAFE_IN;
        driveTo(RobotConstants.AUTO_PARK_FORWARD_IN, targetY, 5.0);

        while (opModeIsActive()) {
            drivetrain.update();
            telemetry.addLine("Parked.");
            drivetrain.addTelemetry(telemetry);
            telemetry.update();
        }
    }

    private void driveTo(double targetX, double targetY, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            drivetrain.update();
            double dx = targetX - drivetrain.odometry.getX();
            double dy = targetY - drivetrain.odometry.getY();
            double distance = Math.hypot(dx, dy);
            if (distance < RobotConstants.AUTO_TOLERANCE_IN) break;

            double heading = drivetrain.getHeading();
            double forward = dx * Math.cos(heading) + dy * Math.sin(heading);
            double left = -dx * Math.sin(heading) + dy * Math.cos(heading);

            double speed = Range.clip(distance * RobotConstants.AUTO_DRIVE_KP,
                    RobotConstants.AUTO_MIN_POWER, RobotConstants.AUTO_MAX_POWER);
            double turn = heading * RobotConstants.AUTO_HEADING_KP;
            drivetrain.drive(forward / distance * speed, -left / distance * speed, turn);

            telemetry.addData("Distance left (in)", "%.1f", distance);
            drivetrain.addTelemetry(telemetry);
            telemetry.update();
        }
        drivetrain.stop();
    }
}
