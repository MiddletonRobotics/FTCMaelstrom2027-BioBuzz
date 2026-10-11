package org.firstinspires.ftc.teamcode.testing.tuning.pinpoint;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.DriverControls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Disabled
@TeleOp(name = "Pinpoint 3 - Test", group = "Pinpoint Tuning")
public class PinpointTest extends OpMode {
    private Drivetrain drivetrain;
    private DriverControls driverControls;
    private Localizer pinpoint;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Drivetrain(hardwareMap);
        driverControls = new DriverControls(drivetrain);
        if (PinpointTuningConfig.isConnected(hardwareMap)) {
            pinpoint = Constants.localizer(hardwareMap);
            pinpoint.setPose(Pose.zero());
            telemetry.addLine("Pinpoint ready. Keep the robot still, then press START.");
        } else {
            telemetry.addLine(PinpointTuningConfig.missingMessage());
            telemetry.addLine("You can still drive; Pinpoint readings will be missing.");
        }
    }

    @Override
    public void start() {
        if (pinpoint != null) pinpoint.setPose(Pose.zero());
    }

    @Override
    public void loop() {
        telemetry.addLine("--- Pinpoint ---");
        if (pinpoint == null) {
            telemetry.addLine(PinpointTuningConfig.missingMessage());
        } else {
            if (gamepad1.cross) pinpoint.setPose(Pose.zero());
            pinpoint.update();
            Pose pose = pinpoint.pose();
            telemetry.addData("X (in, forward)", "%.2f", pose.x());
            telemetry.addData("Y (in, left)", "%.2f", pose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.addLine("Cross = reset to 0");
        }
        telemetry.addLine();
        driverControls.update(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
