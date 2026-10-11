package org.firstinspires.ftc.teamcode.testing.tuning.pinpoint;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Locale;

@Disabled
@TeleOp(name = "Pinpoint 2 - Pod Offsets", group = "Pinpoint Tuning")
public class PinpointOffsetTuner extends OpMode {
    private Localizer pinpoint;

    @Override
    public void init() {
        if (!PinpointTuningConfig.isConnected(hardwareMap)) {
            telemetry.addLine(PinpointTuningConfig.missingMessage());
            return;
        }
        pinpoint = new PinpointLocalizer(hardwareMap, PinpointTuningConfig.withDirections(
                Constants.localizerConfig.xPodDirection.get(),
                Constants.localizerConfig.yPodDirection.get()));
        telemetry.addLine("Line the robot up square, keep it still, then press START.");
    }

    @Override
    public void start() {
        if (pinpoint != null) pinpoint.setPose(Pose.zero());
    }

    @Override
    public void loop() {
        if (pinpoint == null) {
            telemetry.addLine(PinpointTuningConfig.missingMessage());
            telemetry.update();
            return;
        }
        if (gamepad1.cross) pinpoint.setPose(Pose.zero());
        pinpoint.update();
        Pose pose = pinpoint.pose();
        double headingDeg = Math.toDegrees(pose.heading());

        telemetry.addData("Heading", "%.1f deg (target 180)", headingDeg);
        telemetry.addData("Drift", "x %.2f  y %.2f in", pose.x(), pose.y());
        telemetry.addLine();
        if (Math.abs(Math.abs(headingDeg) - 180) > 5) {
            telemetry.addLine("Turn the robot COUNTER-CLOCKWISE until heading is about 180");
        } else {
            telemetry.addLine(String.format(Locale.ROOT, "c.xPodOffset.set(%.3f);", -pose.y() / 2.0));
            telemetry.addLine(String.format(Locale.ROOT, "c.yPodOffset.set(%.3f);", -pose.x() / 2.0));
            telemetry.addLine();
            telemetry.addLine("Copy into pedro/Constants.java. Cross = reset.");
        }
        telemetry.update();
    }
}
