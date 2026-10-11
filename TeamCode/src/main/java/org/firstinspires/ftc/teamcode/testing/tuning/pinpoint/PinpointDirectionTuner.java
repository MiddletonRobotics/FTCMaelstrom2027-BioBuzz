package org.firstinspires.ftc.teamcode.testing.tuning.pinpoint;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Pinpoint 1 - Pod Directions", group = "Pinpoint Tuning")
public class PinpointDirectionTuner extends OpMode {
    private static final double MIN_PUSH_IN = 3.0;
    private Localizer pinpoint;

    @Override
    public void init() {
        if (!PinpointTuningConfig.isConnected(hardwareMap)) {
            telemetry.addLine(PinpointTuningConfig.missingMessage());
            return;
        }
        pinpoint = new PinpointLocalizer(hardwareMap,
                PinpointTuningConfig.withDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD));
        telemetry.addLine("Keep the robot still, then press START.");
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

        telemetry.addData("X (forward pod)", "%.2f in", pose.x());
        telemetry.addData("Y (strafe pod)", "%.2f in", pose.y());
        telemetry.addLine();
        showDirection(pose.x(), "x", "1) Push the robot FORWARD about 12 in");
        showDirection(pose.y(), "y", "2) Push the robot LEFT about 12 in");
        telemetry.addLine();
        telemetry.addLine("Copy into pedro/Constants.java. Cross = reset.");
        telemetry.update();
    }

    private void showDirection(double reading, String pod, String instruction) {
        if (Math.abs(reading) < MIN_PUSH_IN) {
            telemetry.addLine(instruction);
        } else {
            EncoderDirection direction = reading < 0 ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;
            telemetry.addLine("c." + pod + "PodDirection.set(" + PinpointTuningConfig.name(direction) + ");");
        }
    }
}
