package org.firstinspires.ftc.teamcode.testing.tuning.pinpoint;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

final class PinpointTuningConfig {
    private PinpointTuningConfig() {}

    static boolean isConnected(HardwareMap hardwareMap) {
        return hardwareMap.tryGet(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()) != null;
    }

    static String missingMessage() {
        return "No Pinpoint named \"" + Constants.localizerConfig.name.get() + "\" in the configuration.";
    }

    static PinpointConfig withDirections(GoBildaPinpointDriver.EncoderDirection xDirection,
                                         GoBildaPinpointDriver.EncoderDirection yDirection) {
        PinpointConfig saved = Constants.localizerConfig;
        return new PinpointConfig(c -> {
            c.name.set(saved.name.get());
            try {
                c.podType.set(saved.podType.get());
            } catch (IllegalStateException customPods) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(saved.ticksPerUnit.get());
            }
            c.xPodDirection.set(xDirection);
            c.yPodDirection.set(yDirection);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });
    }
}
