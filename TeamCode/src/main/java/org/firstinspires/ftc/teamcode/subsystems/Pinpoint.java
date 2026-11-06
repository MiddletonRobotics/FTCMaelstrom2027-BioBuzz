package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Pinpoint {
    private final GoBildaPinpointDriver device;

    public Pinpoint(HardwareMap hardwareMap) {
        device = hardwareMap.tryGet(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);
        if (device == null) return;
        device.setOffsets(RobotConstants.POD_X_OFFSET, RobotConstants.POD_Y_OFFSET, DistanceUnit.INCH);
        device.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        device.setEncoderDirections(direction(RobotConstants.POD_X_REVERSED), direction(RobotConstants.POD_Y_REVERSED));
        device.resetPosAndIMU();
    }

    public boolean found() { return device != null; }

    public void update() {
        if (device != null) device.update();
    }

    public void setPose(double x, double y, double headingDeg) {
        if (device != null) device.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg));
    }

    public String problem() {
        if (device == null) return "no \"" + RobotConstants.PINPOINT + "\" in config";
        DeviceStatus status = device.getDeviceStatus();
        if (status != DeviceStatus.READY) return "pinpoint " + status;
        if (device.getFrequency() < RobotConstants.MIN_PINPOINT_HZ) {
            return String.format("pinpoint slow (%.0f Hz)", device.getFrequency());
        }
        return null;
    }

    public double x() { return device == null ? 0 : device.getPosX(DistanceUnit.INCH); }
    public double y() { return device == null ? 0 : device.getPosY(DistanceUnit.INCH); }
    public double heading() { return device == null ? 0 : device.getHeading(AngleUnit.RADIANS); }

    public static EncoderDirection direction(boolean reversed) {
        return reversed ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;
    }
}
