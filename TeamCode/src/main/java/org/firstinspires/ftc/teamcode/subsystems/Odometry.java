package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Odometry {
    private final Localizer pinpoint;
    private Pose pose = Pose.zero();

    public Odometry(HardwareMap hardwareMap) {
        boolean found = hardwareMap.tryGet(GoBildaPinpointDriver.class, RobotConstants.PINPOINT) != null;
        pinpoint = found ? Constants.localizer(hardwareMap) : null;
        reset();
    }

    public boolean isConnected() {
        return pinpoint != null;
    }

    public void update() {
        if (pinpoint == null) return;
        pinpoint.update();
        pose = pinpoint.pose();
    }

    public void reset() {
        pose = Pose.zero();
        if (pinpoint != null) pinpoint.setPose(pose);
    }

    public double getX() { return pose.x(); }
    public double getY() { return pose.y(); }
    public double getHeading() { return pose.heading(); }

    public void addTelemetry(Telemetry telemetry) {
        if (pinpoint == null) {
            telemetry.addLine("! no \"" + RobotConstants.PINPOINT + "\" in config, no odometry");
            return;
        }
        telemetry.addData("Position", "x %.1f in  y %.1f in  %.0f deg",
                getX(), getY(), Math.toDegrees(getHeading()));
    }
}
