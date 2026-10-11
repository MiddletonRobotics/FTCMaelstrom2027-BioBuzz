package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Odometry {
    private static final double INCHES_PER_TICK =
            Math.PI * RobotConstants.WHEEL_DIAMETER_IN * RobotConstants.DRIVE_GEAR_RATIO
                    / RobotConstants.DRIVE_TICKS_PER_REV;

    private double x, y;
    private int lastFl, lastFr, lastBl, lastBr;
    private boolean initialized = false;

    public void update(int fl, int fr, int bl, int br, double heading) {
        if (initialized) {
            double dFl = (fl - lastFl) * INCHES_PER_TICK;
            double dFr = (fr - lastFr) * INCHES_PER_TICK;
            double dBl = (bl - lastBl) * INCHES_PER_TICK;
            double dBr = (br - lastBr) * INCHES_PER_TICK;

            double forward = (dFl + dFr + dBl + dBr) / 4.0;
            double left = -(dFl - dFr - dBl + dBr) / 4.0 * RobotConstants.ODOMETRY_STRAFE_MULTIPLIER;

            x += forward * Math.cos(heading) - left * Math.sin(heading);
            y += forward * Math.sin(heading) + left * Math.cos(heading);
        }
        lastFl = fl;
        lastFr = fr;
        lastBl = bl;
        lastBr = br;
        initialized = true;
    }

    public double getX() { return x; }
    public double getY() { return y; }
}
