package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Tracks the robot's position on the field using the four drive motor encoders
 * for distance and the IMU for heading.
 *
 * Field frame: +x = forward from the starting pose, +y = left, heading CCW positive.
 * Mecanum wheels slip, so this is good for basic autonomous paths. For accurate
 * autos, add dead-wheel pods (or a goBILDA Pinpoint) and swap this class out.
 */
public class Odometry {
    private static final double INCHES_PER_TICK =
            Math.PI * RobotConstants.WHEEL_DIAMETER_IN * RobotConstants.DRIVE_GEAR_RATIO
                    / RobotConstants.DRIVE_TICKS_PER_REV;

    private double x, y;
    private int lastFl, lastFr, lastBl, lastBr;
    private boolean initialized = false;

    public void update(int fl, int fr, int bl, int br, double heading) {
        if (!initialized) {
            lastFl = fl; lastFr = fr; lastBl = bl; lastBr = br;
            initialized = true;
            return;
        }
        double dFl = (fl - lastFl) * INCHES_PER_TICK;
        double dFr = (fr - lastFr) * INCHES_PER_TICK;
        double dBl = (bl - lastBl) * INCHES_PER_TICK;
        double dBr = (br - lastBr) * INCHES_PER_TICK;
        lastFl = fl; lastFr = fr; lastBl = bl; lastBr = br;

        // Same wheel mixing as Drivetrain.drive(), run backwards.
        double forward = (dFl + dFr + dBl + dBr) / 4.0;
        double strafeRight = (dFl - dFr - dBl + dBr) / 4.0 * RobotConstants.ODOMETRY_STRAFE_MULTIPLIER;
        double left = -strafeRight;

        x += forward * Math.cos(heading) - left * Math.sin(heading);
        y += forward * Math.sin(heading) + left * Math.cos(heading);
    }

    public void setPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() { return x; }
    public double getY() { return y; }
}
