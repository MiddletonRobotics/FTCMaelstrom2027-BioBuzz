package org.firstinspires.ftc.teamcode.subsystems;

public final class FieldCentric {
    private static final double TURN_STICK = 0.4;
    private static final double LEARN_ANGLE = Math.toRadians(15);
    private static final double EVIDENCE_CAP = Math.toRadians(30);
    private static final double DEAD_TIME = 3.0;
    private static final double DEAD_ANGLE = Math.toRadians(5);
    private static final double MAX_DT = 0.5;

    private static int sign = 0;
    private static double evidence = 0;

    private double offset = 0;
    private double last = Double.NaN;
    private double turnTime = 0;
    private double turnAngle = 0;

    public void zero(double heading) {
        if (valid(heading)) offset = heading;
    }

    public void update(double heading, double turn, double dt) {
        if (!valid(heading)) {
            last = Double.NaN;
            return;
        }
        if (valid(last) && Math.abs(turn) >= TURN_STICK && dt > 0 && dt < MAX_DT) {
            double change = wrap(heading - last);
            turnTime += dt;
            turnAngle += Math.abs(change);
            evidence = clamp(evidence - change * Math.signum(turn), EVIDENCE_CAP);
            if (evidence >= LEARN_ANGLE) sign = 1;
            else if (evidence <= -LEARN_ANGLE) sign = -1;
        }
        last = heading;
    }

    public double angle(double heading) {
        if (sign == 0 || !valid(heading)) return 0;
        return sign * wrap(heading - offset);
    }

    public static double[] rotate(double forward, double strafe, double angle) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new double[]{forward * c - strafe * s, forward * s + strafe * c};
    }

    public boolean learned() { return sign != 0; }
    public boolean flipped() { return sign < 0; }
    public boolean dead() { return sign == 0 && turnTime >= DEAD_TIME && turnAngle < DEAD_ANGLE; }

    static void forget() {
        sign = 0;
        evidence = 0;
    }

    private static boolean valid(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
