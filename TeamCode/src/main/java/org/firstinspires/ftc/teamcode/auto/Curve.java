package org.firstinspires.ftc.teamcode.auto;

public final class Curve {
    public static final double FIELD = 144;

    private static final int SAMPLES = 300;
    private static final int WINDOW = 40;
    private static final double LOOKAHEAD = 8;
    private static final double MIN_POWER = 0.12;
    private static final double DRIVE_KP = 0.05;
    private static final double TURN_KP = 1.0;
    private static final double MAX_TURN = 0.4;
    private static final double DONE_IN = 1.5;
    private static final double DONE_RAD = Math.toRadians(6);
    private static final double CHECK_S = 1.0;
    private static final double CHECK_MIN_GAIN = 2.0;
    private static final double CHECK_FAR_IN = 6.0;
    private static final double OFF_PATH_IN = 12.0;

    private final double[] xs = new double[SAMPLES + 1];
    private final double[] ys = new double[SAMPLES + 1];
    private final double[] hs = new double[SAMPLES + 1];
    private final double[] along = new double[SAMPLES + 1];
    private final double length;
    private final double maxPower;

    private int index = 0;
    private boolean done = false;
    private boolean stuck = false;
    private double checkTime = Double.NaN;
    private double checkLeft = Double.NaN;
    private double left = Double.NaN;
    private double offPath = 0;

    public Curve(double[][] points, double startDeg, double endDeg, boolean blue, double maxPower) {
        this.maxPower = maxPower;
        double total = 0;
        for (int i = 0; i <= SAMPLES; i++) {
            double t = (double) i / SAMPLES;
            double[] p = bezier(points, t);
            double x = p[0], y = p[1];
            double h = Math.toRadians(startDeg + (endDeg - startDeg) * t);
            if (blue) {
                x = FIELD - x;
                y = FIELD - y;
                h += Math.PI;
            }
            xs[i] = x;
            ys[i] = y;
            hs[i] = wrap(h);
            if (i > 0) total += Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
            along[i] = total;
        }
        length = total;
    }

    public double[] command(double x, double y, double h, double time) {
        if (done || stuck) return new double[]{0, 0, 0};

        int last = Math.min(index + WINDOW, SAMPLES);
        double best = Double.MAX_VALUE;
        for (int i = index; i <= last; i++) {
            double d = Math.hypot(xs[i] - x, ys[i] - y);
            if (d < best) {
                best = d;
                index = i;
            }
        }

        double endDist = Math.hypot(xs[SAMPLES] - x, ys[SAMPLES] - y);
        left = Math.max(endDist, length - along[index]);
        offPath = best;
        if (best > OFF_PATH_IN) {
            stuck = true;
            return new double[]{0, 0, 0};
        }

        int target = SAMPLES;
        for (int i = index; i <= SAMPLES; i++) {
            if (along[i] >= along[index] + LOOKAHEAD) {
                target = i;
                break;
            }
        }
        boolean finalStretch = target == SAMPLES;
        double headingError = wrap(hs[target] - h);

        if (finalStretch && endDist < DONE_IN && Math.abs(headingError) < DONE_RAD) {
            done = true;
            return new double[]{0, 0, 0};
        }

        if (Double.isNaN(checkTime)) {
            checkTime = time;
            checkLeft = left;
        } else if (time - checkTime >= CHECK_S) {
            if (left > CHECK_FAR_IN && checkLeft - left < CHECK_MIN_GAIN) {
                stuck = true;
                return new double[]{0, 0, 0};
            }
            checkTime = time;
            checkLeft = left;
        }

        double ex = xs[target] - x;
        double ey = ys[target] - y;
        double distance = Math.hypot(ex, ey);
        double speed = clamp(DRIVE_KP * left, MIN_POWER, maxPower);
        if (finalStretch && endDist < DONE_IN) speed = 0;
        double vx = distance > 1e-6 ? ex / distance * speed : 0;
        double vy = distance > 1e-6 ? ey / distance * speed : 0;

        double forward = vx * Math.cos(h) + vy * Math.sin(h);
        double strafe = vx * Math.sin(h) - vy * Math.cos(h);
        double turn = clamp(-TURN_KP * headingError, -MAX_TURN, MAX_TURN);
        return new double[]{forward, strafe, turn};
    }

    public boolean done() { return done; }
    public boolean stuck() { return stuck; }
    public double left() { return left; }
    public double offPath() { return offPath; }

    static double[] bezier(double[][] points, double t) {
        double[][] p = new double[points.length][];
        for (int i = 0; i < points.length; i++) p[i] = points[i].clone();
        for (int n = points.length - 1; n > 0; n--)
            for (int i = 0; i < n; i++) {
                p[i][0] += (p[i + 1][0] - p[i][0]) * t;
                p[i][1] += (p[i + 1][1] - p[i][1]) * t;
            }
        return p[0];
    }

    static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }
}
