package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Auto {
    private static final double UP = Math.PI / 2;
    private static final double LEFT = Math.PI;

    private static final Pose START = new Pose(61, 9, UP);
    private static final Pose SHOOT = new Pose(61, 34, UP);
    private static final Pose SIDE_FLOWER = new Pose(16, 48, LEFT);
    private static final Pose FAR_FLOWER = new Pose(48, 128, UP);
    private static final Pose PARK = new Pose(14, 93, UP);

    private static final Pose VIA_SIDE = new Pose(43, 37);
    private static final Pose VIA_FAR_1 = new Pose(10, 25);
    private static final Pose VIA_FAR_2 = new Pose(30, 110);
    private static final Pose VIA_PARK = new Pose(23, 29);

    private static final double SHOOT_TIME = 2.0;
    private static final double INTAKE_TIME = 1.0;
    private static final double PATH_TIMEOUT = 5.0;
    private static final double FAR_FLOWER_CUTOFF = 22.0;
    private static final double PARK_BY = 26.0;

    private final LinearOpMode opMode;
    private final Follower follower;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean blue = false;

    public Auto(LinearOpMode opMode) {
        this.opMode = opMode;
        follower = Constants.create(opMode.hardwareMap);
    }

    public void setBlue(boolean blue) { this.blue = blue; }
    public boolean isBlue() { return blue; }
    public Pose startPose() { return flip(START); }

    public void run() {
        matchTimer.reset();
        Pose start = flip(START), shoot = flip(SHOOT), park = flip(PARK);
        follower.setPose(start);

        followPath(start, shoot);
        shoot();

        if (hasTimeLeft()) {
            Pose sideFlower = flip(SIDE_FLOWER), viaSide = flip(VIA_SIDE);
            followPath(shoot, viaSide, sideFlower);
            intake();
            followPath(sideFlower, viaSide, shoot);
            shoot();
        }

        if (matchTimer.seconds() < FAR_FLOWER_CUTOFF) {
            Pose farFlower = flip(FAR_FLOWER);
            followPath(shoot, flip(VIA_FAR_1), flip(VIA_FAR_2), farFlower);
            intake();
            followPath(farFlower, park);
        } else {
            Pose here = follower.pose();
            if (here.distance(shoot) < 6) followPath(shoot, flip(VIA_PARK), park);
            else followPath(here, park);
        }

        follower.hold(park);
        while (opMode.opModeIsActive()) follower.update();
    }

    private void followPath(Pose... points) {
        Pose from = points[0], to = points[points.length - 1];
        Path path = points.length == 2 ? line(from, to) : curve(points);
        drive(path.linear(from, to));
    }

    private void drive(Path path) {
        if (!opMode.opModeIsActive()) return;
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT) {
            follower.update();
            opMode.telemetry.addData("Time", "%.1f s", matchTimer.seconds());
            opMode.telemetry.addData("Pose", follower.pose());
            opMode.telemetry.update();
        }
    }

    private void shoot() { holdFor(SHOOT_TIME);
    private void intake() { holdFor(INTAKE_TIME); }

    private void holdFor(double seconds) {
        follower.hold(follower.pose());
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < seconds && hasTimeLeft()) {
            follower.update();
        }
    }

    private boolean hasTimeLeft() {
        return matchTimer.seconds() < PARK_BY;
    }

    private Pose flip(Pose red) {
        if (!blue) return red;
        return new Pose(144 - red.x(), 144 - red.y(), red.heading() + Math.PI);
    }
}
