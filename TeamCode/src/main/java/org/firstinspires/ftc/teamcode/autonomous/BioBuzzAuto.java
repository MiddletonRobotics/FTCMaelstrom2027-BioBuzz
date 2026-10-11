package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Autonomous(name = "BioBuzz Auto - Pedro", group = "BioBuzz")
public class BioBuzzAuto extends LinearOpMode {
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

    private static final double SHOOT_TIME_S = 2.0;
    private static final double INTAKE_TIME_S = 1.0;
    private static final double PATH_TIMEOUT_S = 5.0;
    private static final double FAR_FLOWER_CUTOFF_S = 16.0;
    private static final double PARK_BY_S = 26.0;

    private Follower follower;
    private boolean blue = false;
    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower = Constants.create(hardwareMap);

        while (opModeInInit()) {
            if (gamepad1.cross || gamepad1.square) blue = true;
            if (gamepad1.circle) blue = false;
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addLine("X / Square = blue, B / Circle = red");
            telemetry.addData("Start pose", side(START));
            telemetry.update();
        }
        if (isStopRequested()) return;
        matchTimer.reset();

        Pose start = side(START), shoot = side(SHOOT), sideFlower = side(SIDE_FLOWER),
                farFlower = side(FAR_FLOWER), park = side(PARK), viaSide = side(VIA_SIDE),
                viaFar1 = side(VIA_FAR_1), viaFar2 = side(VIA_FAR_2), viaPark = side(VIA_PARK);
        follower.setPose(start);

        drive(line(start, shoot).linear(start, shoot));
        shoot();

        if (beforePark()) {
            drive(curve(shoot, viaSide, sideFlower).linear(shoot, sideFlower));
            intake();
            drive(curve(sideFlower, viaSide, shoot).linear(sideFlower, shoot));
            shoot();
        }

        if (matchTimer.seconds() < FAR_FLOWER_CUTOFF_S) {
            drive(curve(shoot, viaFar1, viaFar2, farFlower).linear(shoot, farFlower));
            intake();
            drive(curve(farFlower, viaFar2, viaFar1, shoot).linear(farFlower, shoot));
            shoot();
        }

        Pose here = follower.pose();
        boolean atShoot = here.distance(shoot) < 6;
        drive(atShoot ? curve(shoot, viaPark, park).linear(shoot, park)
                      : line(here, park).linear(here, park));
        follower.hold(park);
        while (opModeIsActive()) {
            follower.update();
        }
    }

    private void drive(Path path) {
        if (!opModeIsActive()) return;
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_S) {
            follower.update();
            telemetry.addData("Time", "%.1f s", matchTimer.seconds());
            telemetry.addData("Pose", follower.pose());
            telemetry.update();
        }
    }

    private void shoot() {
        waitHolding(SHOOT_TIME_S);
    }

    private void intake() {
        waitHolding(INTAKE_TIME_S);
    }

    private void waitHolding(double seconds) {
        follower.hold(follower.pose());
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds && beforePark()) {
            follower.update();
        }
    }

    private boolean beforePark() {
        return matchTimer.seconds() < PARK_BY_S;
    }

    private Pose side(Pose red) {
        if (!blue) return red;
        return new Pose(144 - red.x(), 144 - red.y(), red.heading() + Math.PI);
    }
}
