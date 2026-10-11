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

/**
 * BIOBUZZ autonomous (Pedro Pathing). Every shot is taken from ONE spot just below our HIVE.
 *
 *   1. Start on the audience wall straight below the shooting spot, drive up, launch 4 pre-loads.
 *   2. Side-wall FLOWER (4 POLLEN) -> shooting spot -> launch.
 *   3. Far-wall FLOWER (4 POLLEN) -> shooting spot -> launch (skipped if short on time).
 *   4. PARK with only a corner in the LOADING ZONE, so our partner can park too.
 *
 * Rules this follows (BIOBUZZ manual):
 *   G304  start touching a wall, fully on our side (red = columns A-C, x < 72), not touching a
 *         FLOWER, not in the LOADING ZONE.  (Figure 11-1 shows this exact kind of start.)
 *   G402  stay on our side during AUTO: every pose keeps the robot at x < 72.
 *   G407  never more than 4 POLLEN: one FLOWER per trip, launch before the next.
 *   G417  never touch the HIVE: paths curve around the frame (about x 47-97, y 47-97).
 *   G418  take POLLEN only from the bottom of a FLOWER.
 *
 * Coordinates: Pedro field, inches, origin in the corner by the audience and the red ALLIANCE wall.
 * +x points from the red wall toward the blue wall, +y points away from the audience.
 * Poses are for RED and assume an 18 x 18 in robot. Blue is rotated 180 degrees about the
 * field center, which is how the BIOBUZZ field is laid out.
 *
 * During INIT on gamepad 1: X / Square = blue, B / Circle = red.
 *
 * shoot() and intake() only wait for now. Fill them in once the mechanisms are on the robot.
 */
@Disabled // replaced by AutoOpMode ("BioBuzz Auto"), which uses autonomous/Auto
@Autonomous(name = "BioBuzz Auto - Pedro", group = "BioBuzz")
public class BioBuzzAuto extends LinearOpMode {
    private static final double UP = Math.PI / 2;     // facing away from the audience
    private static final double LEFT = Math.PI;       // facing the red ALLIANCE wall

    // ---------------- Poses (red side) ----------------
    // Back on the audience wall, straight below the shooting spot. Robot covers x 52-70.
    private static final Pose START       = new Pose(61, 9, UP);
    // The one shooting spot, just below the red HIVE (from the field drawing).
    private static final Pose SHOOT       = new Pose(61, 34, UP);
    // Facing the FLOWER on the red wall at (0, 48).
    private static final Pose SIDE_FLOWER = new Pose(16, 48, LEFT);
    // Facing the FLOWER on the far wall at (48, 144).
    private static final Pose FAR_FLOWER  = new Pose(48, 128, UP);
    // LOADING ZONE is x 0-11, y 96-120. Robot covers x 5-23, y 84-102: a 6 x 6 in corner is in,
    // the other 18 in of the zone stays free for our partner.
    private static final Pose PARK        = new Pose(14, 93, UP);

    // Curve control points: the shortest curves that keep 2.4+ in between the robot and the
    // HIVE frame base (x 47-97, y 52.5-91.5, manual 9.6.1) and keep the robot on our side.
    private static final Pose VIA_SIDE    = new Pose(43, 37);
    private static final Pose VIA_FAR_1   = new Pose(10, 25);
    private static final Pose VIA_FAR_2   = new Pose(30, 110);
    private static final Pose VIA_PARK    = new Pose(23, 29);

    // ---------------- Timing ----------------
    private static final double SHOOT_TIME_S   = 2.0;  // launch 4 POLLEN
    private static final double INTAKE_TIME_S  = 1.0;  // pull POLLEN from a FLOWER
    private static final double PATH_TIMEOUT_S = 5.0;  // give up on a path that gets stuck
    private static final double FAR_FLOWER_CUTOFF_S = 16.0; // skip far FLOWER if later than this
    private static final double PARK_BY_S      = 26.0; // always leave time to park

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
            if (gamepad1.cross || gamepad1.square) blue = true;   // X button on Logitech/Xbox = square
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

        // 1. Pre-loads (25 in).
        drive(line(start, shoot).linear(start, shoot));
        shoot();

        // 2. Side-wall FLOWER (47 in each way).
        if (beforePark()) {
            drive(curve(shoot, viaSide, sideFlower).linear(shoot, sideFlower));
            intake();
            drive(curve(sideFlower, viaSide, shoot).linear(sideFlower, shoot));
            shoot();
        }

        // 3. Far-wall FLOWER (120 in each way, around the HIVE on the red-wall side).
        if (matchTimer.seconds() < FAR_FLOWER_CUTOFF_S) {
            drive(curve(shoot, viaFar1, viaFar2, farFlower).linear(shoot, farFlower));
            intake();
            drive(curve(farFlower, viaFar2, viaFar1, shoot).linear(farFlower, shoot));
            shoot();
        }

        // 4. Partial PARK (85 in). If we bailed out early, go straight from wherever we are.
        Pose here = follower.pose();
        boolean atShoot = here.distance(shoot) < 6;
        drive(atShoot ? curve(shoot, viaPark, park).linear(shoot, park)
                      : line(here, park).linear(here, park));
        follower.hold(park);
        while (opModeIsActive()) {
            follower.update();
        }
    }

    /** Follow a path until it finishes, times out, or the OpMode stops. */
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

    /** Hold position while launching. TODO: spin up the shooter and run the transfer here. */
    private void shoot() {
        waitHolding(SHOOT_TIME_S);
    }

    /** Hold position while pulling POLLEN from a FLOWER. TODO: run the intake here. */
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

    /** Red pose as-is, or rotated 180 degrees about the field center for blue. */
    private Pose side(Pose red) {
        if (!blue) return red;
        return new Pose(144 - red.x(), 144 - red.y(), red.heading() + Math.PI);
    }
}
