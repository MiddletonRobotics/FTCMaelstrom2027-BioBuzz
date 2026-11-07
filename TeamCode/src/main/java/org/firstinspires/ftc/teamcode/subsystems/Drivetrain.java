package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {
    private final DcMotorEx[] wheels;
    private final Pinpoint pinpoint;
    private final FieldCentric field = new FieldCentric();
    private long lastNanos = 0;

    public Drivetrain(HardwareMap hardwareMap) {
        this(hardwareMap, new ArrayList<>());
    }

    public Drivetrain(HardwareMap hardwareMap, List<String> warnings) {
        wheels = MotorFinder.drive(hardwareMap, warnings);
        boolean[] reversed = {
                RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
                RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};
        for (int i = 0; i < 4; i++) {
            wheels[i].setDirection(MotorFinder.direction(reversed[i]));
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        pinpoint = new Pinpoint(hardwareMap);
        if (!pinpoint.found()) warnings.add("no pinpoint, driving robot-centric");
    }

    public DcMotorEx anchor() { return wheels[0]; }

    public void update() { pinpoint.update(); }

    public boolean fieldCentric() {
        return RobotConstants.FIELD_CENTRIC && pinpoint.found() && field.learned();
    }

    public void resetForward() { field.zero(pinpoint.heading()); }

    public void driveField(double forward, double strafe, double turn) {
        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0 : (now - lastNanos) / 1e9;
        lastNanos = now;
        double heading = pinpoint.found() ? pinpoint.heading() : Double.NaN;
        field.update(heading, turn, dt);
        double angle = RobotConstants.FIELD_CENTRIC ? field.angle(heading) : 0;
        double[] moved = FieldCentric.rotate(forward, strafe, angle);
        drive(moved[0], moved[1], turn);
    }

    public void drive(double forward, double strafe, double turn) {
        strafe *= RobotConstants.STRAFE_CORRECTION;
        turn *= RobotConstants.TURN_SCALE;
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        wheels[0].setPower((forward + strafe + turn) / max);
        wheels[1].setPower((forward - strafe - turn) / max);
        wheels[2].setPower((forward - strafe + turn) / max);
        wheels[3].setPower((forward + strafe - turn) / max);
    }

    public void resetPose() { pinpoint.setPose(0, 0, 0); }
    public void setPose(double x, double y, double headingDeg) { pinpoint.setPose(x, y, headingDeg); }
    public boolean hasPinpoint() { return pinpoint.found(); }
    public String pinpointProblem() { return pinpoint.problem(); }
    public void resetHeading() { pinpoint.setPose(pinpoint.x(), pinpoint.y(), 0); }
    public double x() { return pinpoint.x(); }
    public double y() { return pinpoint.y(); }
    public double heading() { return pinpoint.heading(); }

    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
    }

    public void addWheelTelemetry(Telemetry telemetry) {
        String[] names = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            telemetry.addData("Wheel " + names[i], "power %.2f  ticks %d  port %d  mode %s",
                    wheels[i].getPower(), wheels[i].getCurrentPosition(),
                    wheels[i].getPortNumber(), wheels[i].getMode());
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        if (!RobotConstants.FIELD_CENTRIC) {
            telemetry.addData("Drive", "ROBOT centric (FIELD_CENTRIC is off)");
            return;
        }
        if (!pinpoint.found()) {
            telemetry.addData("Drive", "ROBOT centric (no pinpoint in config)");
            return;
        }
        if (field.dead()) {
            telemetry.addData("Drive", "ROBOT centric");
            telemetry.addLine("! pinpoint heading doesn't change when turning, check its cable");
        } else if (!field.learned()) {
            telemetry.addData("Drive", "robot centric until the first turn, then FIELD centric");
        } else {
            telemetry.addData("Drive", "FIELD centric  %.0f deg from forward%s",
                    Math.toDegrees(field.angle(heading())), field.flipped() ? "  (pinpoint flipped, handled)" : "");
        }
        String problem = pinpoint.problem();
        if (problem != null) telemetry.addLine("  " + problem);
    }
}
