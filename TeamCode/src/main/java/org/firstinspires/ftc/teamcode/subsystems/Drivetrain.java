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
        }
        pinpoint = new Pinpoint(hardwareMap);
        if (!pinpoint.found()) warnings.add("no pinpoint, driving robot-centric");
    }

    public DcMotorEx anchor() { return wheels[0]; }

    public void update() { pinpoint.update(); }

    public boolean fieldCentric() { return pinpoint.problem() == null; }

    public void driveField(double forward, double strafe, double turn) {
        double h = fieldCentric() ? pinpoint.heading() : 0;
        drive(forward * Math.cos(h) - strafe * Math.sin(h), forward * Math.sin(h) + strafe * Math.cos(h), turn);
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
    public void resetHeading() { pinpoint.setPose(pinpoint.x(), pinpoint.y(), 0); }
    public double x() { return pinpoint.x(); }
    public double y() { return pinpoint.y(); }
    public double heading() { return pinpoint.heading(); }

    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive", fieldCentric()
                ? String.format("FIELD centric, heading %.0f deg (options = reset)", Math.toDegrees(heading()))
                : "ROBOT centric, " + pinpoint.problem());
    }
}
