package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Drivetrain {
    private final DcMotorEx[] wheels;
    private final Pinpoint pinpoint;

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
    }

    public DcMotorEx anchor() { return wheels[0]; }

    public void update() { pinpoint.update(); }

    public void drive(double forward, double strafe, double turn) {
        strafe *= RobotConstants.STRAFE_CORRECTION;
        turn *= RobotConstants.TURN_SCALE;
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        wheels[0].setPower((forward + strafe + turn) / max);
        wheels[1].setPower((forward - strafe - turn) / max);
        wheels[2].setPower((forward - strafe + turn) / max);
        wheels[3].setPower((forward + strafe - turn) / max);
    }

    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
    }

    public boolean hasPinpoint() { return pinpoint.found(); }
    public String pinpointProblem() { return pinpoint.problem(); }
    public void setPose(double x, double y, double headingDeg) { pinpoint.setPose(x, y, headingDeg); }
    public double x() { return pinpoint.x(); }
    public double y() { return pinpoint.y(); }
    public double heading() { return pinpoint.heading(); }
}
