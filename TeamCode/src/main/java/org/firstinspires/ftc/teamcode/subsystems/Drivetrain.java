package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {
    private final DcMotorEx[] wheels;
    private final Pinpoint pinpoint;
    private final IMU imu;

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
        imu = hardwareMap.tryGet(IMU.class, RobotConstants.IMU);
        if (imu == null) warnings.add("no \"" + RobotConstants.IMU + "\" in config, field centric drives robot centric");
        else imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RobotConstants.HUB_LOGO, RobotConstants.HUB_USB)));
    }

    public DcMotorEx anchor() { return wheels[0]; }

    public void update() { pinpoint.update(); }

    public double yaw() {
        if (imu == null) return 0;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetYaw() {
        if (imu != null) imu.resetYaw();
    }

    public void driveFieldCentric(double forward, double strafe, double turn) {
        double yaw = yaw();
        double cos = Math.cos(yaw), sin = Math.sin(yaw);
        drive(forward * cos - strafe * sin, forward * sin + strafe * cos, turn);
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
}
