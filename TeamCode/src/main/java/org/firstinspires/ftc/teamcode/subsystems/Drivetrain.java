package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Mecanum drivetrain with robot-centric and field-centric driving,
 * plus wheel-encoder + IMU odometry (see {@link Odometry}).
 */
public class Drivetrain {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor[] motors;

    /** Null if no IMU is configured; field-centric is then unavailable. */
    private final IMU imu;

    public final Odometry odometry;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT);
        backRight  = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT);
        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        // Left side motors are mounted mirrored, so they need to be reversed.
        // If the robot spins instead of driving forward, swap which side is reversed.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Still reads encoders, just doesn't use them for speed control.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.tryGet(IMU.class, RobotConstants.IMU);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RobotConstants.HUB_LOGO, RobotConstants.HUB_USB)));
            imu.resetYaw();
        }

        odometry = new Odometry();
    }

    /**
     * Robot-centric drive.
     * @param forward positive = drive forward (-1 to 1)
     * @param strafe  positive = strafe right (-1 to 1)
     * @param turn    positive = turn clockwise (-1 to 1)
     */
    public void drive(double forward, double strafe, double turn) {
        strafe *= RobotConstants.STRAFE_CORRECTION;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Find the biggest wheel power (ignoring + or -). Start at 1 so we never speed up.
        double max = 1.0;
        max = Math.max(max, Math.abs(fl));
        max = Math.max(max, Math.abs(fr));
        max = Math.max(max, Math.abs(bl));
        max = Math.max(max, Math.abs(br));

        // Divide every wheel by max so none goes above 1 but they keep the same ratios.
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    /** Field-centric drive: pushing the stick forward always moves away from the driver. */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeading();
        // Rotate the joystick vector by -heading to get robot-relative movement.
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double rotStrafe  = strafe * cos - forward * sin;
        double rotForward = strafe * sin + forward * cos;
        drive(rotForward, rotStrafe, turn);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public boolean hasImu() {
        return imu != null;
    }

    /** Heading in radians, counter-clockwise positive, 0 = facing the way the robot faced at reset. */
    public double getHeading() {
        if (imu == null) return 0;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Makes the robot's current facing direction "forward" for field-centric. */
    public void resetHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    /** Call once per loop so odometry keeps tracking. */
    public void update() {
        odometry.update(
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                getHeading());
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(getHeading()));
        telemetry.addData("Pose (in)", "x %.1f  y %.1f", odometry.getX(), odometry.getY());
        telemetry.addData("Encoders FL/FR/BL/BR", "%d / %d / %d / %d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
    }
}
