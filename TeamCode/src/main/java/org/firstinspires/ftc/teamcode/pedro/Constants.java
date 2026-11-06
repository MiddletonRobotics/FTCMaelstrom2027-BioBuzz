package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConstants.FRONT_LEFT);
        c.frontRightName.set(RobotConstants.FRONT_RIGHT);
        c.backLeftName.set(RobotConstants.BACK_LEFT);
        c.backRightName.set(RobotConstants.BACK_RIGHT);
        c.frontLeftDirection.set(MotorFinder.direction(RobotConstants.FRONT_LEFT_REVERSED));
        c.frontRightDirection.set(MotorFinder.direction(RobotConstants.FRONT_RIGHT_REVERSED));
        c.backLeftDirection.set(MotorFinder.direction(RobotConstants.BACK_LEFT_REVERSED));
        c.backRightDirection.set(MotorFinder.direction(RobotConstants.BACK_RIGHT_REVERSED));
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(RobotConstants.PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(RobotConstants.POD_X_OFFSET);
        c.yPodOffset.set(RobotConstants.POD_Y_OFFSET);
        c.xPodDirection.set(Pinpoint.direction(RobotConstants.POD_X_REVERSED));
        c.yPodDirection.set(Pinpoint.direction(RobotConstants.POD_Y_REVERSED));
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
    });

    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Algorithm algorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), algorithm());
    }
}
