package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

public class Tuning {
    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::algorithm);
    }

    @Tuner(name = "OTOS Tuner")
    public static Procedure otos() {
        return new OTOSTuner();
    }

    @Tuner(name = "OctoQuad Tuner")
    public static Procedure octoQuad() {
        return new OctoQuadTuner();
    }

    @Tuner(name = "Two Wheel Tuner")
    public static Procedure twoWheel() {
        return new TwoWheelTuner();
    }

    @Tuner(name = "Three Wheel Tuner")
    public static Procedure threeWheel() {
        return new ThreeWheelTuner();
    }

    @Tuner(name = "Three Wheel IMU Tuner")
    public static Procedure threeWheelImu() {
        return new ThreeWheelIMUTuner();
    }
}
