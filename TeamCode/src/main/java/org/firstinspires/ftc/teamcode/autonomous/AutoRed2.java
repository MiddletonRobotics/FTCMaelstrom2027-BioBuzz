package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED 2 - Scoring Side", group = "BioBuzz")
public class AutoRed2 extends HalfPark {
    @Override
    protected String title() { return "RED 2 - Scoring Side"; }

    @Override
    protected String placement() { return "BACK against RED wall, center 17.5 in from SCORING-SIDE wall, facing blue wall"; }

    @Override
    protected double[] startPose() { return new double[]{8, 126.5, 0}; }

    @Override
    protected double[][] legs() { return new double[][]{{60, 126.5}, {60, 115}, {12, 115}}; }

    @Override
    protected String[] legNames() { return new String[]{"forward", "strafe right", "back up to park"}; }
}
