package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE 1 - Scoring Side", group = "BioBuzz")
public class AutoBlue1 extends HalfPark {
    @Override
    protected String title() { return "BLUE 1 - Scoring Side"; }

    @Override
    protected String placement() { return "BACK against SCORING-SIDE wall, center 57 in from BLUE wall, facing audience"; }

    @Override
    protected double[] startPose() { return new double[]{87, 135, 270}; }

    @Override
    protected double[][] legs() { return new double[][]{{87, 114}, {131, 114}, {131, 47}}; }

    @Override
    protected String[] legNames() { return new String[]{"forward", "strafe left to blue wall", "forward to park"}; }
}
