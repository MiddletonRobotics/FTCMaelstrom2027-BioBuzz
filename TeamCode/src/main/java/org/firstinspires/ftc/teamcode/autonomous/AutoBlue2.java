package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE 2 - Audience Side", group = "BioBuzz")
public class AutoBlue2 extends HalfPark {
    @Override
    protected String title() { return "BLUE 2 - Audience Side"; }

    @Override
    protected String placement() { return "BACK against BLUE wall, center 17.5 in from AUDIENCE wall, facing red wall"; }

    @Override
    protected double[] startPose() { return new double[]{136, 17.5, 180}; }

    @Override
    protected double[][] legs() { return new double[][]{{84, 17.5}, {84, 29}, {132, 29}}; }

    @Override
    protected String[] legNames() { return new String[]{"forward", "strafe right", "back up to park"}; }
}
