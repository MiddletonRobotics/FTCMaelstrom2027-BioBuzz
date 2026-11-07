package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED 1 - Audience Wall", group = "BioBuzz")
public class AutoRed1 extends HalfPark {
    @Override
    protected String title() { return "RED 1 - Audience Wall"; }

    @Override
    protected String placement() { return "BACK against AUDIENCE wall, center 57 in from RED wall, facing away from audience"; }

    @Override
    protected double[] startPose() { return new double[]{57, 9, 90}; }

    @Override
    protected double[][] legs() { return new double[][]{{57, 30}, {13, 30}, {13, 97}}; }

    @Override
    protected String[] legNames() { return new String[]{"forward", "strafe left to red wall", "forward to park"}; }
}
