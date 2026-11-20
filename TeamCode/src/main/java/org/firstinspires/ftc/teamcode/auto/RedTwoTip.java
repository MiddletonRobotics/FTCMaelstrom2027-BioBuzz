package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Two Tip", group = "Auto")
public class RedTwoTip extends TwoTipAuto {
    @Override
    protected boolean blue() { return false; }
}
