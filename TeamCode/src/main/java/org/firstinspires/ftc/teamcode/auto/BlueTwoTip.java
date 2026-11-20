package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Two Tip", group = "Auto")
public class BlueTwoTip extends TwoTipAuto {
    @Override
    protected boolean blue() { return true; }
}
