package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class Buttons {
    private static final double TRIGGER = 0.1;

    private final Gamepad g1;
    private final Gamepad g2;

    public Buttons(Gamepad g1, Gamepad g2) {
        this.g1 = g1;
        this.g2 = g2;
    }

    public double forward() { return -g1.left_stick_y; }
    public double strafe() { return g1.left_stick_x; }
    public double turn() { return g1.right_stick_x; }

    public boolean intakeIn() { return g1.left_trigger > TRIGGER || g2.left_trigger > TRIGGER; }
    public boolean intakeOut() { return g1.left_bumper || g2.left_bumper; }

    public boolean transferIn() { return g1.right_trigger > TRIGGER || g2.right_trigger > TRIGGER; }
    public boolean transferOut() { return g1.right_bumper || g2.right_bumper; }

    public boolean elevatorUp() { return g1.triangle || g2.triangle; }
    public boolean elevatorDown() { return g1.cross || g2.cross; }
    public boolean elevatorNudgeUp() { return g1.dpad_up || g2.dpad_up; }
    public boolean elevatorNudgeDown() { return g1.dpad_down || g2.dpad_down; }

    public boolean gate() { return g1.circle || g2.circle; }
    public boolean ramp() { return g1.square || g2.square; }
}
