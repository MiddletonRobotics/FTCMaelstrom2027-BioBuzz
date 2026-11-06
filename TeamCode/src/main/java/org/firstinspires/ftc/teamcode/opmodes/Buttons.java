package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class Buttons {
    private static final double TRIGGER = 0.1;
    private static final double DEADZONE = 0.1;

    private final Gamepad g1;
    private final Gamepad g2;

    public Buttons(Gamepad g1, Gamepad g2) {
        this.g1 = g1;
        this.g2 = g2;
    }

    public double forward() { return deadzone(-g1.left_stick_y); }
    public double strafe() { return deadzone(g1.left_stick_x); }
    public double turn() { return deadzone(g1.right_stick_x); }

    public boolean intake() { return g1.left_trigger > TRIGGER; }
    public boolean shoot() { return g1.right_trigger > TRIGGER; }
    public boolean unjam() { return g1.right_bumper; }
    public boolean resetHeading() { return g1.options; }

    public boolean elevatorUp() { return g1.dpad_up; }
    public boolean elevatorDown() { return g1.dpad_down; }

    public boolean switchMode() { return g2.triangle; }
    public boolean gate() { return g2.circle; }
    public boolean spitOut() { return g2.left_bumper; }

    private static double deadzone(double value) {
        if (Math.abs(value) < DEADZONE) return 0;
        return Math.signum(value) * (Math.abs(value) - DEADZONE) / (1 - DEADZONE);
    }
}
