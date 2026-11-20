package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HiveShot {
    private static final double STILL_S = 2.0;
    private static final double LIFT_S = 0.5;
    private static final double LOWER_S = 0.5;

    private enum State { OFF, SPIN_UP, FIRE, LOWER }

    private final ElapsedTime stillTime = new ElapsedTime();
    private final ElapsedTime stateTime = new ElapsedTime();
    private State state = State.OFF;
    private boolean lastButton = false;

    public void update(boolean button, boolean hive, boolean moving, boolean shooterReady) {
        boolean pressed = button && !lastButton;
        lastButton = button;

        if (!hive && shooterOn()) go(State.LOWER);
        if (pressed && hive) go(state == State.OFF || state == State.LOWER ? State.SPIN_UP : State.LOWER);
        if (moving) stillTime.reset();

        if (state == State.SPIN_UP && stillTime.seconds() >= STILL_S && shooterReady) go(State.FIRE);
        else if (state == State.FIRE && moving) go(State.SPIN_UP);
        else if (state == State.LOWER && stateTime.seconds() > LOWER_S) go(State.OFF);
    }

    public boolean active() { return state != State.OFF; }
    public boolean shooterOn() { return state == State.SPIN_UP || state == State.FIRE; }
    public boolean feeding() { return state == State.FIRE; }
    public boolean elevatorUp() { return state == State.FIRE && stateTime.seconds() < LIFT_S; }
    public boolean elevatorDown() { return state == State.LOWER; }

    public void addTelemetry(Telemetry telemetry) {
        String text;
        switch (state) {
            case SPIN_UP:
                text = String.format("SPINNING UP - hold still %.1f / %.0f s", Math.min(stillTime.seconds(), STILL_S), STILL_S);
                break;
            case FIRE:
                text = "FIRING (intake full + elevator up)";
                break;
            case LOWER:
                text = "elevator down";
                break;
            default:
                text = "off (D2 L1 to start)";
        }
        telemetry.addData("Auto shot", text);
    }

    private void go(State next) {
        state = next;
        stateTime.reset();
    }
}
