package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * Full robot. Gamepad 1 drives (see {@link DriverControls}), gamepad 2 runs the mechanisms:
 *   Right bumper     intake in (hold)        Left bumper    intake reverse (hold)
 *   D-pad up/right/down  elevator UP / TRANSFER / DOWN
 *   Left stick Y     elevator manual nudge
 *   Right trigger    feeder into shooter (hold)   Left trigger  feeder reverse (hold)
 *   Cross (A)        toggle flower servo
 *   Square (X)       toggle gate servo
 *   Triangle (Y)     toggle shooter spin-up
 *   Circle (B)       toggle shooter servo (if configured)
 *
 * Disabled until every mechanism is wired and in the Driver Hub configuration.
 * Delete the @Disabled line to make it show up on the Driver Hub.
 */
@Disabled
@TeleOp(name = "BioBuzz TeleOp", group = "BioBuzz")
public class BioBuzzTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private DriverControls driverControls;
    private Intake intake;
    private Elevator elevator;
    private Transfer transfer;
    private Shooter shooter;
    private final Gamepad previous2 = new Gamepad();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Drivetrain(hardwareMap);
        driverControls = new DriverControls(drivetrain);
        intake   = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter  = new Shooter(hardwareMap);
        telemetry.addLine("BioBuzz ready. Elevator must be fully DOWN. Press START.");
    }

    @Override
    public void loop() {
        driverControls.update(gamepad1, telemetry);

        Gamepad g = gamepad2;

        if (g.right_bumper)     intake.in();
        else if (g.left_bumper) intake.reverse();
        else                    intake.stop();

        if (g.dpad_up)          elevator.goToUp();
        else if (g.dpad_right)  elevator.goToTransfer();
        else if (g.dpad_down)   elevator.goToDown();
        elevator.manual(-g.left_stick_y);

        if (g.right_trigger > 0.5)     transfer.feed();
        else if (g.left_trigger > 0.5) transfer.reverseFeed();
        else                           transfer.stopFeed();

        if (g.cross && !previous2.cross)       transfer.toggleFlower();
        if (g.square && !previous2.square)     transfer.toggleGate();
        if (g.triangle && !previous2.triangle) shooter.toggle();
        if (g.circle && !previous2.circle)     shooter.toggleServo();
        previous2.copy(g);

        elevator.addTelemetry(telemetry);
        transfer.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.stop();
        transfer.stopFeed();
        shooter.stop();
    }
}
