package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Disabled
@TeleOp(name = "BioBuzz TeleOp", group = "BioBuzz")
public class BioBuzzTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private DriverControls driverControls;
    private Intake intake;
    private Elevator elevator;
    private Transfer transfer;
    private Shooter shooter;
    private final Gamepad previous = new Gamepad();
    private boolean shooterWasReady = false;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Drivetrain(hardwareMap);
        driverControls = new DriverControls(drivetrain);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        telemetry.addLine("BioBuzz ready. Elevator must be fully DOWN. Press START.");
    }

    @Override
    public void loop() {
        driverControls.update(gamepad1, telemetry);

        Gamepad g = gamepad2;

        if (g.right_bumper) intake.in();
        else if (g.left_bumper) intake.reverse();
        else intake.stop();

        if (g.dpad_up) elevator.goToUp();
        else if (g.dpad_right) elevator.goToTransfer();
        else if (g.dpad_down) elevator.goToDown();
        if (Math.abs(g.left_stick_y) > RobotConstants.STICK_DEADBAND) elevator.manual(-g.left_stick_y);

        if (g.right_trigger > 0.5) transfer.feed();
        else if (g.left_trigger > 0.5) transfer.reverseFeed();
        else transfer.stopFeed();

        if (g.cross && !previous.cross) transfer.toggleFlower();
        if (g.square && !previous.square) transfer.toggleGate();
        if (g.triangle && !previous.triangle) shooter.toggle();
        if (g.circle && !previous.circle) shooter.toggleServo();
        if (g.options && !previous.options) shooter.adjustTargetRpm(RobotConstants.SHOOTER_RPM_STEP);
        if (g.share && !previous.share) shooter.adjustTargetRpm(-RobotConstants.SHOOTER_RPM_STEP);
        if (g.ps && !previous.ps) elevator.resetZero();
        previous.copy(g);

        intake.update();
        elevator.update();

        boolean ready = shooter.isReady();
        if (ready && !shooterWasReady) g.rumble(300);
        shooterWasReady = ready;

        intake.addTelemetry(telemetry);
        elevator.addTelemetry(telemetry);
        transfer.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.stop();
        transfer.reset();
        shooter.stop();
    }
}
