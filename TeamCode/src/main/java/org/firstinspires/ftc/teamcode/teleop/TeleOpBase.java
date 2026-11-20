package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

public abstract class TeleOpBase extends OpMode {
    private static final int RUMBLE_MS = 300;

    private final List<String> warnings = new ArrayList<>();
    private final HiveShot hiveShot = new HiveShot();
    private List<LynxModule> hubs;
    private Buttons buttons;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Elevator elevator;
    private Servos servos;

    private boolean hive = true;
    private boolean lastSwitch = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        buttons = new Buttons(gamepad1, gamepad2);

        drivetrain = new Drivetrain(hardwareMap, warnings);
        intake = new Intake(hardwareMap, drivetrain.anchor(), warnings);
        shooter = new Shooter(hardwareMap, drivetrain.anchor(), warnings);
        elevator = new Elevator(hardwareMap, drivetrain.anchor(), warnings);
        servos = new Servos(hardwareMap, warnings);
    }

    @Override
    public void init_loop() {
        drivetrain.update();
        telemetry.addLine("Ready. Starts in HIVE mode.");
        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.update();
    }

    @Override
    public void start() {
        showMode();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        if (buttons.switchMode() && !lastSwitch) {
            hive = !hive;
            showMode();
        }
        lastSwitch = buttons.switchMode();

        drivetrain.update();
        drivetrain.drive(buttons.forward(), buttons.strafe(), buttons.turn());

        boolean moving = buttons.forward() != 0 || buttons.strafe() != 0 || buttons.turn() != 0;
        hiveShot.update(buttons.autoShot(), hive, moving, shooter.ready());

        boolean unjam = buttons.unjam();
        boolean shooting = hive && buttons.shoot() && !unjam;
        boolean feeding = shooting || (hive && buttons.transferFull() && !unjam);

        if (hiveShot.shooterOn() && !feeding) shooter.auto(true);
        else shooter.run(feeding, hive);

        if (hiveShot.feeding() && !unjam) {
            intake.feedShooter();
        } else {
            intake.run(hive, buttons.intake(), buttons.spitOut(), unjam,
                    buttons.transferFull(), buttons.transferReverse(), buttons.transfer());
        }

        servos.set(buttons.gate(), hive, buttons.deFlower());

        boolean manualElevator = buttons.elevatorUp() || buttons.elevatorDown();
        if (hiveShot.active() && !manualElevator) elevator.run(hiveShot.elevatorUp(), hiveShot.elevatorDown());
        else elevator.run(buttons.elevatorUp(), buttons.elevatorDown());

        showTelemetry();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.stop();
        shooter.stop();
        elevator.stop();
    }

    private void showMode() {
        for (Gamepad g : new Gamepad[]{gamepad1, gamepad2}) {
            if (hive) g.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            else g.setLedColor(0.6, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            g.rumble(RUMBLE_MS);
        }
    }

    private void showTelemetry() {
        telemetry.addLine(hive ? "===== HIVE MODE (shooter) =====" : "===== FLOWER MODE (elevator) =====");
        for (String warning : warnings) telemetry.addLine("! " + warning);
        hiveShot.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        elevator.addTelemetry(telemetry);
        servos.addTelemetry(telemetry);
        telemetry.update();
    }
}
