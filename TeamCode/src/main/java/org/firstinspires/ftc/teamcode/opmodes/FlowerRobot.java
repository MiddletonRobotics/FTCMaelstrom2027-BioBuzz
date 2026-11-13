package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "flowerv2 ROBOT centric", group = "BioBuzz")
public class FlowerRobot extends OpMode {
    private static final int RUMBLE_MS = 300;

    private final List<String> warnings = new ArrayList<>();
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
        telemetry.addLine("flowerv2 ROBOT centric ready. Starts in HIVE mode.");
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

        boolean reverse = buttons.unjam();
        boolean shooting = hive && buttons.shoot() && !reverse;
        boolean feeding = shooting || (hive && buttons.transferFull() && !reverse);
        shooter.run(feeding);
        intake.run(hive, buttons.intake(), buttons.spitOut(), reverse, shooting, buttons.transferFull(), buttons.transferReverse(), warnings);
        servos.set(buttons.gate(), hive, buttons.deFlower());
        elevator.run(buttons.elevatorUp(), buttons.elevatorDown());

        telemetry.addLine(hive ? "===== HIVE MODE (shooter) =====" : "===== FLOWER MODE (elevator) =====");
        for (String warning : warnings) telemetry.addLine("! " + warning);
        telemetry.addData("Drive", "ROBOT centric");
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        elevator.addTelemetry(telemetry);
        servos.addTelemetry(telemetry);
        telemetry.update();
    }

    private void showMode() {
        for (Gamepad g : new Gamepad[]{gamepad1, gamepad2}) {
            if (hive) g.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            else g.setLedColor(0.6, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            g.rumble(RUMBLE_MS);
        }
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.stop();
        shooter.stop();
        elevator.stop();
        servos.stop();
    }
}
