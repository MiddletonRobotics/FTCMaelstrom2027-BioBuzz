package org.firstinspires.ftc.teamcode.testing.tuning.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotorFinder;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Wheel Check", group = "Tuning")
public class WheelCheck extends OpMode {
    private static final double POWER = 0.3;
    private static final String[] NAMES = {"FRONT LEFT", "FRONT RIGHT", "BACK LEFT", "BACK RIGHT"};

    private final List<String> notes = new ArrayList<>();
    private DcMotorEx[] wheels;

    @Override
    public void init() {
        boolean[] reversed = {
                RobotConstants.FRONT_LEFT_REVERSED, RobotConstants.FRONT_RIGHT_REVERSED,
                RobotConstants.BACK_LEFT_REVERSED, RobotConstants.BACK_RIGHT_REVERSED};
        wheels = MotorFinder.drive(hardwareMap, notes);
        for (int i = 0; i < 4; i++) {
            wheels[i].setDirection(MotorFinder.direction(reversed[i]));
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addLine("Robot on a box, wheels in the air. Press START.");
        for (String note : notes) telemetry.addLine("! " + note);
    }

    @Override
    public void loop() {
        boolean[] held = {gamepad1.left_bumper, gamepad1.right_bumper,
                gamepad1.left_trigger > 0.3, gamepad1.right_trigger > 0.3};
        boolean all = gamepad1.dpad_up;
        String spinning = all ? "ALL FOUR" : "nothing";
        for (int i = 0; i < 4; i++) {
            boolean on = all || held[i];
            wheels[i].setPower(on ? POWER : 0);
            if (on && !all) spinning = NAMES[i];
        }

        telemetry.addLine("HOLD a button. That wheel should roll FORWARD");
        telemetry.addLine("(top of the wheel moves toward the intake / front).");
        telemetry.addLine();
        telemetry.addLine("LB = front left      RB = front right");
        telemetry.addLine("LT = back left       RT = back right");
        telemetry.addLine("Dpad up = all four");
        telemetry.addLine();
        telemetry.addData("Spinning", spinning);
        for (int i = 0; i < 4; i++) {
            telemetry.addData(NAMES[i], "control hub port %d", wheels[i].getPortNumber());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
    }
}
