package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "6KRPM")
public class SixKRpm extends OpMode {
    private static final String MOTOR = "motor0";
    private static final double FREE_RPM = 6000;
    private static final double MAX_RPM = 6000;
    private static final double RAMP_PER_SEC = 0.5;

    private final double maxPower = MAX_RPM / FREE_RPM;
    private final ElapsedTime timer = new ElapsedTime();
    private DcMotor motor;
    private double power = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        double dt = timer.seconds();
        timer.reset();

        double target = -gamepad1.left_stick_y * maxPower;
        double step = RAMP_PER_SEC * dt;
        power += Range.clip(target - power, -step, step);
        motor.setPower(power);

        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Approx RPM", "%.0f", power * FREE_RPM);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
