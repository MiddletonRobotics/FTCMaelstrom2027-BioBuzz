package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Iterator;

@Disabled
@TeleOp(name = "Hardware Check", group = "Testing")
public class HardwareCheck extends OpMode {
    private static final String[] MOTORS = {
            RobotConstants.FRONT_LEFT, RobotConstants.FRONT_RIGHT,
            RobotConstants.BACK_LEFT, RobotConstants.BACK_RIGHT,
            RobotConstants.INTAKE, RobotConstants.ELEVATOR,
            RobotConstants.FEEDER, RobotConstants.SHOOTER};
    private static final String[] SERVOS = {
            RobotConstants.FLOWER_SERVO, RobotConstants.GATE_SERVO, RobotConstants.SHOOTER_SERVO};

    private final DcMotor[] motors = new DcMotor[MOTORS.length];

    @Override
    public void init() {
        for (int i = 0; i < MOTORS.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotor.class, MOTORS[i]);
        }
    }

    @Override
    public void init_loop() {
        report();
    }

    @Override
    public void loop() {
        report();
    }

    private void report() {
        telemetry.addLine("--- Motors (encoder ticks) ---");
        for (int i = 0; i < MOTORS.length; i++) {
            telemetry.addData(MOTORS[i], motors[i] == null ? "MISSING" : String.valueOf(motors[i].getCurrentPosition()));
        }
        telemetry.addLine("--- Servos ---");
        for (String name : SERVOS) {
            telemetry.addData(name, found(Servo.class, name));
        }
        telemetry.addLine("--- Other ---");
        telemetry.addData(RobotConstants.IMU, found(IMU.class, RobotConstants.IMU));
        telemetry.addData("pinpoint", found(GoBildaPinpointDriver.class, "pinpoint"));
        Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        if (sensors.hasNext()) telemetry.addData("Battery (V)", "%.2f", sensors.next().getVoltage());
        telemetry.update();
    }

    private String found(Class<?> type, String name) {
        return hardwareMap.tryGet(type, name) == null ? "MISSING" : "found";
    }
}
