package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestOp")
public class TestOp extends OpMode {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor intake, elevator, transfer, shooter;
    public Servo flowerServo, gateServo, shooterServo;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "LeftRear");
        backRight  = hardwareMap.get(DcMotor.class, "RightRear");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter  = hardwareMap.get(DcMotor.class, "shooter");

        flowerServo  = hardwareMap.get(Servo.class, "flowerServo");
        gateServo    = hardwareMap.get(Servo.class, "gateServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
    }

    @Override
    public void loop() {
        // Drivetrain: left stick = both front wheels, right stick = both back wheels
        frontLeft.setPower(-gamepad1.left_stick_y);
        frontRight.setPower(-gamepad1.left_stick_y);
        backLeft.setPower(-gamepad1.right_stick_y);
        backRight.setPower(-gamepad1.right_stick_y);

        // Motors: hold button = on, let go = off
        intake.setPower(gamepad1.right_bumper ? 1 : 0);
        elevator.setPower(gamepad1.dpad_up ? 1 : 0);
        transfer.setPower(gamepad1.right_trigger);
        shooter.setPower(gamepad1.triangle ? 1 : 0);

        // Servos: one button = position 1, other button = position 0
        if (gamepad1.dpad_right) flowerServo.setPosition(1);
        if (gamepad1.dpad_left)  flowerServo.setPosition(0);

        if (gamepad1.circle) gateServo.setPosition(1);
        if (gamepad1.square) gateServo.setPosition(0);

        if (gamepad1.cross)   shooterServo.setPosition(1);
        if (gamepad1.options) shooterServo.setPosition(0);
    }
}
