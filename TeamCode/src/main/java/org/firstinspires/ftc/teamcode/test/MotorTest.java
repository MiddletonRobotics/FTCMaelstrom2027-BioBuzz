package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
    public DcMotor frontRight, frontLeft, rightRear, leftRear;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightRear  = hardwareMap.get(DcMotor.class, "RightRear");
        leftRear   = hardwareMap.get(DcMotor.class, "LeftRear");
    }

    @Override
    public void loop() {
        // Cross (X) = spin all 4 slow (0.3 power = about 94 RPM on a 312 RPM motor)
        if (gamepad1.cross) {
            frontRight.setPower(0.3);
            frontLeft.setPower(0.3);
            rightRear.setPower(0.3);
            leftRear.setPower(0.3);
        }

        // Circle = stop all 4
        if (gamepad1.circle) {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            rightRear.setPower(0);
            leftRear.setPower(0);
        }

        telemetry.addData("Motor power", frontRight.getPower());
        telemetry.update();
    }
}
