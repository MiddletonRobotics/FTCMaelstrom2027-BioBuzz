package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/** One-motor roller intake. */
public class Intake {
    private final DcMotor motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void in()      { motor.setPower(RobotConstants.INTAKE_POWER); }
    public void reverse() { motor.setPower(-RobotConstants.INTAKE_POWER); }
    public void stop()    { motor.setPower(0); }
}
