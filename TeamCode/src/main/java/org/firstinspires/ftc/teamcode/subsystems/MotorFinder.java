package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public final class MotorFinder {
    private MotorFinder() {}

    public static final String[] DRIVE_NAMES = {
            RobotConstants.FRONT_LEFT, RobotConstants.FRONT_RIGHT,
            RobotConstants.BACK_LEFT, RobotConstants.BACK_RIGHT};
    private static final int[] DRIVE_PORTS = {
            RobotConstants.FRONT_LEFT_PORT, RobotConstants.FRONT_RIGHT_PORT,
            RobotConstants.BACK_LEFT_PORT, RobotConstants.BACK_RIGHT_PORT};

    public static DcMotorEx[] drive(HardwareMap hardwareMap, List<String> notes) {
        DcMotorEx[] motors = new DcMotorEx[4];
        DcMotorController hub = null;
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, DRIVE_NAMES[i]);
            if (motors[i] != null && hub == null) hub = motors[i].getController();
        }
        if (hub == null) {
            throw new IllegalStateException("No drive motors found. Name them frontLeft, frontRight,"
                    + " backLeft, backRight in the robot configuration.");
        }
        for (int i = 0; i < 4; i++) {
            if (motors[i] == null) {
                motors[i] = onPort(hardwareMap, DRIVE_PORTS[i], hub, true);
                if (motors[i] == null) {
                    throw new IllegalStateException("Can't find " + DRIVE_NAMES[i] + " (control hub port "
                            + DRIVE_PORTS[i] + "). Add it to the robot configuration.");
                }
                notes.add(DRIVE_NAMES[i] + " not in config, using control hub port " + DRIVE_PORTS[i]);
            } else if (motors[i].getController() != hub) {
                notes.add(DRIVE_NAMES[i] + " is on a different hub than the other drive motors, check config");
            }
        }
        return motors;
    }

    public static DcMotorEx expansion(HardwareMap hardwareMap, String name, int port,
                                      DcMotorEx driveMotor, List<String> notes) {
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
        if (motor != null) return motor;
        motor = onPort(hardwareMap, port, driveMotor.getController(), false);
        if (motor == null) {
            throw new IllegalStateException("Can't find " + name + " (expansion hub port " + port
                    + "). Add it to the robot configuration.");
        }
        notes.add(name + " not in config, using expansion hub port " + port);
        return motor;
    }

    public static void checkUnique(DcMotorEx... motors) {
        for (int i = 0; i < motors.length; i++) {
            for (int j = i + 1; j < motors.length; j++) {
                if (motors[i].getController() == motors[j].getController()
                        && motors[i].getPortNumber() == motors[j].getPortNumber()) {
                    throw new IllegalStateException("Two motors in the config are on the same port ("
                            + motors[i].getPortNumber() + "). Fix the robot configuration.");
                }
            }
        }
    }

    private static DcMotorEx onPort(HardwareMap hardwareMap, int port, DcMotorController hub, boolean sameHub) {
        for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
            if (motor.getPortNumber() == port && (motor.getController() == hub) == sameHub) return motor;
        }
        return null;
    }
}
