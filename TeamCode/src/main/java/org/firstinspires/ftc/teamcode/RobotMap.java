package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotMap{

    // Robot Parameters
    public static final Boolean DISPLAY_TIME = true;

    // Drivetrain Parameters
    public static final String LEFT_DRIVE_MOTOR = "left_drive";
    public static final String RIGHT_DRIVE_MOTOR = "right_drive";
    public static final DcMotor.Direction LEFT_DRIVE_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_DRIVE_DIRECTION = DcMotor.Direction.REVERSE;

    //TankDrive Parameters
    public static final Boolean DISPLAY_MOTOR_VALUES = true;
    public static final Boolean REVERSE_JOYSTICKS = false;
    public static final double SPEEDLIMIT = 0.5;
}
