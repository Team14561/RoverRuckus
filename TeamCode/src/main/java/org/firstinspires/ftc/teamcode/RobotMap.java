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

    //Arm Parameters
    public static final String LEFT_ARM_MOTOR = "arm_left";
    public static final String RIGHT_ARM_MOTOR = "arm_right";
    public static final DcMotor.Direction LEFT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double ARM_SPEED = 0.5;

    //TankDrive Parameters
    public static final Boolean DISPLAY_MOTOR_VALUES = true;
    public static final Boolean REVERSE_JOYSTICKS = false;
    public static final double SPEEDLIMIT = 0.5;

    //Claw Parameters
    public static final String CLAW_SERVO = "claw_servo";
    public static final double RANGE = 180.0;
    public static final double OPEN = 0.0/RANGE;
    public static final double CLOSED = 180.0/RANGE;
    public static final double MINIMUM_ANGLE = 0.0;
    public static final double MAXIMUM_ANGLE = 1.0;
    public static final double SERVO_ANGLE_DEFAULT = CLOSED;
}
