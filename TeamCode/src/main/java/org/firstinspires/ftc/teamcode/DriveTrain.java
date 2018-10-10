package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the drivetrain of an FTC robot.
 */
public class DriveTrain {

    // Class variables
    DcMotor leftMotor, rightMotor;
    Telemetry telemetry;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        leftMotor = hardwareMap.get(DcMotor.class, RobotMap.LEFT_DRIVE_MOTOR);
        rightMotor = hardwareMap.get(DcMotor.class, RobotMap.RIGHT_DRIVE_MOTOR);

        // Set the motor directions
        leftMotor.setDirection(RobotMap.LEFT_DRIVE_DIRECTION);
        rightMotor.setDirection(RobotMap.RIGHT_DRIVE_DIRECTION);
    }

    /**
     * Set the drivetrain motor power for both left and right motors
     *
     * @param gamepad The gamepad from which to read joystick values
     */
    public void tankDrive(Gamepad gamepad) {

        // Get joystick values from gamepad
        double leftPower  = gamepad.left_stick_y;
        double rightPower = gamepad.right_stick_y;

        // Limit speed of drivetrain
        leftPower *= RobotMap.SPEEDLIMIT;
        rightPower *= RobotMap.SPEEDLIMIT;

        // Reverse joystick values if requested
        if (RobotMap.REVERSE_JOYSTICKS) {
            leftPower *= -1.0;
            rightPower *= -1.0;
        }

        setPower(leftPower, rightPower);
    }

    private void setPower(double leftPower, double rightPower){
        // Make sure power levels are within expected range
        leftPower = safetyCheck(leftPower);
        rightPower = safetyCheck(rightPower);

        // Send calculated power to motors
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        //Display power levels for operator
        if (RobotMap.DISPLAY_MOTOR_VALUES) {
            telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                    leftPower, rightPower);
        }
    }

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(-1.0, out);
        out = Math.min(1.0, out);
        return out;
    }

}
