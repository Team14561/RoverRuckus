package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class Arm {

    // Class variables
    DcMotor leftMotor, rightMotor, encoderMotor;
    Telemetry telemetry;
    int encoderZero;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        leftMotor = hardwareMap.get(DcMotor.class, RobotMap.LEFT_ARM_MOTOR);
        rightMotor = hardwareMap.get(DcMotor.class, RobotMap.RIGHT_ARM_MOTOR);

        // Set the motor directions
        leftMotor.setDirection(RobotMap.LEFT_ARM_DIRECTION);
        rightMotor.setDirection(RobotMap.RIGHT_ARM_DIRECTION);

        //Set the encoder starting position
        encoderMotor = leftMotor;
        encoderZero = encoderMotor.getCurrentPosition();
    }

    /**
     * Set the arm motor power for both left and right motors
     *
     * @param gamepad The gamepad from which to read joystick values
     */
    public void manual(Gamepad gamepad) {

        // Get joystick values from gamepad
        double power  = gamepad.right_stick_y;

        // Limit speed of drivetrain
        power *= RobotMap.ARM_SPEED;

        setPower(power);

        //output the encoder value//
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Arm Encoder", encoderDegrees());
        }
    }

    private void setPower(double power  ){
        // Make sure power levels are within expected range
        power = safetyCheck(power);

        // Send calculated power to motors
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(-1.0, out);
        out = Math.min(1.0, out);
        return out;
    }

    private double encoderDegrees() {
        return (encoderMotor.getCurrentPosition()-encoderZero)/RobotMap.ARM_SCALE;
    }

}
