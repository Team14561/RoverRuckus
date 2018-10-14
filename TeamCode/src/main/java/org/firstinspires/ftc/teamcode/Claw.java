package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class Claw {

    // Class variables
    Servo clawServo;
    Telemetry telemetry;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        clawServo = hardwareMap.get(Servo.class, RobotMap.CLAW_SERVO);
    }

    /**
     * Move Servo
     *
     *
     * @param gamepad The gamepad from which to read joystick values
     */
   public void buttonServo(Gamepad gamepad){
       double servoAngle = RobotMap.SERVO_ANGLE_DEFAULT;
        if (gamepad.b){
            servoAngle = RobotMap.OPEN;
        }

        servoAngle = safetyCheck(servoAngle);
        clawServo.setPosition(servoAngle);
   }

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(RobotMap.MINIMUM_ANGLE, out);
        out = Math.min(RobotMap.MAXIMUM_ANGLE, out);
        return out;
    }

}
