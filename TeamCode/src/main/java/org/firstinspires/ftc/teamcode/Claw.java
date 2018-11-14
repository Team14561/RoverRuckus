package org.firstinspires.ftc.teamcode;

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
     * @param gamepad The gamepad from which to read joystick values
     */
    private boolean clawOpen = false;
    private boolean aReleased = true;
    public void buttonServo(Gamepad gamepad) {
        double servoAngle = RobotMap.SERVO_ANGLE_DEFAULT;

        if (gamepad.a) {
            if (aReleased) clawOpen = !clawOpen;
            aReleased = false;
        }
        else{
            aReleased = true;
        }
        servoAngle = (clawOpen) ? RobotMap.SERVO_OPEN : RobotMap.SERVO_CLOSED;

        if (gamepad.b)servoAngle = RobotMap.SERVO_OPEN;

        servoAngle = safetyCheck(servoAngle);
        clawServo.setPosition(servoAngle);
   }

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(RobotMap.MINIMUM_SERVO_POSITION, out);
        out = Math.min(RobotMap.MAXIMUM_SERVO_POSITION, out);
        return out;
    }

       public void open(){
        clawServo.setPosition(RobotMap.SERVO_OPEN);
    }

    public void close(){
        clawServo.setPosition(RobotMap.SERVO_CLOSED);
    }

}
