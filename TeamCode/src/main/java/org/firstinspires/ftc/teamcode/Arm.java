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
        zeroTheEncoder();
    }

    private void zeroTheEncoder() {
        encoderZero = encoderMotor.getCurrentPosition();

    }

    /**
     * Set the arm motor power for both left and right motors
     *
     * @param gamepad The gamepad from which to read joystick values
     */

    Boolean highSpeed = true;

    public void manual(Gamepad gamepad) {
        // toggle arm speed for latching
        if (gamepad.left_bumper) {
            highSpeed = false;
        }
        else if (gamepad.right_bumper){
            highSpeed = true;
        }

        double speedLimit;
        if (highSpeed) {
            speedLimit = RobotMap.HIGHSPEED_LIMIT_ARM;
        }
        else {
            speedLimit = RobotMap.LOWSPEED_ARM;
        }

        // Get joystick values from gamepad
        double power  = gamepad.left_stick_y;
        // Limit speed of drivetrain
        power *= speedLimit;

        //check if we want to go to a set position
        if (gamepad.left_trigger > 0.5) {
            double setPoint = RobotMap.ARM_CLIMB;
            double error = setPoint - encoderDegrees();
            power = RobotMap.kP * error;
            if (power > speedLimit) power = speedLimit;
            if (power < -speedLimit) power = -speedLimit;

        } else if (gamepad.right_trigger > 0.5){
            double setPoint = RobotMap.PICKUP_POSITION;
            double error = setPoint - encoderDegrees();
            power = RobotMap.kP * error;
            if (power > speedLimit) power = speedLimit;
            if (power < -speedLimit) power = - speedLimit;

        }
        else if (gamepad.right_stick_button) {
            double setPoint = RobotMap.LANDER_STORAGE_DROP;
            double error = setPoint - encoderDegrees();
            power = RobotMap.kP * error;
            if (power > speedLimit) power = speedLimit;
            if (power < -speedLimit) power = -speedLimit;
        }



        setPower(power);

        //output the encoder value//
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Arm Encoder", encoderDegrees());
        }

        //check to see if we want to 0 the encoder
        if (gamepad.y) {
            zeroTheEncoder();
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

    public void land() {

        double armSpeed = 0.4;
        double maxEncoderDegree = 290.0;
        setPower(armSpeed);
        while (encoderDegrees() < maxEncoderDegree) {
            telemetry.addData("Arm Encoder Land", encoderDegrees());
            telemetry.update();
        }
        setPower(0.0);
    }
    public void moveUp() {

        double armSpeed = -0.4;
        double maxEncoderDegree = 280.0;
        setPower(armSpeed);
        while (encoderDegrees() > maxEncoderDegree) {
            telemetry.addData("Arm Encoder Pull Up", encoderDegrees());
            telemetry.update();
        }
        setPower(0.0);
    }

    public void deployMarker(){

        double armSpeed = -0.4;
        double maxEncoderDegree = -170.0;
        zeroTheEncoder();
        setPower(armSpeed);
        while (encoderDegrees() > maxEncoderDegree) {
            telemetry.addData("Arm Encoder Deploy Marker", encoderDegrees());
            telemetry.update();
        }
        setPower(0.0);

    }

    public void depotRaise() {

        double armSpeed = 0.4;
        double maxEncoderDegree = 20.0;
       zeroTheEncoder();
        setPower(armSpeed);
        while (encoderDegrees() < maxEncoderDegree) {
            telemetry.addData("Arm Encoder Pull Up", encoderDegrees());
            telemetry.update();
        }
        setPower(0.0);
    }

}
