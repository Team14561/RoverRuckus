package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Class for controlling the drivetrain of an FTC robot.
 */
public class DriveTrain {

    // Class variables
    DcMotor leftMotor, rightMotor;
    BNO055IMU gyro;
    Telemetry telemetry;
    int leftZero, rightZero;
    double gyroZero = 0.0;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry   the robot instance of the telemetry object
     */
    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        leftMotor = hardwareMap.get(DcMotor.class, RobotMap.LEFT_DRIVE_MOTOR);
        rightMotor = hardwareMap.get(DcMotor.class, RobotMap.RIGHT_DRIVE_MOTOR);

        // Set the motor directions
        leftMotor.setDirection(RobotMap.LEFT_DRIVE_DIRECTION);
        rightMotor.setDirection(RobotMap.RIGHT_DRIVE_DIRECTION);

        //Set the encoder starting positions
        leftZero = leftMotor.getCurrentPosition();
        rightZero = rightMotor.getCurrentPosition();

        //Set up gyroscope

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        //gyroInit();

    }

    public void gyroInit(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(parameters);
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 10);

        gyroZero = gyro.getAngularOrientation().firstAngle;
    }


    /**
     * Set the drivetrain motor power for both left and right motors using the joystick values
     *
     * @param gamepad The gamepad from which to read joystick values
     */

    Boolean highSpeed = true;


    public void tankDrive(Gamepad gamepad) {

        if (gamepad.left_bumper) {
            highSpeed = false;

        }
        else if (gamepad.right_bumper){
            highSpeed = true;
        }


        double speedLimit;
        if (highSpeed) {
            speedLimit = RobotMap.HIGHSPEED_LIMIT;
        }
         else {
            speedLimit = RobotMap.LOWSPEED;
        }

        // Get joystick values from gamepad
        double leftPower = gamepad.left_stick_y;
        double rightPower = gamepad.right_stick_y;

        // Limit speed of drivetrain
        leftPower *= speedLimit;
        rightPower *= speedLimit;


        // Reverse joystick values if requested
        if (RobotMap.REVERSE_JOYSTICKS) {
            leftPower *= -1.0;
            rightPower *= -1.0;
        }

        setPower(leftPower, rightPower);

        // Output Encoder Values
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.addData("Right Encoder", rightEncoderInches());
            //telemetry.addData("Gyroscope", gyro.getAngularOrientation().firstAngle);
        }
    }

    private void setPower(double leftPower, double rightPower) {
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

    private double leftEncoderInches() {
        return (leftMotor.getCurrentPosition() - leftZero) / RobotMap.MOTOR_SCALE;
    }

    private double rightEncoderInches() {
        return (rightMotor.getCurrentPosition() - rightZero) / RobotMap.MOTOR_SCALE;
    }


    public void unhook() {
        double leftSpeed = 0.3;
        double rightSpeed = -0.5;
        double StartEncoder = rightEncoderInches();
        double endingEncoder = StartEncoder + 6.0; //12.0;
        setPower(leftSpeed, rightSpeed);
        while (rightEncoderInches() < endingEncoder) {
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower(0.0,0.0);
    }

    public void goldToDepot(){

        double leftSpeed = -0.7;
        double rightSpeed = -0.35;
        double StartEncoder = rightEncoderInches();
        double endingEncoder = StartEncoder + 24.0;
        setPower (leftSpeed, rightSpeed);
        while (rightEncoderInches() < endingEncoder) {
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);

      /*  leftSpeed = -0.5;
        rightSpeed = 0.5;
        StartEncoder = rightEncoderInches();
        endingEncoder = StartEncoder - 24.0;
        setPower (leftSpeed, rightSpeed);
        while (rightEncoderInches() > endingEncoder) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);

        leftSpeed = -0.5;
        rightSpeed = -0.5;
        StartEncoder = rightEncoderInches();
        endingEncoder = StartEncoder + 25.0;
        setPower (leftSpeed, rightSpeed);
        while (rightEncoderInches() < endingEncoder) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);
        */
    }

    public void backFromDepot() {
        double leftSpeed = 0.65;
        double rightSpeed = 0.65;
        double StartEncoder = rightEncoderInches();
        double endingEncoder = StartEncoder - 15.0;
        setPower(leftSpeed, rightSpeed);
        while (rightEncoderInches() > endingEncoder) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);
    }

    public void silverToDepot() {

        gyroInit();

        double leftSpeed = -0.5;
        double rightSpeed = -0.5;
        double StartEncoder = rightEncoderInches();
        double endingEncoder = StartEncoder + 30.0; //27.00
        setPower (leftSpeed, rightSpeed);
        while (rightEncoderInches() < endingEncoder) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);

        leftSpeed = 0.5;
        rightSpeed = -0.5;
        double endingGyro = gyroZero + 83.0; //50.0
        setPower (leftSpeed, rightSpeed);
        while(gyro.getAngularOrientation().firstAngle <  endingGyro){
            telemetry.addData("Gyroscope", gyro.getAngularOrientation().firstAngle);
            telemetry.addData("Left Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);


        leftSpeed = -0.5;
        rightSpeed = -0.5;
        StartEncoder = rightEncoderInches();
        endingEncoder = StartEncoder + 47.0;
        setPower (leftSpeed, rightSpeed);
        while (rightEncoderInches() < endingEncoder) {
            telemetry.addData("Gyroscope", gyro.getAngularOrientation().firstAngle);
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);

    }

    public void silverBackToCrater(){
        double leftSpeed = 0.65;
        double rightSpeed = 0.65;
        double StartEncoder = rightEncoderInches();
        double endingEncoder = StartEncoder - 47.0;
        setPower(leftSpeed, rightSpeed);
        while (rightEncoderInches() > endingEncoder) {
            telemetry.addData("Start", StartEncoder);
            telemetry.addData("Goal", endingEncoder);
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);

        leftSpeed = 0.2;
        rightSpeed = 0.2;
        StartEncoder = rightEncoderInches();
        endingEncoder = StartEncoder - 9.0;
        setPower(leftSpeed, rightSpeed);
        while (rightEncoderInches() > endingEncoder) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);
    }

    public void goldBackToCrater(){
        double leftSpeed = -0.5;
        double rightSpeed = 0.5;
        double StartEncoder = rightEncoderInches();
        double endingEncoder = StartEncoder - 14.0;
        setPower(leftSpeed, rightSpeed);
        while (rightEncoderInches() > endingEncoder) {
            telemetry.addData("Start", StartEncoder);
            telemetry.addData("Goal", endingEncoder);
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);

        leftSpeed = -0.6;
        rightSpeed = -0.65;
        StartEncoder = rightEncoderInches();
        endingEncoder = StartEncoder + 50.0;
        setPower(leftSpeed, rightSpeed);
        while (rightEncoderInches() < endingEncoder) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);

        leftSpeed = -0.3;
        rightSpeed = -0.2;
        StartEncoder = rightEncoderInches();
        endingEncoder = StartEncoder + 8.0;
        setPower(leftSpeed, rightSpeed);
        while (rightEncoderInches() < endingEncoder) {
            telemetry.addData("Left Encoder", leftEncoderInches());
            telemetry.addData("Right Encoder", rightEncoderInches());
            telemetry.update();
        }
        setPower ( 0.0, 0.0);
    }

}

