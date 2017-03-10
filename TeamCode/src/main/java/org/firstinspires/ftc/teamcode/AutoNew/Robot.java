package org.firstinspires.ftc.teamcode.AutoNew;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Robot {

    public static final double ODS_WHITE_VALUE = 0.1f; // TODO: find this
    public static final double DISTANCE_TO_WALL_FOR_BEACON = 8.0f; // TODO: check this value (it's in inches)

    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;
    DcMotor mlaunch1;
    DcMotor mlaunch2;
    DcMotor elevator;
    DcMotor lift;

    Servo stop1;
    Servo stop2;
    Servo leftButtonPusher;
    Servo rightButtonPusher;

    BNO055IMU imu;
    ColorSensor beaconLeftColor;
    ColorSensor beaconRightColor;
    ModernRoboticsI2cRangeSensor range;
    OpticalDistanceSensor centerLine;
    OpticalDistanceSensor leftLine;
    OpticalDistanceSensor rightLine;

    LinearOpMode opMode;
    Alliance alliance;

    public Robot(HardwareMap hardwareMap, LinearOpMode inOpMode, Alliance inAlliance) {
        // motors
        mL1 = hardwareMap.dcMotor.get("mL1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mR2 = hardwareMap.dcMotor.get("mR2");
        elevator = hardwareMap.dcMotor.get("elevator");
        lift = hardwareMap.dcMotor.get("lift");
        mlaunch1 = hardwareMap.dcMotor.get("launch1");
        mlaunch2 = hardwareMap.dcMotor.get("launch2");

        mL1.setDirection(DcMotor.Direction.FORWARD);
        mL2.setDirection(DcMotor.Direction.FORWARD);
        mR1.setDirection(DcMotor.Direction.REVERSE);
        mR2.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        mlaunch1.setDirection(DcMotor.Direction.FORWARD);
        mlaunch2.setDirection(DcMotor.Direction.FORWARD);

        // servos
        stop1 = hardwareMap.servo.get("stop1");
        stop2 = hardwareMap.servo.get("stop2");
        leftButtonPusher = hardwareMap.servo.get("button1");
        rightButtonPusher = hardwareMap.servo.get("button2");

        // sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        beaconLeftColor = hardwareMap.colorSensor.get("beacon_left_color");
        beaconLeftColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        beaconRightColor = hardwareMap.colorSensor.get("beacon_right_color");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        centerLine = hardwareMap.opticalDistanceSensor.get("center_line");
        leftLine = hardwareMap.opticalDistanceSensor.get("left_line");
        rightLine = hardwareMap.opticalDistanceSensor.get("right_line");

        opMode = inOpMode;
        alliance = inAlliance;
    }

    public void idle() throws InterruptedException {
        opMode.idle();
    }

    /*
     * movement functions
     */
    public void leftMotors(double power) {
        mL1.setPower(power);
        mL2.setPower(power);
    }

    public void rightMotors(double power) {
        mR1.setPower(power);
        mR2.setPower(power);
    }

    public void retractBoth() throws InterruptedException {
        leftButtonPusher.setPosition(.4);
        rightButtonPusher.setPosition(.65);
        Thread.sleep(300);
    }

    public void extendLeft() throws InterruptedException {
        leftButtonPusher.setPosition(1);
        rightButtonPusher.setPosition(.65);
        Thread.sleep(300);
    }

    public void extendRight() throws InterruptedException {
        leftButtonPusher.setPosition(.4);
        rightButtonPusher.setPosition(.25);
        Thread.sleep(300);
    }

    public void extendBoth() throws InterruptedException {
        leftButtonPusher.setPosition(1);
        rightButtonPusher.setPosition(.25);
        Thread.sleep(300);
    }

    /*
     * sensor functions
     */
    public float getHeading() {
        return imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
    }

    public Alliance getRightBeaconColor() {
        if (beaconRightColor.red() == 255 || beaconRightColor.blue() == 255) {
            return Alliance.UNKNOWN;
        } else if (beaconRightColor.red() > beaconRightColor.blue()) {
            return Alliance.RED;
        } else if (beaconRightColor.red() < beaconRightColor.blue()) {
            return Alliance.BLUE;
        } else {
            return Alliance.UNKNOWN;
        }
    }

    /*
     * higher-level functions
     */
    public void moveDistance(int distanceToMove, float power) throws InterruptedException {
        int targetDistance = mL1.getCurrentPosition() + distanceToMove;
        boolean moveBack = targetDistance < mL1.getCurrentPosition();

        while ((!moveBack && mL1.getCurrentPosition() < targetDistance) || (moveBack && mL1.getCurrentPosition() > targetDistance)) {
            leftMotors((moveBack ? -power : power));
            rightMotors((moveBack ? -power : power));

            idle();
        }
    }

    public void turnToHeading(int targetHeading, float power) throws InterruptedException {
        float currentHeading = getHeading();
        boolean turnLeft = false;
        while (currentHeading != targetHeading) {
            turnLeft = (currentHeading > targetHeading);

            leftMotors(turnLeft ? power : -power);
            rightMotors(turnLeft ? -power : power);

            currentHeading = getHeading();

            idle();
        }
    }

    public void moveUntilCenterLine(float power) throws InterruptedException {
        while (centerLine.getLightDetected() < ODS_WHITE_VALUE) {
            leftMotors(power);
            rightMotors(power);

            idle();
        }
    }

    public void turnUntilLine(float power, boolean turnLeft, OpticalDistanceSensor sensorToTest) throws InterruptedException {
        while (sensorToTest.getLightDetected() < ODS_WHITE_VALUE) {
            leftMotors((turnLeft ? power : 0.0f));
            rightMotors((turnLeft ? 0.0f : power));

            idle();
        }
    }

    public void lineFollow() throws InterruptedException {
        while (range.getDistance(DistanceUnit.INCH) > DISTANCE_TO_WALL_FOR_BEACON) {
            double leftLineValue = leftLine.getLightDetected();
            double rightLineValue = rightLine.getLightDetected();

            // TODO: verify these values (I took them from the code for Standard's robot)
            if (0.005 < leftLineValue && leftLineValue < 0.07 && 0.005 < rightLineValue && rightLineValue < 0.07) {
                leftMotors(0.5f);
                rightMotors(0.5f);
            } else if (leftLineValue > rightLineValue) {
                leftMotors(0.5f);
                rightMotors(0.0f);
            } else if (rightLineValue > leftLineValue) {
                leftMotors(0.0f);
                rightMotors(0.5f);
            } else {
                leftMotors(0.0f);
                rightMotors(0.0f);
            }

            opMode.telemetry.addData("Left line value", leftLineValue);
            opMode.telemetry.addData("Right line value", rightLineValue);
            opMode.telemetry.update();

            idle();
        }
    }

    public void pressButton() throws InterruptedException {
        Alliance rightColor = getRightBeaconColor();

        if (rightColor == Alliance.UNKNOWN) {
            // unknown alliance, just go back then
            leftMotors(-0.5f);
            rightMotors(-0.5f);
            Thread.sleep(3000);
            leftMotors(0.0f);
            rightMotors(0.0f);
            return;
        }

        // extend the correct arm
        if (rightColor == alliance) {
            extendRight();
        } else {
            extendLeft();
        }

        // ram into wall
        leftMotors(0.5f);
        rightMotors(0.5f);
        Thread.sleep(3000);

        // and go back
        leftMotors(-0.5f);
        rightMotors(-0.5f);
        Thread.sleep(3000);

        // and stop
        leftMotors(0.0f);
        rightMotors(0.0f);
    }
}
