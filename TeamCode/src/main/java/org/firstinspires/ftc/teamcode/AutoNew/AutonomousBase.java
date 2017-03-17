package org.firstinspires.ftc.teamcode.AutoNew;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class AutonomousBase extends LinearOpMode {

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

    Alliance alliance;

    public void initThings(HardwareMap hardwareMap, Alliance inAlliance) {
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
        beaconLeftColor.enableLed(false);
        beaconRightColor = hardwareMap.colorSensor.get("beacon_right_color");
        beaconRightColor.enableLed(false);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        centerLine = hardwareMap.opticalDistanceSensor.get("center_line");
        leftLine = hardwareMap.opticalDistanceSensor.get("left_line");
        rightLine = hardwareMap.opticalDistanceSensor.get("right_line");

        alliance = inAlliance;
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
        shooting functions
     */
    public void engageFlywheels() {
        mlaunch1.setPower(0.8);
        mlaunch2.setPower(-0.8);
    }

    public void disableFlywheels() {
        mlaunch1.setPower(0);
        mlaunch2.setPower(0);
    }

    public void openStops() {
        stop1.setPosition(.2);
        stop2.setPosition(.6);
    }

    public void closeStops() {
        stop1.setPosition(.6);
        stop2.setPosition(.1);
    }

    /*
     * sensor functions
     */
    public float getHeading() {
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public Alliance getRightBeaconColor() throws InterruptedException {
        int i = 0;
        while (i < 500) {
            telemetry.addData("r", beaconRightColor.red());
            telemetry.addData("g", beaconRightColor.green());
            telemetry.addData("b", beaconRightColor.blue());
            telemetry.update();
            i++;
            Thread.sleep(1);
        }
        if (beaconRightColor.red() == 255 || beaconRightColor.blue() == 255 || beaconRightColor.red() == beaconRightColor.blue()) {
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
        int targetDistance = mR2.getCurrentPosition() + distanceToMove;
        boolean moveBack = targetDistance < mR2.getCurrentPosition();

        while (
                (!moveBack && mR2.getCurrentPosition() < targetDistance) ||
                (moveBack && mR2.getCurrentPosition() > targetDistance)) {
            leftMotors((moveBack ? -power : power));
            rightMotors((moveBack ? -power : power));

            telemetry.addData("Encoder", mR2.getCurrentPosition());
            telemetry.update();

            idle();
        }

        leftMotors(0.0);
        rightMotors(0.0);
    }

    public void moveToDistance(float targetDistance, float power) throws InterruptedException {
        while (range.getDistance(DistanceUnit.INCH) > targetDistance) {
            leftMotors(power);
            rightMotors(power);

            idle();
        }

        leftMotors(0.0);
        rightMotors(0.0);
    }

    public void shoot() throws InterruptedException {
        openStops();
        Thread.sleep(200);

        elevator.setPower(1);
        Thread.sleep(3000);

        closeStops();
        Thread.sleep(1000);

        elevator.setPower(0);
        disableFlywheels();
    }


    public void turnToHeading(int targetHeading, float power, int threshold) throws InterruptedException {
        float currentHeading = getHeading();
        boolean turnLeft = false;
        while (Math.abs(currentHeading - targetHeading) > threshold) {
            turnLeft = (currentHeading > targetHeading);

            /*if ((currentHeading < 0 && targetHeading > 0) || (currentHeading > 0 && targetHeading < 0)) {
                turnLeft = !turnLeft;
            }*/

            double usedPower = power;
            if ((Math.abs(currentHeading - targetHeading)) < 10) {
                usedPower = power / 1.5;
            }
//            if ((Math.abs(currentHeading - targetHeading)) < 6) {
//                usedPower = power / 2;
//            }

            leftMotors(turnLeft ? -usedPower : usedPower);
            rightMotors(turnLeft ? usedPower : -usedPower);

            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.update();

            currentHeading = getHeading();

            idle();
        }

        leftMotors(0.0);
        rightMotors(0.0);
    }

    public void turnToHeading_noScale(int targetHeading, float power) throws InterruptedException {
        float currentHeading = getHeading();
        boolean turnLeft = (currentHeading > targetHeading);
        while (
                (turnLeft && currentHeading > targetHeading) ||
                (!turnLeft && currentHeading < targetHeading)) {

            leftMotors(turnLeft ? -power : power);
            rightMotors(turnLeft ? power : -power);

            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.update();

            currentHeading = getHeading();

            idle();
        }

        leftMotors(0.0);
        rightMotors(0.0);
    }

    public void moveUntilCenterLine(float power) throws InterruptedException {
        while (centerLine.getLightDetected() < ODS_WHITE_VALUE) {
            telemetry.addData("lights", centerLine.getLightDetected());
            telemetry.update();

            leftMotors(power);
            rightMotors(power);

            idle();
        }

        leftMotors(0.0);
        rightMotors(0.0);
    }

    public void turnUntilSemiLine(float power, boolean turnLeft) throws InterruptedException {
        while (((rightLine.getLightDetected() < .04) && (rightLine.getLightDetected() > .06)) && ((leftLine.getLightDetected() < .04) && (leftLine.getLightDetected() > .06))) {
            leftMotors((turnLeft ? power : -power));
            rightMotors((turnLeft ? -power : power));

            idle();
        }
    }

    public void turnUntilLine(float power, boolean turnLeft, OpticalDistanceSensor sensorToTest) throws InterruptedException {
        while (sensorToTest.getLightDetected() < ODS_WHITE_VALUE) {
            leftMotors((turnLeft ? power : -power));
            rightMotors((turnLeft ? -power : power));

            idle();
        }
    }

    public void moveBackUntilDistance(float power, int targetDistanceInInches) throws InterruptedException {

        while (targetDistanceInInches - range.getDistance(DistanceUnit.INCH) > 0.5 && targetDistanceInInches - range.getDistance(DistanceUnit.INCH) > 0) {
            leftMotors(-power);
            rightMotors(-power);

            idle();
        }

        while (range.getDistance(DistanceUnit.INCH) - targetDistanceInInches > 0.5 && targetDistanceInInches - range.getDistance(DistanceUnit.INCH) < 0) {
            leftMotors(power);
            rightMotors(power);

            idle();
        }

        leftMotors(0.0f);
        rightMotors(0.0f);
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

            telemetry.addData("Left line value", leftLineValue);
            telemetry.addData("Right line value", rightLineValue);
            telemetry.update();

            idle();
        }
    }

    public void pressButton() throws InterruptedException {
        Alliance rightColor = getRightBeaconColor();

        if (rightColor == Alliance.UNKNOWN) {
            // unknown alliance, just go back then
            leftMotors(0.5f);
            rightMotors(0.5f);
            Thread.sleep(1000);
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

        Thread.sleep(400);

        // ram into wall
        leftMotors(-0.5f);
        rightMotors(-0.5f);
        Thread.sleep(1000);

        // and go back
        leftMotors(0.5f);
        rightMotors(0.5f);
        Thread.sleep(400);

        // and stop
        leftMotors(0.0f);
        rightMotors(0.0f);
    }
}
