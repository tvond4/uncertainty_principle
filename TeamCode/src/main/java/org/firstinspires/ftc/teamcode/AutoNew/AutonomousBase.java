package org.firstinspires.ftc.teamcode.AutoNew;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

    public final double LEFT_PUSHER_UP_POSITION = .4;
    public final double LEFT_PUSHER_DOWN_POSITION = .55;
    public final double RIGHT_PUSHER_UP_POSITION = .65;
    public final double RIGHT_PUSHER_DOWN_POSITION = .3;

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
    CRServo sidePusher;
    CRServo sideWheel;

    ModernRoboticsI2cGyro gyro;
    ColorSensor beaconSideColor;
    OpticalDistanceSensor centerLine;

    Alliance alliance;

    public void initThings(HardwareMap hardwareMap, Alliance inAlliance) throws InterruptedException {
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
        sidePusher = hardwareMap.crservo.get("side_pusher");
        sideWheel = hardwareMap.crservo.get("side_wheel");

        // sensors
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("mrimu");
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        beaconSideColor = hardwareMap.colorSensor.get("beacon_side_color");
        beaconSideColor.setI2cAddress(I2cAddr.create8bit(0x4c));
        beaconSideColor.enableLed(false);

        centerLine = hardwareMap.opticalDistanceSensor.get("center_line");

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

    public void retractSidePusher() throws InterruptedException {
        sidePusher.setDirection(DcMotorSimple.Direction.FORWARD);
        sidePusher.setPower(1.0);
        sleep(2000);
        sidePusher.setPower(0.0);
    }

    public void extendSidePusher() throws InterruptedException {
        sidePusher.setDirection(DcMotorSimple.Direction.REVERSE);
        sidePusher.setPower(1.0);
        sleep(2000);
        sidePusher.setPower(0.0);
    }

    public void retractSideWheel() throws InterruptedException {
        sideWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        sideWheel.setPower(1.0);
        sleep(2000);
        sideWheel.setPower(0.0);
    }

    public void extendSideWheel() throws InterruptedException {
        sideWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        sideWheel.setPower(1.0);
        sleep(2000);
        sideWheel.setPower(0.0);
    }

    /*
     * shooting functions
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
        //Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        float thing = gyro.getHeading();
        if (thing > 180) {
            thing -= 360;
        }
        return thing * -1;
    }

    public Alliance getSideBeaconColor() throws InterruptedException {
        int i = 0;
        while (i < 500) {
            telemetry.addData("r", beaconSideColor.red());
            telemetry.addData("g", beaconSideColor.green());
            telemetry.addData("b", beaconSideColor.blue());
            telemetry.update();
            i++;
            Thread.sleep(1);
        }
        if (beaconSideColor.red() == 255 || beaconSideColor.blue() == 255 || beaconSideColor.red() == beaconSideColor.blue()) {
            return Alliance.UNKNOWN;
        } else if (beaconSideColor.red() > beaconSideColor.blue()) {
            return Alliance.RED;
        } else if (beaconSideColor.red() < beaconSideColor.blue()) {
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

    public void moveDistance_smooth(int distanceToMove, float power) throws InterruptedException {
        int targetDistance = mR2.getCurrentPosition() + distanceToMove;
        int distanceToTarget;
        boolean moveBack = targetDistance < mR2.getCurrentPosition();

        while (
                (!moveBack && mR2.getCurrentPosition() < targetDistance) ||
                        (moveBack && mR2.getCurrentPosition() > targetDistance)) {
            distanceToTarget = Math.abs(targetDistance - mR2.getCurrentPosition());

            double powerToUse = power;

            if (distanceToTarget < 200) {
                powerToUse = power / 1.5;
            }

            if (powerToUse < 0.3) {
                powerToUse = 0.3;
            }

            leftMotors((moveBack ? -powerToUse : powerToUse));
            rightMotors((moveBack ? -powerToUse : powerToUse));

            telemetry.addData("Encoder", mR2.getCurrentPosition());
            telemetry.update();

            idle();
        }

        leftMotors(0.0);
        rightMotors(0.0);
    }

    /*public void moveToDistance(float targetDistance, float power) throws InterruptedException {
        while (range.getDistance(DistanceUnit.INCH) > targetDistance) {
            leftMotors(power);
            rightMotors(power);

            idle();
        }

        leftMotors(0.0);
        rightMotors(0.0);
    }*/

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


    public void turnToHeading(int targetHeading, float power, int threshold, boolean negativeSpin) throws InterruptedException {
        float currentHeading = getHeading();
        boolean turnLeft = false;
        while (Math.abs(currentHeading - targetHeading) > threshold) {
            turnLeft = (currentHeading > targetHeading);
            float distanceToTarget = Math.abs(currentHeading - targetHeading);

            double usedPower = power;
            if ((Math.abs(currentHeading - targetHeading)) < 10) {
                usedPower = power / 1.7;
            }

            /*
            usedPower = distanceToTarget * 0.01;
            if (usedPower < 0.3) {
                userPower = 0.3;
            }
             */

            leftMotors(turnLeft ? -usedPower : (negativeSpin ? usedPower : 0));
            rightMotors(turnLeft ? (negativeSpin ? usedPower : 0) : -usedPower);

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

    public void turnUntilLine(float power, boolean turnLeft, OpticalDistanceSensor sensorToTest) throws InterruptedException {
        while (sensorToTest.getLightDetected() < ODS_WHITE_VALUE) {
            leftMotors((turnLeft ? power : -power));
            rightMotors((turnLeft ? -power : power));

            idle();
        }
    }

    /*public void moveBackUntilDistance(float power, int targetDistanceInInches) throws InterruptedException {
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
    }*/

    public void pressButton() throws InterruptedException {
        moveDistance_smooth(-300, 0.5f);
        sleep(1000);

        Alliance sideColor = getSideBeaconColor();

        if (sideColor == Alliance.UNKNOWN) {
            // unknown alliance, just give up then
            return;
        }

        if (sideColor != alliance) {
            moveDistance_smooth(500, 0.5f);
        }
        extendSidePusher();
        sleep(1000);
        retractSidePusher();
    }
}
