package org.firstinspires.ftc.teamcode.AutoNew;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Consts;

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
    CRServo sideFrontWheel;
    CRServo sideBackWheel;

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
        sideFrontWheel = hardwareMap.crservo.get("side_front_wheel");
        sideBackWheel = hardwareMap.crservo.get("side_back_wheel");

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
        sleep(4000);
        sidePusher.setPower(0.0);
    }

    public void extendSidePusher() throws InterruptedException {
        sidePusher.setDirection(DcMotorSimple.Direction.REVERSE);
        sidePusher.setPower(1.0);
        sleep(4000);
        sidePusher.setPower(0.0);
    }

    public void engageSideWheelServo(CRServo servo) throws InterruptedException  {
        servo.setPower(1.0);
        sleep(2000);
        servo.setPower((servo.equals(sideBackWheel) ? Consts.SIDE_BACK_WHEEL_STOP_POWER : 0.0));
    }

    public void extendSideFrontWheel() throws InterruptedException {
        sideFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        engageSideWheelServo(sideFrontWheel);
    }

    public void retractSideFrontWheel() throws InterruptedException {
        sideFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        engageSideWheelServo(sideFrontWheel);
    }

    public void extendSideBackWheel() throws InterruptedException {
        sideBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        engageSideWheelServo(sideBackWheel);
    }

    public void retractSideBackWheel() throws InterruptedException {
        sideBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        engageSideWheelServo(sideBackWheel);
    }

    public void enableSideWheels() {
        sideFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        sideBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        sideFrontWheel.setPower(0.9);
        sideBackWheel.setPower(1.0);
    }

    public void holdSideWheels() {
        sideFrontWheel.setPower(0.0);
        sideBackWheel.setPower(Consts.SIDE_BACK_WHEEL_STOP_POWER);
    }

    public void extendSideWheels() throws InterruptedException {
        sideFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        sideBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        sideFrontWheel.setPower(0.9);
        sideBackWheel.setPower(1.0);
        sleep(2000);
        sideFrontWheel.setPower(0.0);
        sideBackWheel.setPower(Consts.SIDE_BACK_WHEEL_STOP_POWER);
    }

    public void retractSideWheels() throws InterruptedException {
        sideFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        sideBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        sideFrontWheel.setPower(0.9);
        sideBackWheel.setPower(1.0);
        sleep(2000);
        sideFrontWheel.setPower(0.0);
        sideBackWheel.setPower(Consts.SIDE_BACK_WHEEL_STOP_POWER);
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
        stop1.setPosition(.0);
        stop2.setPosition(.6);
    }

    public void closeStops() {
        stop1.setPosition(.55);
        stop2.setPosition(.15);
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
        while (i < 250) {
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
        sleep(200);

        elevator.setPower(1);
        sleep(2000);

        closeStops();
        //sleep(1000);

        elevator.setPower(0);
        disableFlywheels();
    }

/*
    ---------- 0 ----------
    |                     |
    |                     |
    |                     |
    |                     |
   90          o         -90
    |                     |
    |                     |
    |                     |
    |                     |
    ----(179)-180-(-179)---

 */

    public void turnToHeading(int targetHeading, float power, int threshold, boolean negativeSpin) throws InterruptedException {
        float currentHeading = getHeading();
        boolean turnLeft = (currentHeading > targetHeading);

        Log.i("turnToHeading", "Starting at: " + currentHeading);
        Log.i("turnToHeading", "Ending at: " + targetHeading);
        Log.i("turnToHeading", "Turn direction: " + (turnLeft ? "left" : "right"));

        while (Math.abs(currentHeading - targetHeading) > threshold) {
            turnLeft = (currentHeading > targetHeading);
            float distanceToTarget = Math.abs(currentHeading - targetHeading);

            double usedPower = power;
            if ((Math.abs(currentHeading - targetHeading)) < 10) {
                usedPower = power / 1.7;
            }

            if (usedPower < 0.4) {
                usedPower = 0.4;
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

    public void moveUntilCenterLine(float leftPower, float rightPower) throws InterruptedException {
        while (centerLine.getLightDetected() < ODS_WHITE_VALUE) {
            telemetry.addData("lights", centerLine.getLightDetected());
            telemetry.update();

            leftMotors(leftPower);
            rightMotors(rightPower);

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

    public void pressButton(boolean shouldTryAgain, int tryDir) throws InterruptedException {
        int blueNegativeFactor = (alliance == Alliance.RED ? 1 : -1);
        /*moveDistance_smooth(blueNegativeFactor * -300, 0.5f);
        sleep(1000);*/

        Alliance sideColor = getSideBeaconColor();

        if (sideColor == Alliance.UNKNOWN) {
            // unknown alliance
            /*if (shouldTryAgain) {
                // try again
                Log.i("Autonomous", "trying again");
                moveDistance(blueNegativeFactor * tryDir * 200, 0.5f);
                pressButton(false, tryDir);
            } else {
                // just give up then
            }*/
            return;
        }

        if (sideColor != alliance) {
            moveDistance_smooth(blueNegativeFactor * tryDir * -300, 0.5f);
        }
        extendSidePusher();
        sleep(100);
        retractSidePusher();
    }
}
