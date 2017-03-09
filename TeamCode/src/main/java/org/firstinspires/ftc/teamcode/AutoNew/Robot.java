package org.firstinspires.ftc.teamcode.AutoNew;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Robot {
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
    Servo lift1;
    Servo lift2;

    BNO055IMU imu;

    LinearOpMode opMode;

    public Robot(HardwareMap hardwareMap, LinearOpMode inOpMode) {
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
        lift1 = hardwareMap.servo.get("lift1");
        lift2 = hardwareMap.servo.get("lift2");

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

        opMode = inOpMode;
    }

    public void leftMotors(double power) {
        mL1.setPower(power);
        mL2.setPower(power);
    }

    public void rightMotors(double power) {
        mR1.setPower(power);
        mR2.setPower(power);
    }

    public float getHeading() {
        return imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
    }

    public void turnToHeading(int targetHeading, float power) throws InterruptedException {
        float currentHeading = getHeading();
        boolean turnLeft = false;
        while (currentHeading != targetHeading) {
            turnLeft = (currentHeading > targetHeading);

            leftMotors(turnLeft ? power : -power);
            rightMotors(turnLeft ? -power : power);

            currentHeading = getHeading();

            opMode.idle();
        }
    }
}
