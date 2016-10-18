package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by student on 10/17/16.
 */
@Autonomous(name = "turnticks", group = "automas")

public class turnticks extends LinearOpMode {
    ColorSensor colorSensor1;    // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cGyro gyro;   // Hardware Device Object

    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;

    public void runOpMode() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        colorSensor1 = hardwareMap.colorSensor.get("color sensor1");

        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR2 = hardwareMap.dcMotor.get("mR2");

        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);
        mR1.setDirection(DcMotor.Direction.FORWARD);
        mR2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.addData("we ready boy", gyro.getIntegratedZValue());
        telemetry.addData("Botta start", gyro.getIntegratedZValue());
        telemetry.update();
        waitForStart();
        turnticks(1, 100);
    }
    void turnticks(double turndirection, double ticks) {
        int startLeft = mL2.getCurrentPosition();
        int startRight = mR2.getCurrentPosition();
        while ((Math.abs(mL2.getCurrentPosition() - startLeft) < ticks) || //if less than distance it's supposed to be
                (Math.abs(mR2.getCurrentPosition() - startRight) < ticks)) {
            if(turndirection == 1){
                drive(-.3,.3);
            }
            else{
                drive(.3,-.3);
            }
        }
    }
    void turn(double turndirection) {
        while ((turndirection > gyro.getIntegratedZValue() + 10) || (turndirection < gyro.getIntegratedZValue() - 10)) {  //while turndirection is outside of anglez+-thresh(2)
            int gyro1 = gyro.getIntegratedZValue(); //calls anglez
            if (turndirection >= gyro1 + 10) { //if turnagle is greater than or = anglez+thres
                double drivepower = (turndirection - gyro1 + 50) / 100; //set drivepower proportional to distance away from turndirection
                // Higher the "30" more power to the motors, less precise, Lower slower and more precise but can stall out motors
                drive(drivepower, -drivepower);
            } else if (turndirection <= gyro1 - 10) { //if turnagle is less than or = anglez+thres
                double drivepower = (gyro1 - turndirection + 50) / 100; //set drivepower proportional to distance away from turndirection.
                // Higher the "30" more power to the motors, less precise, Lower slower and more precise but can stall out motors
                drive(-drivepower, drivepower);
            } else { //if inside threshold stop driving break
                drive(0, 0);
                break;
            }
        }
    }
    void drive(double power1, double power2) {
        if (power1 > 1) power1 = 1;
        else if (power1 < -1) power1 = -1;
        if (power2 > 1) power2 = 1;
        else if (power2 < -1) power2 = -1;
        mL1.setPower(power1);
        mL2.setPower(power1);
        mR1.setPower(power2);
        mR2.setPower(power2);
    }
}