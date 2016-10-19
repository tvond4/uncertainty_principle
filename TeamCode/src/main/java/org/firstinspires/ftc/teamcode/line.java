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
 * Created by student on 10/13/16.
 */
@Autonomous(name = "line", group = "automas")

public class line extends LinearOpMode {
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
        driveTicksStraightline(.1, 90);
    }

    void driveTicksStraightline(double power, double direction) {
        int s = 1;
        int t = 0;
        telemetry.addData("inside", gyro.getIntegratedZValue());
        telemetry.update();

        while (!(rangeSensor.cmOptical() > 0)) {
            int gyro1 = gyro.getIntegratedZValue(); //calls anglez
            if ((colorSensor1.green() > 5) && ((direction < gyro1 + 5) && (direction > gyro1 - 5))) {
                telemetry.addData("on line", gyro1);
                telemetry.update();
                driveTicksStraight(power, 10, 90);
            } else if ((colorSensor1.green() > 5) && !((direction < gyro1 + 5) && (direction > gyro1 - 5))) {
                telemetry.addData("Yes No", gyro1);
                telemetry.update();
                turnticks(1, 70);
            }else if (!(colorSensor1.green() > 5) && ((direction < gyro1 + 5) && (direction > gyro1 - 5))) {
                telemetry.addData("Not Yes", gyro1);
                telemetry.update();
                turn(0);
            }
            else {
                telemetry.addData("Nope", gyro1);
                telemetry.update();
                if(colorSensor1.green() < 5){
//                    if(t==2){
//                        telemetry.addData("broken", t);
//                        telemetry.update();
//                        break;
//                    }
//                    else {
//                        if (s == 1) {
                            telemetry.addData("backingup", s);
                            telemetry.update();
                            driveTicksStraight(-.3, 400, gyro1);
//                        }
//                        else {
//                            telemetry.addData("forwarding", s);
//                            telemetry.update();
//                            driveTicksStraight(.3, 800, gyro1);
//                        }
//                    }
//                    s *= -1;
//                    t += 1;
                }
            }
        }
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
    public boolean findline(double power, int ticks, double direction) {
        while(colorSensor1.green() < 5){
            driveTicksStraight(.1, 100, direction);
            driveTicksStraight(-.1, 200, direction);
        }
        return true;
    }
        void driveTicksStraight(double power, int ticks, double direction) {
        int startLeft = mL2.getCurrentPosition();
        int startRight = mR2.getCurrentPosition();
        int gyro1 = gyro.getIntegratedZValue(); //calls anglez

        while ((Math.abs(mL2.getCurrentPosition() - startLeft) < ticks) || //if less than distance it's supposed to be
                (Math.abs(mR2.getCurrentPosition() - startRight) < ticks)) {
            gyro1 = gyro.getIntegratedZValue();
            double drivepower = (Math.abs(direction - gyro1) + powertodrivepowervariable(power)) / 100; //set drivepower proportional to distance away from direction
            if (direction > gyro1 + 5) {
                telemetry.addData("inside, right", gyro1);
                telemetry.addData("drivepower:", drivepower);
                telemetry.update();
                //  double drivepower = (direction-gyro1+10)/100; //set drivepower proportional to distance away from direction
                drive(power + drivepower, power - drivepower);
            } else if (direction < gyro1 - 5) {
                telemetry.addData("inside, left", gyro1);
                telemetry.addData("drivepower:", drivepower);
                telemetry.update();
                drive(power - drivepower, power + drivepower);
            } else {
                drive(power, power);
                telemetry.addData("straight", power);
                telemetry.update();
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

    public double powertodrivepowervariable(double power) {
        double variable = 00;
        variable = power * 100;
        return variable;
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
}