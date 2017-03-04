package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by student on 11/30/16.
 */
//@Autonomous(name="OneShot", group="automas")

public class ShootAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;
    DcMotor mlaunch1;
    DcMotor mlaunch2;
    DcMotor elevator;
    Servo stop1;
    Servo stop2;

    public void runOpMode() throws InterruptedException {
        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR2 = hardwareMap.dcMotor.get("mR2");
        mlaunch1 = hardwareMap.dcMotor.get("launch1");
        mlaunch2 = hardwareMap.dcMotor.get("launch2");
        elevator = hardwareMap.dcMotor.get("elevator");
        stop1 = hardwareMap.servo.get("stop1");
        stop2 = hardwareMap.servo.get("stop2");

        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);
        mR1.setDirection(DcMotor.Direction.FORWARD);
        mR2.setDirection(DcMotor.Direction.FORWARD);
        //maybe change
        mlaunch1.setDirection(DcMotor.Direction.FORWARD);
        mlaunch2.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        //start of program
        waitForStart();

        shoot(.3);
        sleep(2500);     // pause for servos to move
        stop1.setPosition(.2);         //open
        stop2.setPosition(.5);
        sleep(100);     // pause for servos to move
        elevator(-.5);
        sleep(400);
        stop1.setPosition(.6); //close
        stop2.setPosition(.2);
        sleep(400);
        elevator(0);
        shoot(0);
    }
    void shoot(double power) {
        mlaunch1.setPower(power);
        mlaunch2.setPower(-power);
    }

    void elevator(double power) {
        elevator.setPower(power);
    }
}
