package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by student on 3/1/17.
 */
public class OneSensorLineFollowing extends LinearOpMode {

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

    OpticalDistanceSensor lineSensor1;  // Hardware Device Object

    double drive = .4;
    double TurnCoe = .2;

    @Override
    public void runOpMode() throws InterruptedException {
        //if white turn right
        //if black turn left
//
//        if (Sees light){
//            drive(drive-TurnCoe, drive+TurnCoe);
//        }
//        else if (No Light){
//            drive(drive+TurnCoe, drive-TurnCoe);
//        }
//        lineSensor1.getLightDetected();
//
        }

    void drive(double power1, double power2){
        if(power1 > 1) power1= 1;
        else if (power1<-1) power1 = -1;
        if(power2 > 1) power2= 1;
        else if (power2<-1) power2 = -1;
        mL1.setPower(power1);
        mL2.setPower(power1);
        mR1.setPower(power2);
        mR2.setPower(power2);
    }
}
