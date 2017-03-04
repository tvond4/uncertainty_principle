package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by student on 2/28/17.
 */

@Autonomous(name = "ReadBeacon", group = "automas")

public class ReadBeacon extends LinearOpMode {

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
    ColorSensor colorSensor1;    // Hardware Device Object
    ColorSensor colorSensor2;    // Hardware Device Object

    boolean color1;
    boolean color2;

    @Override
    public void runOpMode() throws InterruptedException {



        //Read Side 1
        //Read Side 2
        //if side one is red side one true; blue side false
        //if side two is red side one true; blue side false
        //if both true beacon Red
        //if both false beacon Blue
        //if side 1 true side two false //mixed red 1
        //if side 1 false side two true //mixed blue 1


        if ((readcolor1() == true)&&(readcolor2() == true)){
            telemetry.addData("Red Both", readcolor1());
            telemetry.update();
        }
        else if ((readcolor1() == false)&&(readcolor2() == false)){
            telemetry.addData("Blue Both", readcolor1());
            telemetry.update();
        }
        else if ((readcolor1() == true)&&(readcolor2() == false)){
            telemetry.addData("RED 1, Blue 2", readcolor1());
            telemetry.update();
        }
        else if ((readcolor1() == false)&&(readcolor2() == true)){
            telemetry.addData("Blue 1, Red 2", readcolor1());
            telemetry.update();
        }
        else{
            telemetry.addData("No Data", colorSensor1.blue());
            telemetry.update();
        }

    }
    public boolean readcolor1() {
        int red1 = 0;
        int blue1 = 0;
        while ((red1<20)&&(blue1<20)) {
            if (colorSensor1.red() > colorSensor1.blue()) {
               red1++;
            } else if (colorSensor1.red() < colorSensor1.blue()) {
                blue1++;
            }
        }
        if(red1>=20) color1 = true;
        else if(blue1>=20) color1 = false;
        return color1;
    }
    public boolean readcolor2() {
        int red2 = 0;
        int blue2 = 0;
        while ((red2<20)&&(blue2<20)) {
            if (colorSensor2.red() > colorSensor2.blue()) {
                red2++;
            } else if (colorSensor2.red() < colorSensor2.blue()) {
                blue2++;
            }
        }
        if(red2>=20) color2 = true;
        else if(blue2>=20) color2 = false;
        return color2;
    }
}
