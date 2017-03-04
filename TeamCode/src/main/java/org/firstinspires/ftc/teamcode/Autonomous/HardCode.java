package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Autonomous(name = "Hard1", group = "automas")

public class HardCode extends LinearOpMode {
    ColorSensor colorSensor1;    // Hardware Device Object
    ModernRoboticsI2cRangeSensor rangeSensor;

    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;

    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;
    Servo servobutton;

    static final int LED_CHANNEL = 3;

    public void runOpMode() throws InterruptedException {

        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR2 = hardwareMap.dcMotor.get("mR2");

        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);
        mR1.setDirection(DcMotor.Direction.FORWARD);
        mR2.setDirection(DcMotor.Direction.FORWARD);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        servobutton = hardwareMap.servo.get("buttonPusher");
        sensorRGB = hardwareMap.colorSensor.get("color");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        colorSensor1 = hardwareMap.colorSensor.get("color sensor1");

        cdim.setDigitalChannelState(LED_CHANNEL, true);
        int michela = 1;
        waitForStart();
            drivesimple(.2, 210); //-300
            telemetry.addData("Donewithsimple", michela);
            telemetry.update();
         /*
        turnticks(1, 50);
        telemetry.addData("donewithturnticks", rangeSensor.cmOptical());
        telemetry.update();
        drivetowall();
        telemetry.addData("donewithwall", rangeSensor.cmOptical());
        telemetry.update();
        pushbutton();
        telemetry.addData("done", rangeSensor.cmOptical());
        telemetry.update();
//        drivesimple(.3, 210); //-300
//        turn(90);
//        driveTicksStraight(.1, 40, 90); //-200
//        pushbutton();
*/
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

    void drivetowall() {
        while(rangeSensor.cmOptical() > 0){
            telemetry.addData("cmoptical", rangeSensor.cmOptical());
            telemetry.update();
            drive(.1, .1);
        }
    }

    void pushbutton() {
        if (sensorRGB.blue() > sensorRGB.red()) {
            telemetry.addData("Button blue", sensorRGB.blue());
            telemetry.update();
            servobutton.setPosition(1);
        } else if (sensorRGB.blue() < sensorRGB.red()) {
            telemetry.addData("Button red", sensorRGB.red());
            telemetry.update();
            servobutton.setPosition(-.1);
        } else {
            telemetry.addData("nope", sensorRGB.red());
            telemetry.update();
            servobutton.setPosition(0);
        }
    }

    void drivesimple(double power, int ticks) {
        int startRight = mR2.getCurrentPosition();

        telemetry.addData("TicksR", startRight);
        telemetry.update();
        while (Math.abs(mR2.getCurrentPosition() - startRight) < ticks){ //if less than distance it's supposed to be
            drive(power, power);
            telemetry.addData("here: ", Math.abs(mR2.getCurrentPosition() - startRight));
            telemetry.addData("ticks", ticks);
            telemetry.update();

        }
        if(Math.abs(mR2.getCurrentPosition() - startRight) >= ticks) { //if less than distance it's supposed to be
            telemetry.addData("Donewithsimple", startRight);
            telemetry.update();
        }

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