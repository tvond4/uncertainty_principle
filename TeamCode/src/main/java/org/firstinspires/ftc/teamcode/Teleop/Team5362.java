package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 6/2/16.
 */
//@TeleOp(name="TeleOp5362", group="Teleop")

public class Team5362 extends OpMode{
    DcMotor drive1;
    DcMotor drive2;

    public void init() {

        drive1 = hardwareMap.dcMotor.get("drive1");
        drive2 = hardwareMap.dcMotor.get("drive2");

    }

    public void loop() {
        if(gamepad1.left_stick_y>.2){
            drive1.setPower(1);
        }
        else if(gamepad1.left_stick_y<-.2){
            drive1.setPower(-1);
        }
        else {
            drive1.setPower(0);
        }
        if( gamepad1.right_stick_y>.2){
            drive2.setPower(-1);
        }
        else if( gamepad1.right_stick_y<-.2){
            drive2.setPower(1);
        }
        else {
            drive2.setPower(0);
        }

    }

}