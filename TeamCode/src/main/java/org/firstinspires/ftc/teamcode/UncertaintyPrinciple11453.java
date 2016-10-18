package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by student on 6/2/16.
 */
@TeleOp(name="TeleOp", group="Teleop")
public class UncertaintyPrinciple11453 extends OpMode{
    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;
    public void init() {
        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR2 = hardwareMap.dcMotor.get("mR2");

        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);
        mR1.setDirection(DcMotor.Direction.FORWARD);
        mR2.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() {

        double power = 00;
        double turn = 00;
        power = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;

        telemetry.addData("power", power);
        telemetry.addData("turn", turn);
        telemetry.update();

        if (Math.abs(power) < .1) power = 0;
        if (Math.abs(turn) < .1) turn = 0;

        drive(power-turn,power+turn);

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