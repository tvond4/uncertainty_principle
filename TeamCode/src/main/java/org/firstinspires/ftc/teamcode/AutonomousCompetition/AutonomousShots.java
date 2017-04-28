package org.firstinspires.ftc.teamcode.AutonomousCompetition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousShots extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;
    DcMotor mlaunch1;
    DcMotor mlaunch2;
    DcMotor elevator;
    Servo stop1;
    Servo stop2;

    public void turn180() throws InterruptedException {
        mL1.setPower(.5);
        mR1.setPower(.5);
        mL2.setPower(.5);
        mR2.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        /*sideFrontWheel.setPower(Consts.SIDE_FRONT_WHEEL_STOP_POWER);
        sideBackWheel.setPower(Consts.SIDE_BACK_WHEEL_STOP_POWER);*/

        mL1.setPower(0);
        mR1.setPower(0);
        mL2.setPower(0);
        mR2.setPower(0);
    }
}
