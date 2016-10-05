package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by student on 6/2/16.
 */
@Autonomous(name="automasTTVVV", group="automas")
public class AuTomas extends LinearOpMode {
    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;


    public void runOpMode() throws InterruptedException {
        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR2 = hardwareMap.dcMotor.get("mR2");
        telemetry.addData("You suck", mL1.getCurrentPosition());

        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        driveTicks(-.3, 1000);
    }
    public void drive(double power) {
        mL1.setPower(power);
        mR1.setPower(power);
        mL2.setPower(power);
        mR2.setPower(power);
    }

    void driveTicks(double power, int ticks) {
        int startLeft = mL2.getCurrentPosition();
        int startRight = mR2.getCurrentPosition();

        while ((Math.abs(mL2.getCurrentPosition()-startLeft) < ticks) ||
                (Math.abs(mR2.getCurrentPosition()-startRight) < ticks)) {
            drive(power);
            telemetry.update();

        }
        drive(0);
    }

    }