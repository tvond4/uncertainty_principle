
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Autonomous(name="CapBallShot7", group="automas")

public class UPAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;
    DcMotor mlaunch1;
    DcMotor mlaunch2;
    DcMotor elevator;
    Servo stop1;
    Servo stop2;

    @Override
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

        mL1.setDirection(DcMotor.Direction.FORWARD);
        mL2.setDirection(DcMotor.Direction.FORWARD);
        mR1.setDirection(DcMotor.Direction.REVERSE);
        mR2.setDirection(DcMotor.Direction.REVERSE);

        mlaunch1.setDirection(DcMotor.Direction.FORWARD);
        mlaunch2.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Step 1:  Drive forward for 3 seconds
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
        mL1.setPower(0);
        mR1.setPower(0);
        mL2.setPower(0);
        mR2.setPower(0);
        sleep(2000);

        shoot(.175);
        sleep(2500);     // pause for servos to move
        stop1.setPosition(.2);         //open
        stop2.setPosition(.5);
        sleep(100);     // pause for servos to move
        elevator(-.4);
        sleep(400);
        elevator(0);
        stop1.setPosition(.6); //close
        stop2.setPosition(.2);

        sleep(2000);     // pause for servos to move
        elevator(-1);
        sleep(500);     // pause for servos to move
        elevator(0);

        sleep(2000);     // pause for servos to move
        stop1.setPosition(.2);         //open
        stop2.setPosition(.5);
        sleep(100);     // pause for servos to move
        elevator(-.4);
        sleep(1000);
        elevator(0);
        stop1.setPosition(.6); //close
        stop2.setPosition(.2);
        sleep(1000);

        shoot(.0);

        // Step 2:  Spin right for 1.3 seconds
        mL1.setPower(-1);
        mR1.setPower(1);
        mL2.setPower(-1);
        mR2.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        mL1.setPower(-.7);
        mR1.setPower(-.7);
        mL2.setPower(-.7);
        mR2.setPower(-.7);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }


        // Step 4:  Stop and close the claw.
        mL1.setPower(0);
        mR1.setPower(0);
        mL2.setPower(0);
        mR2.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        idle();
    }
    void shoot(double power) {
        mlaunch1.setPower(power);
        mlaunch2.setPower(-power);
    }
    void elevator(double power) {
        elevator.setPower(power);
    }

}
