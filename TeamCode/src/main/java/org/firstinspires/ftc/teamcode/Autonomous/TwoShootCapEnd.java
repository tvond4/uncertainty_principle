package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by student on 11/30/16.
 */
//@Autonomous(name="TwoShotCapEnd", group="automas")

public class TwoShootCapEnd extends LinearOpMode {

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

    //encoder variables
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


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
        //maybe change
        mlaunch1.setDirection(DcMotor.Direction.FORWARD);
        mlaunch2.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        mL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        mL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //start of program
        waitForStart();



        encoderDrive(.3,  5,  5, 3);  // drive forward initial

//        //doubleshot
//        fullshot();
//        fullshot();

//        shoot(0);

//        encoderDrive(.5,2,2,2);

//        turn180();


//        encoderDrive(-.5,2,2,2);

        sleep(10000);     // pause for servos to move

//        OTHER CODE
//        sleep(1000);     // ball 2 pause for servos to move
//        stop1.setPosition(.2);
//        stop2.setPosition(.5);
//        sleep(500);     // pause for servos to move
//        //close
//        stop1.setPosition(.6);
//        stop2.setPosition(.2);
//        sleep(1000);     // pause for servos to move
//        elevator(0);


//        shoot(0);
//        encoderDrive(.5,  10,  10, 30);  // S1: Forward 47 Inches with 5 Sec timeout
//        drive(1, -1);
//            turnticks(1, 500);
//        turn(180); //turn 180˚
//        encoderDrive(.5,  5,  5, 10);  // move on platform
    }
    void shoot(double power) {
        mlaunch1.setPower(power);
        mlaunch2.setPower(-power);
    }

    void elevator(double power) {
        elevator.setPower(power);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget1;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = mL2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = mR2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget1 = mL1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget2 = mR1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            mL2.setTargetPosition(newLeftTarget);
            mR2.setTargetPosition(newRightTarget);
            mL1.setTargetPosition(newLeftTarget1);
            mR1.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            mL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            mL2.setPower(Math.abs(speed));
            mR2.setPower(Math.abs(speed));
            mL1.setPower(Math.abs(speed));
            mR1.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mL2.isBusy() && mR2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        mL2.getCurrentPosition(),
                        mR2.getCurrentPosition());
                telemetry.addData("Path3",  "Running at %7d :%7d",
                        mL1.getCurrentPosition(),
                        mR1.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            mL2.setPower(0);
            mR2.setPower(0);
            mL1.setPower(0);
            mR1.setPower(0);

            // Turn off RUN_TO_POSITION
            mL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
//    void turnticks(double turndirection, double ticks) {
//        int startLeft = mL2.getCurrentPosition();
//        int startRight = mR2.getCurrentPosition();
//        int startLeft2 = mL1.getCurrentPosition();
//        int startRight2 = mR1.getCurrentPosition();
//        while (((Math.abs(mL2.getCurrentPosition() - startLeft) < ticks) || //if less than distance it's supposed to be
//                (Math.abs(mR2.getCurrentPosition() - startRight) < ticks) || //if less than distance it's supposed to be
//                (Math.abs(mL1.getCurrentPosition() - startLeft2) < ticks) || //if less than distance it's supposed to be
//                (Math.abs(mR1.getCurrentPosition() - startRight2) < ticks)) && opModeIsActive()){
//            if(turndirection == 1){
//                drive(-1,1);
//            }
//            else{
//                drive(1,-1);
//            }
//        }
//    }
    void drive(double power1, double power2){
        if(power1 > 1) power1= 1;
        else if (power1<-1) power1 = -1;
        if(power2 > 1) power2= 1;
        else if (power2<-1) power2 = -1;
        mL1.setPower(-power1);
        mL2.setPower(-power1);
        mR1.setPower(-power2);
        mR2.setPower(-power2);
    }

//    void turn180() {
//        int startDir = gyro.getIntegratedZValue();
//
//        while (Math.abs(gyro.getIntegratedZValue() - (startDir+180)) < 10){
//            drive(-.3,.3);
//            telemetry.addData("AngleZ", gyro.getIntegratedZValue());
//            telemetry.update();
//        }
//    }

    void fullshot() {
        try{
            shoot(.275);
            sleep(2500);     // pause for servos to move
            stop1.setPosition(.2);         //open
            stop2.setPosition(.5);
            sleep(100);     // pause for servos to move
            elevator(-.5);
            sleep(400);
            elevator(0);
            stop1.setPosition(.6); //close
            stop2.setPosition(.2);
        }catch(InterruptedException e){
            telemetry.addData("something went wrong: ", e);
            updateTelemetry(telemetry);
        }
    }
}
