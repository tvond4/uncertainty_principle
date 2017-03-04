package org.firstinspires.ftc.teamcode.SensorTests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by student on 10/27/16.
 */

//    @Autonomous(name="Encoder12", group="automas")
    public class EncoderDrive extends LinearOpMode {

        /* Declare OpMode members. */
        private ElapsedTime runtime = new ElapsedTime();
    ColorSensor sensorRGB;
    Servo servobutton;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED             = 0.6;
        static final double     TURN_SPEED              = 0.5;

        DcMotor mL1;
        DcMotor mR1;
        DcMotor mL2;
        DcMotor mR2;

        @Override
        public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

            mL1 = hardwareMap.dcMotor.get("mL1");
            mR1 = hardwareMap.dcMotor.get("mR1");
            mL2 = hardwareMap.dcMotor.get("mL2");
            mR2 = hardwareMap.dcMotor.get("mR2");
            sensorRGB = hardwareMap.colorSensor.get("color");
            servobutton = hardwareMap.servo.get("buttonPusher");

            mL1.setDirection(DcMotor.Direction.REVERSE);
            mL2.setDirection(DcMotor.Direction.REVERSE);
            mR1.setDirection(DcMotor.Direction.FORWARD);
            mR2.setDirection(DcMotor.Direction.FORWARD);


            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            mL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();

            mL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0",  "Starting at %7d :%7d",
                    mL2.getCurrentPosition(),
                    mR2.getCurrentPosition());
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(DRIVE_SPEED,  17,  17, 10);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED,   5.5, -5.5, 15); //5.5 // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED,  10, 10, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
            sleep(1000);     // pause for servos to move
            pushbutton();
            sleep(1000);
            // pause for servos to move

            while (opModeIsActive()) {
                mL2.setPower(1);
                mR2.setPower(1);
                mL1.setPower(1);
                mR1.setPower(1);
                sleep(500);
                mL2.setPower(0);
                mR2.setPower(0);
                mL1.setPower(0);
                mR1.setPower(0);
                sleep(250);
                mL2.setPower(-1);
                mR2.setPower(-1);
                mL1.setPower(-1);
                mR1.setPower(-1);
                sleep(500);
                mL2.setPower(0);
                mR2.setPower(0);
                mL1.setPower(0);
                mR1.setPower(0);
                sleep(250);
            }

            encoderDrive(.2,  2, 2, 22.0);  // S3: Reverse 24 Inches with 4 Sec timeout
            encoderDrive(-.2,  2, 2, 24.0);  // S3: Reverse 24 Inches with 4 Sec timeout
            encoderDrive(.2,  2, 2, 22.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//            drivetowal();

            sleep(1000);     // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    void pushbutton() {
        if (sensorRGB.blue() > sensorRGB.red()) {
            telemetry.addData("Button blue", sensorRGB.blue());
            telemetry.update();
            servobutton.setPosition(-1);//1
        } else if (sensorRGB.blue() < sensorRGB.red()) {
            telemetry.addData("Button red", sensorRGB.red());
            telemetry.update();
            servobutton.setPosition(1);//.1
        } else {
            telemetry.addData("nope", sensorRGB.red());
            telemetry.update();
            servobutton.setPosition(0);
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

        /*
         *  Method to perfmorm a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the opmode running.
         */
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
//    public void  drivetowal() throws InterruptedException {
//        while (rangeSensor.rawUltrasonic() > 15) {
//            encoderDrive(DRIVE_SPEED, .5, .5, 5.0);
//        }
//        drive(0, 0);
//    }
}


