package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "automasMRGyro00", group = "automas")

public class AutomasMRGyro extends LinearOpMode {

    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;

    ModernRoboticsI2cGyro gyro;   // Hardware Device Object

    public void runOpMode() throws InterruptedException {

        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState = false;

        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR2 = hardwareMap.dcMotor.get("mR2");

        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);
        mR1.setDirection(DcMotor.Direction.FORWARD);
        mR2.setDirection(DcMotor.Direction.FORWARD);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            curResetState = (gamepad1.a && gamepad1.b);
            if (curResetState && !lastResetState) {
                gyro.resetZAxisIntegrator();
            }
            lastResetState = curResetState;

            xVal = gyro.rawX();
            yVal = gyro.rawY();
            zVal = gyro.rawZ();
            heading = gyro.getHeading();
            angleZ = gyro.getIntegratedZValue();

            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);
            telemetry.update();
            telemetry.addData("done", heading);
            telemetry.update();

//            turnToHeading(.5, 100);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        driveTicksStraight(.4, 100, 0);
        turn(90);
    }
    void driveTicksStraight(double power, int ticks, double direction) {
        int startLeft = mL2.getCurrentPosition();
        int startRight = mR2.getCurrentPosition();
        int gyro1 = gyro.getIntegratedZValue(); //calls anglez
        double error = .1;
//        double initHeading = heading;
        if (!(direction == gyro.getIntegratedZValue())){
            turn(direction);
        }

        while ((Math.abs(mL2.getCurrentPosition()-startLeft) < ticks) || //if less than distance it's supposed to be
                (Math.abs(mR2.getCurrentPosition()-startRight) < ticks)) {

//            double error = gyro.getIntegratedZValue() - initHeading;
//            if (!(direction == 0)) {
//                if (direction >  gyro1) {
//                    drive(power + error, power - error);
//                } else if (direction <  gyro1) {
//                    drive(power - error, power + error);
//                }
//                else {
//                    drive(power, power);
//                    telemetry.addData("straight1", power);
//                    telemetry.update();
//                }
//            }
                if (direction >  gyro1+5) {
                    double drivepower = (direction-gyro1+10)/100; //set drivepower proportional to distance away from direction
                    drive(power + drivepower, power - drivepower);
                } else if (direction <  gyro1-5) {
                    double drivepower = (direction-gyro1+10)/100; //set drivepower proportional to distance away from direction
                    drive(power - drivepower, power + drivepower);
                }
                else {
                    drive(power, power);
                    telemetry.addData("straight", power);
                    telemetry.update();
                }
        }
//            pl-=error * error_const*s;
//            pr+=error * error_const*s;
//
//            telemetry.addData("Left Power", power);
//            telemetry.addData("Right Power", power);
//            telemetry.update();

//            pl = scale(pl);
//            pr = scale(pr);
//
//            drive(pl, pr);

        }
//        drive(0, 0)

    public void turn(double turndirection) {
        while ((turndirection > gyro.getIntegratedZValue()+10)||(turndirection < gyro.getIntegratedZValue()-10)){  //while turndirection is outside of anglez+-thresh(2)
            int gyro1 = gyro.getIntegratedZValue(); //calls anglez
            if (turndirection >= gyro1+10) { //if turnagle is greater than or = anglez+thres
                double drivepower = (turndirection-gyro1+50)/100; //set drivepower proportional to distance away from turndirection
                // Higher the "30" more power to the motors, less precise, Lower slower and more precise but can stall out motors
                drive(drivepower,-drivepower);
            } else if (turndirection <= gyro1-10) { //if turnagle is less than or = anglez+thres
                double drivepower = (gyro1-turndirection+50)/100; //set drivepower proportional to distance away from turndirection.
                // Higher the "30" more power to the motors, less precise, Lower slower and more precise but can stall out motors
                drive(-drivepower, drivepower);
            }
            else{ //if inside threshold stop driving break
                drive(0, 0);
                break;
            }
        }
    }

//            if (gyro.getIntegratedZValue()+180 > desiredHeading+180) {
//      /* might need a dead zone for turning... */
//            //Turn left until robot reaches the desiredHeading
//            while (gyro.getIntegratedZValue()+180 > desiredHeading+180) {
//                drive(-turnPower, turnPower);
//            }
//            drive(0, 0);
//        } else {
//            //Turn right until robot reaches the desiredHeading
//            while (gyro.getIntegratedZValue()+180 < desiredHeading+180) {
//                drive(turnPower, -turnPower);
//            }
//            drive(0, 0);
//        }
//    }

//
//    public void driveStraight(double power, double initHeading, int s) {
//        double error_const = .2;
//
//        //gyro is too finicky to do integral stuff so just the basic derivative stuff
//        double pl = power;
//        double pr = power;
//
//        double error = gyro.getIntegratedZValue() - initHeading;
//
//        pl-=error * error_const*s;
//        pr+=error * error_const*s;
//
//        pl = scale(pl);
//        pr = scale(pr);
//        telemetry.addData("Scale1", scale(pl));
//        telemetry.addData("Scale2", scale(pr));
//        telemetry.update();
//
//        drive(pl, pr);
//
//    }
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
    double scale(double d) {
        if (d > 1.0)
            return 1.0;
        else if (d < -1.0)
            return -1.0;
        else
            return d;
    }

//    public void gyroTurn (  double speed, double angle, double angleZ, double xVal, double yVal, double zVal)
//            throws InterruptedException {
//
//        while (opModeIsActive() && !onHeading(angle, angleZ)){
////        // keep looping while we are still active, and not on heading.
////        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, angleZ)) {
////            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//            idle();
//        }
//    }
//    boolean onHeading(double angle, double angleZ) {
//        double   error ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//        double turnspeed = .1;
//        error = getError(angle, angleZ);
//
//        // determine turn power based on +/- error
//        if (getError(angle, angleZ) == 0) {
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else if (getError(angle, angleZ) < 0){
//
//            rightSpeed  = -turnspeed;
//            leftSpeed   = turnspeed;
//        }
//        else {
//            rightSpeed  = turnspeed;
//            leftSpeed   = -turnspeed;
//        }
//
//        // Send desired speeds to motors.
//        mL1.setPower(leftSpeed);
//        mR1.setPower(rightSpeed);
//        mL2.setPower(leftSpeed);
//        mR2.setPower(rightSpeed);
//
//        // Display it for the driver.
//        telemetry.addData("Target1", "%5.2f", angle);
//        telemetry.addData("Err/St1", "%5.2f/%5.2f", error);
//        telemetry.addData("Speed1.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }
//    public double getError(double targetAngle, double angleZ) {
//
//        double robotError;
//
//        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - angleZ;
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
}