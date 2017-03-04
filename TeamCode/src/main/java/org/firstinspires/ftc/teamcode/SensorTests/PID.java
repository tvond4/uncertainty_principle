package org.firstinspires.ftc.teamcode.SensorTests;

        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

//@Autonomous(name = "straight", group = "automas")
/**
 * Created by student on 10/11/16.
 */
public class PID extends LinearOpMode {
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

        //your moves
        driveTicksStraight(.4, 100, 90);
    }
    void driveTicksStraight(double power, int ticks, double direction) {
        int startLeft = mL2.getCurrentPosition();
        int startRight = mR2.getCurrentPosition();
        int gyro1;

        if (!(direction == gyro.getIntegratedZValue())){
            turn(direction);
        }

        while ((Math.abs(mL2.getCurrentPosition()-startLeft) < ticks) || //if less than distance it's supposed to be
                (Math.abs(mR2.getCurrentPosition()-startRight) < ticks)) {
            gyro1 = gyro.getIntegratedZValue();
            double drivepower = (Math.abs(direction-gyro1)+powertodrivepowervariable(power))/100; //set drivepower proportional to distance away from direction
            if (direction > gyro1+5) {
                drive(power + drivepower, power - drivepower);
            }
            else if (direction < gyro1-5) {
                drive(power - drivepower, power + drivepower);
            }
            else {
                drive(power, power);
            }
        }
    }
    public double powertodrivepowervariable(double power) {
        double variable = 00;
        variable = power*100;
        return variable;
    }

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
