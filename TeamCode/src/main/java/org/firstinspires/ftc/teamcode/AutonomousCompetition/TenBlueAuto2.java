package org.firstinspires.ftc.teamcode.AutonomousCompetition;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TenBLUE#2", group="automas")

public class TenBlueAuto2 extends LinearOpMode {

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
    Servo button1;
    Servo button2;

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
        button2 = hardwareMap.servo.get("button2");
        button1 = hardwareMap.servo.get("button1");

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


        shoottest(1);
        
        sleep(7000);

        button2.setPosition(.25);
        button1.setPosition(1);

        // Step 1:  Drive forward for 3 seconds
        mL1.setPower(.5);
        mR1.setPower(.5);
        mL2.setPower(.5);
        mR2.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        mL1.setPower(0);
        mR1.setPower(0);
        mL2.setPower(0);
        mR2.setPower(0);
        sleep(2000);

        stop1.setPosition(.2);         //open
        stop2.setPosition(.6);
        sleep(200);     // pause for servos to move
        elevator(-1);
        sleep(4000);
        stop1.setPosition(.6); //close
        stop2.setPosition(.1);
        sleep(1000);
        elevator(0);
        shoot(0);

        mL1.setPower(.5);
        mR1.setPower(.5);
        mL2.setPower(.5);
        mR2.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
        // Step 2:  Spin right for 1.3 seconds
        mL1.setPower(-1);
        mR1.setPower(1);
        mL2.setPower(-1);
        mR2.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        mL1.setPower(-.7);
        mR1.setPower(-.7);
        mL2.setPower(-.7);
        mR2.setPower(-.7);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .4)) {
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
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
//        telemetry.addData("voltage", voltage);
        //read voltage, set power equal to a constant voltage
        //always send two volts
        //2/battery = power
        double powerset = power/voltage;
//        telemetry.addData("powerset", powerset);
//        telemetry.update();
        mlaunch1.setPower(powerset);
        mlaunch2.setPower(-powerset);
    }
    void elevator(double power) {
        elevator.setPower(power);
    }
    void shoottest(double power) {
        mlaunch1.setPower(power);
        mlaunch2.setPower(-power);
    }
}
