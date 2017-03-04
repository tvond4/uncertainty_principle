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
@TeleOp(name="TeleOp6", group="Teleop")

public class UncertaintyPrinciple11453 extends OpMode{
    DcMotor mL1;
    DcMotor mR1;
    DcMotor mL2;
    DcMotor mR2;
    DcMotor mlaunch1;
    DcMotor mlaunch2;
    DcMotor elevator;
    DcMotor lift;
    Servo stop1;
    Servo stop2;
    Servo lift1;
    Servo lift2;
    double velocityX;
    double velocityY;
    double powerleft;
    double powerright;

    public void init() {

        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mR2 = hardwareMap.dcMotor.get("mR2");
        elevator = hardwareMap.dcMotor.get("elevator");
        lift = hardwareMap.dcMotor.get("lift");
        mlaunch1 = hardwareMap.dcMotor.get("launch1");
        mlaunch2 = hardwareMap.dcMotor.get("launch2");
        stop1 = hardwareMap.servo.get("stop1");
        stop2 = hardwareMap.servo.get("stop2");
        lift1 = hardwareMap.servo.get("lift1");
        lift2 = hardwareMap.servo.get("lift2");


        mL1.setDirection(DcMotor.Direction.FORWARD);
        mL2.setDirection(DcMotor.Direction.FORWARD);
        mR1.setDirection(DcMotor.Direction.REVERSE);
        mR2.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        //maybe change
        mlaunch1.setDirection(DcMotor.Direction.FORWARD);
        mlaunch2.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.REVERSE);

        velocityX = 0;
        velocityY = 0;
    }

    public void loop() {

      //  double power = 0;
       // double turn = 0;

       // power = gamepad1.left_stick_y;
       // turn = gamepad1.right_stick_x;

       // if (Math.abs(power) < .1) power = 0;
       // if (Math.abs(turn) < .1) turn = 0;

//        drive(power-turn,power+turn);
        //drive(power, power);
        if( gamepad1.left_stick_y>.2){
            drive(-1,-1);
        }
        else if( gamepad1.left_stick_y<-.2){
            drive(1,1);
        }
        else if( gamepad1.right_stick_x>.2){
            drive(-1,1);
        }
        else if( gamepad1.right_stick_x<-.2){
            drive(1,-1);
        }
        else if( gamepad1.dpad_up){
            drive(.5,.5);
        }
        else if( gamepad1.dpad_down){
            drive(-.5,-.5);
        }
        else if( gamepad1.dpad_right){
            drive(-.5,.5);
        }
        else if( gamepad1.dpad_left){
            drive(.5,-.5);
        }
        else{
            drive(0,0);
        }

//        powerright = gamepad1.left_stick_y;
//        powerleft = gamepad1.left_stick_y;
//
//        powerright -= 3*gamepad1.right_stick_x;
//        powerleft += 3*gamepad1.right_stick_x;
//
//        mL1.setPower(trim(powerleft));
//        mL2.setPower(trim(powerleft));
//        mR1.setPower(trim(powerright));
//        mR2.setPower(trim(powerright));


       // shoot(.5z); //long range


//        shoot(2.25);
        shoottest(1);

        if(gamepad2.left_bumper) {
            elevator(1);
        }

//        else if(gamepad2.right_bumper) {
//            elevator(.4);
//        }
        else if(gamepad2.x) {
            elevator(-1);
        }
        else elevator(0);

        if(gamepad2.a) {
            stop1.setPosition(.2);
            stop2.setPosition(.6);
        }
        else{
            stop1.setPosition(.6);
            stop2.setPosition(.1);
        }
        if(gamepad2.right_bumper){
            lift(-1);
        }
        else if(gamepad2.y){
            lift(.2);
        }
        else if(gamepad2.dpad_down) {
            lift(.7);
        }
        else{
            lift(0);
        }
        if(gamepad2.dpad_left) {
            lift1.setPosition(.5);
            lift2.setPosition(.5);
        }
        else{
            lift1.setPosition(0);
            lift2.setPosition(1);
        }


    }
//

//        else nom(0);

//        if(gamepad2.a){
//            powerservo = 1;
//            servoP(powerservo);
//            telemetry.addData("11ServoP", powerservo);
//            telemetry.update();
//            //debug statement, have it print servoP value
//        }
//        else if(gamepad2.b){
//            powerservo = -1;
//            servoP(powerservo);
//            telemetry.addData("22ServoP", powerservo);
//            telemetry.update();
//        }
//        else {
//            servoP.setPower(.1);
//            telemetry.addData("33ServoP", powerservo);
//            telemetry.update();
//        }

public double trim (double number) {
    if (number > 1) {
        number = 1;
    } else if (number < -1) {
        number = -1;
    }
    return number;
}
    void lift(double power) {
        lift.setPower(power);
    }
    void elevator(double power) {
        elevator.setPower(power);
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
    void shoottest(double power) {
        mlaunch1.setPower(power);
        mlaunch2.setPower(-power);
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