package org.firstinspires.ftc.teamcode.AutoNew;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousOperation extends LinearOpMode {

    public abstract Alliance getCurrentAlliance();

    private ElapsedTime runtime = new ElapsedTime();

    public Robot robot;

    public void status(String str) {
        telemetry.addData("Status", str);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        status("Starting robot....");

        robot = new Robot(hardwareMap, this);

        status("Ready to go!");

        int blueNegativeFactor = (getCurrentAlliance() == Alliance.RED ? 1 : -1);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.update();

            robot.turnToHeading(blueNegativeFactor * 90, 0.5f);

            idle();
        }
    }
}
