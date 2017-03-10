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

        robot = new Robot(hardwareMap, this, getCurrentAlliance());

        status("Ready to go!");

        int blueNegativeFactor = (getCurrentAlliance() == Alliance.RED ? 1 : -1);

        waitForStart();
        runtime.reset();
        robot.extendRight();

        status("Running...");

        while (opModeIsActive()) {
            telemetry.update();

            robot.moveDistance(1000, 0.5f);
            sleep(3000);

            robot.turnToHeading(blueNegativeFactor * 90, 0.5f);
            sleep(3000);

            robot.moveUntilCenterLine(0.5f);
            sleep(3000);

            robot.turnUntilLine(0.5f, true, robot.leftLine);
            sleep(3000);

            robot.lineFollow();
            sleep(3000);

            robot.pressButton();
            robot.extendRight();
            sleep(3000);

            robot.turnToHeading(0, 0.5f);
            sleep(3000);

            robot.moveUntilCenterLine(0.5f);
            sleep(3000);

            requestOpModeStop();

            idle();
        }
    }
}
