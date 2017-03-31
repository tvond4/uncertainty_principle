package org.firstinspires.ftc.teamcode.AutoNew;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutonomousOperation extends AutonomousBase {

    public abstract Alliance getCurrentAlliance();

    public void status(String str) {
        telemetry.addData("Status", str);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        status("Starting robot....");

        initThings(hardwareMap, getCurrentAlliance());

        status("Ready to go!");

        boolean isRed = getCurrentAlliance() == Alliance.RED;
        int blueNegativeFactor = (isRed ? 1 : -1);

        waitForStart();

        status("Running...");

        while (opModeIsActive()) {
            /*extendBoth();
            sleep(2000);

            retractBoth();
            sleep(2000);

            extendLeft();
            sleep(2000);

            extendRight();
            sleep(2000);

            if (true) {
                return;
            }*/

            telemetry.update();

            moveDistance_smooth(200, 0.4f);
            sleep(1000);

            turnToHeading(blueNegativeFactor * 28, 0.6f, 2, true);
            sleep(1000);

            moveUntilCenterLine(-0.25f);
            sleep(1000);

            moveDistance(100, 0.4f);
            sleep(1000);

            turnToHeading(blueNegativeFactor * 90, 0.6f, 1, false);
            sleep(1000);

            moveDistance(200, 0.4f);
            sleep(1000);

            extendBoth();
            sleep(1000);

            Alliance rightColor = getRightBeaconColor();
            if (rightColor == Alliance.UNKNOWN) {
                telemetry.addLine("Failed to get alliance!!!!");
                telemetry.update();
                Thread.sleep(2000);
            } else if (rightColor == alliance) {
                extendRight();

                Thread.sleep(2000);

                rightMotors(-0.5f);
                leftMotors(-0.3f);

                Thread.sleep(2000);
            } else {
                extendLeft();

                Thread.sleep(2000);

                rightMotors(-0.3f);
                leftMotors(-0.5f);

                Thread.sleep(2000);
            }

            /*moveDistance(-2100, 0.5f);
            sleep(1000);

            turnToHeading(blueNegativeFactor * -170, 0.6f, 2);
            sleep(1000);

            moveUntilCenterLine(-0.25f);
            sleep(1000);

            moveDistance(-150, 0.5f);
            sleep(1000);

            turnToHeading((isRed ? -95 : 90), 0.6f, 1);
            sleep(1000);

            leftMotors(0.5f);
            rightMotors(0.5f);
            sleep(400);
            leftMotors(0.0f);
            rightMotors(0.0f);

            sleep(3000);*/


            requestOpModeStop();

            idle();
        }
    }
}
