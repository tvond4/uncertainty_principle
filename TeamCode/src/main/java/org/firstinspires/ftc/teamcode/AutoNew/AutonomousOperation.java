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
            moveDistance(1000, 0.5f);
            sleep(1000);

            //sleep(2000);

            turnToHeading(55, 0.4f, 1, true);
            sleep(1000);

            moveDistance(2700, 0.4f);
            sleep(1000);

            turnToHeading(45, 0.4f, 1, true);
            sleep(2000);

            moveDistance(200, 0.4f);
            sleep(2000);

            turnToHeading(2, 0.5f, 1, true);
            sleep(2000);

            moveUntilCenterLine(0.5f, 0.5f);
            sleep(2000);

            turnToHeading(0, 0.5f, 0, true);
            sleep(2000);

            pressButton();
            sleep(2000);

            requestOpModeStop();

            idle();
        }
    }
}
