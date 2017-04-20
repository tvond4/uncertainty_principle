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
        engageFlywheels();

        status("Running...");

        while (opModeIsActive()) {
            moveDistance(1400, 0.5f);
            sleep(1000);

            shoot();
            sleep(2000);

            turnToHeading((isRed ? 45 : 135), 0.45f, 1, true);
            sleep(1000);

            moveDistance(blueNegativeFactor * 3900, 0.4f);
            sleep(1000);

            turnToHeading((isRed ? 2 : 164), 0.5f, 1, true);
            sleep(2000);

            moveUntilCenterLine(blueNegativeFactor * 0.45f, blueNegativeFactor * 0.45f);
            sleep(2000);

            moveUntilCenterLine(blueNegativeFactor * -0.3f, blueNegativeFactor * -0.3f);
            sleep(2000);

            turnToHeading((isRed ? 0 : 177), 0.5f, 0, true);
            sleep(2000);

            pressButton();
            sleep(2000);

            requestOpModeStop();

            idle();
        }
    }
}
