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

        int blueNegativeFactor = (getCurrentAlliance() == Alliance.RED ? 1 : -1);

        waitForStart();

        extendBoth();
        engageFlywheels();

        status("Running...");

        while (opModeIsActive()) {
            telemetry.update();

            moveDistance(1750, 0.5f);
            sleep(1000);

            shoot();
            sleep(1000);

            turnToHeading(blueNegativeFactor * -105, 0.5f, 3);
            sleep(1000);

            moveUntilCenterLine(-0.25f);
            sleep(1000);

            turnToHeading(blueNegativeFactor * -90, 0.5f, 1);
            sleep(1000);

            moveBackUntilDistance(-0.5f, 6);
            sleep(1000);

            pressButton();
            extendRight();
            sleep(3000);


            requestOpModeStop();

            idle();
        }
    }
}
