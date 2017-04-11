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
            moveUntilCenterLine(0.5f);
            sleep(1000);

            pressButton();
            sleep(1000);

            moveUntilCenterLine(0.5f);
            sleep(1000);

            pressButton();
            sleep(1000);

            requestOpModeStop();

            idle();
        }
    }
}
