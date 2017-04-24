package org.firstinspires.ftc.teamcode.AutoNew;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Consts;

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
            // forward and shoot
            extendSideWheels();
            moveDistance(1400, 0.5f);
            sleep(250);

            shoot();

            disableFlywheels();

            // aim for the wall and go forwards
            turnToHeading((isRed ? 43 : 135), 0.45f, 1, true);
            sleep(250);

            moveDistance(blueNegativeFactor * 3900, 0.4f);
            sleep(250);

            turnToHeading((isRed ? 0 : 164), 0.5f, 1, true);
            sleep(250);

            sideFrontWheel.setPower(Consts.SIDE_FRONT_WHEEL_HOLD_POWER);
            sideBackWheel.setPower(Consts.SIDE_BACK_WHEEL_HOLD_POWER);

            // press the first beacon
            moveUntilCenterLine(blueNegativeFactor * 0.45f, blueNegativeFactor * 0.45f);
            sleep(250);

            moveUntilCenterLine(blueNegativeFactor * -0.3f, blueNegativeFactor * -0.3f);
            sleep(250);

            turnToHeading((isRed ? 0 : 177), 0.5f, 0, true);
            sleep(250);

            pressButton(true);

            // go towards the second beacon
            moveDistance(blueNegativeFactor * -1500, 1.0f);
            sleep(250);

            moveUntilCenterLine(blueNegativeFactor * -0.45f, blueNegativeFactor * -0.45f);
            sleep(250);

            moveUntilCenterLine(blueNegativeFactor * 0.3f, blueNegativeFactor * 0.3f);
            sleep(250);

            turnToHeading((isRed ? 0 : 177), 0.5f, 0, true);
            sleep(250);

            pressButton(true);
            sleep(250);

            requestOpModeStop();

            idle();
        }
    }
}
