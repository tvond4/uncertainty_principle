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
            //enableSideWheels();
            moveDistance(1400, 0.5f);
            //holdSideWheels();
            sleep(100);

            shoot();

            disableFlywheels();

            // aim for the wall and go forwards
            turnToHeading((isRed ? 43 : 135), 0.45f, 1, true);
            sleep(100);

            moveDistance((isRed ? blueNegativeFactor * 3900 : blueNegativeFactor * 4300), 0.4f);
            sleep(100);

            // aim to wall
            turnToHeading(blueNegativeFactor * 90, 0.5f, 1, true);
            sleep(100);

            moveToDistance(13, 0.5f);
            sleep(100);

            moveDistance(-100, 0.5f);

            turnToHeading((isRed ? 0 : 164), 0.5f, 1, true);
            sleep(100);

            //sideFrontWheel.setPower(Consts.SIDE_FRONT_WHEEL_HOLD_POWER);
            //sideBackWheel.setPower(Consts.SIDE_BACK_WHEEL_HOLD_POWER);

            // press the first beacon
            moveUntilCenterLine(blueNegativeFactor * 0.45f, blueNegativeFactor * 0.45f);
            sleep(100);

            moveUntilCenterLine(blueNegativeFactor * -0.3f, blueNegativeFactor * -0.3f);
            sleep(100);

            turnToHeading((isRed ? 0 : 164), 0.45f, 1, true);
            sleep(100);

            pressButton(true, 1);

//            turnToHeading((isRed ? 0 : 177), 0.5f, 0, true);
//            sleep(100);

            // go towards the second beacon
            moveDistance(blueNegativeFactor * -2000, 1.0f);
            sleep(100);

            sidePusher.setPower(0.0);

            moveUntilCenterLine(blueNegativeFactor * -0.45f, blueNegativeFactor * -0.45f);
            sleep(100);

            moveUntilCenterLine(blueNegativeFactor * 0.3f, blueNegativeFactor * 0.3f);
            sleep(100);

//            turnToHeading((isRed ? 0 : 177), 0.5f, 0, true);
//            sleep(100);

            pressButton(true, 1);
            sleep(100);

            turnToHeading((isRed ? 3 : 164), 0.5f, 1, true);
            sleep(100);

            moveDistance(blueNegativeFactor * -2000, 1.0f);

            requestOpModeStop();

            idle();
        }
    }
}
