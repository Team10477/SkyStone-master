package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Foundation Mission Red Bridge", group = "Mission")
public class FoundationMissionRedBridge extends LinearOpMode {

    FoundationPushBot robot = new FoundationPushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private static final double WHEEL_MOVING_SPEED = 0.25;

    private static final double ARM_DOWN_POSITION = 1;

    private static final double ARM_UP_POSITION = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        myColorSensor.enableColorSensor(robot.colorSensorRight, hardwareMap);

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            resetArms();

            moveToRight();

            goForwardUntilTouchSensorPressed();

            moveArmsDown();

            goBackwardUntilTouchSensorPressed();

            moveArmsUp();

            myColorSensor.strafeToGivenColor(this, robot.colorSensorRight, robot,MyColor.RED ,0.35);

            goForwardNearBridge();

            counter++;
        }

    }

    /**
     * Reset Arms to starting position.
     */
    private void resetArms() {
        if (!robot.isArmsUp())
            robot.setPosition(ARM_UP_POSITION); //Starting Position

    }

    /**
     * Strafe Robot to Right.
     */
    private void moveToRight() {
        robot.setWheelPowerForSide(-0.5);
        sleep(700);
    }

    /**
     * Go Forward Until touch sensor is pressed.
     */
    private void goForwardUntilTouchSensorPressed(){

        while (robot.touchSensor.getState() && opModeIsActive()) {
            robot.setWheelPowerBackward(WHEEL_MOVING_SPEED);
        }

        robot.stopWheels();
    }

    /**
     *  Move Foundation Arms down.
     */
    private void moveArmsDown() {
        robot.setPosition(ARM_DOWN_POSITION);
        sleep(1500);
    }


    /**
     * Go backward until touch sensor is pressed.
     */
    private void goBackwardUntilTouchSensorPressed() {

        robot.setWheelDirectionForward();
        while (robot.touchSensorFront.getState() && opModeIsActive()) {
            robot.setWheelPower(0.35);
        }

        robot.stopWheels();
    }

    /**
     * Move the foundation arms up.
     */
    private void moveArmsUp() {
        robot.setPosition(ARM_UP_POSITION);
        sleep(500);
    }

    /**
     * Strafe little left as adjustment under the bridge.
     */
    private void adjustStrafeLeft() {
        robot.setWheelPowerForSide(WHEEL_MOVING_SPEED);
        sleep(50);
    }

    /**
     * Go Forward for few seconds to park near the bridge.
     */
    private void goForwardNearBridge() {
        robot.setWheelDirectionReverse();
        robot.setWheelPower(0.35);
        sleep(900);
    }
}
