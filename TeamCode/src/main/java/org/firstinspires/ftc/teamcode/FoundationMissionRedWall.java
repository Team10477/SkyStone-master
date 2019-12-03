package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Foundation Mission Red Wall", group = "Mission")
public class FoundationMissionRedWall extends LinearOpMode {

    private FoundationPushBot robot = new FoundationPushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private static final double DRIVE_BACKWARD = 0.25;

    private static final double DRIVE_FORWARD = -0.35;

    private static final double ARM_DOWN_POSITION = 1;

    private static final double ARM_UP_POSITION = 0;

    private static final double STRAFE_RIGHT = 0.35;

    private static final double STRAFE_LEFT = -0.5;

    private ElapsedTime elapsedTime = new ElapsedTime();

    FeedbackMovement feedbackMovement = new FeedbackMovement();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        myColorSensor.enableColorSensor(robot.colorSensorRight, hardwareMap);
        feedbackMovement.initializeImu(hardwareMap);
        feedbackMovement.resetAngle();
        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

           resetArms();

            moveLeft();

           goBackwardUntilTouchSensorPressed();

           moveArmsDown();

           goForwardUntilTouchSensorPressed();

           moveArmsUp();

           moveRightUntilRed();

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
    private void moveLeft() {
        feedbackMovement.initIntegralError(STRAFE_LEFT, robot);
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT);
        sleep(700);
    }

    /**
     * Go Forward Until touch sensor is pressed.
     */
    private void goBackwardUntilTouchSensorPressed(){

        feedbackMovement.initIntegralError(DRIVE_BACKWARD, robot);
        while (robot.touchSensor.getState() && opModeIsActive()) {
               feedbackMovement.driveWithFeedback(robot,DRIVE_BACKWARD, 0);
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
    private void goForwardUntilTouchSensorPressed() {

        elapsedTime.reset();
        feedbackMovement.initIntegralError(DRIVE_FORWARD, robot);
        while ((robot.touchSensorFront.getState() || elapsedTime.seconds() < 4) && opModeIsActive())  {
               feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD, 0);
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
     * Strafe little left as adjustment to park under the bridge.
     */
    private void moveRightUntilRed() {

        feedbackMovement.initIntegralError(STRAFE_RIGHT, robot);

        myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensorRight, robot,MyColor.RED ,STRAFE_RIGHT, feedbackMovement);

        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_RIGHT);

        sleep(50);
    }



}
