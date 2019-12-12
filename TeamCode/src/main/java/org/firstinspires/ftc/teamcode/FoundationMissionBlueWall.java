package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Foundation Mission Blue Wall", group = "Mission")
public class FoundationMissionBlueWall extends LinearOpMode {

    FoundationPushBot robot = new FoundationPushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private FeedbackMovement feedbackMovement = new FeedbackMovement();

    private ElapsedTime elapsedTime = new ElapsedTime();

    private static final double DRIVE_BACKWARD = 0.25;

    private static final double DRIVE_FORWARD = -0.35;

    private static final double DRIVE_BACKWARD_MORE_POWER = 0.35;

    private static final double ARM_DOWN_POSITION = 1;

    private static final double ARM_UP_POSITION = 0;

    private static final double STRAFE_RIGHT = 0.5;

    private static final double STRAFE_LEFT = -0.5;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        myColorSensor.enableColorSensor(robot.colorSensor, hardwareMap);

        feedbackMovement.initializeImu(hardwareMap);

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            resetArms();

            moveToRight();

            goBackwardUntilTouchSensorPressed();

            moveArmsDown();

            goForwardUntilTouchSensorPressed();

            moveArmsUp();

            moveLeftUntilBlue();

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
        feedbackMovement.initIntegralError(STRAFE_RIGHT, robot);
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_RIGHT);
        sleep(700);
    }

    /**
     * Go Backward Until touch sensor is pressed.
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
     * Go Forward until touch sensor is pressed.
     */
    private void goForwardUntilTouchSensorPressed() {

        elapsedTime.reset();
        feedbackMovement.initIntegralError(DRIVE_FORWARD, robot);
        while ((robot.touchSensorFront.getState() && elapsedTime.seconds() < 3) && opModeIsActive())  {
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
     * Strafe little left as adjustment under the bridge.
     */
    private void  moveLeftUntilBlue()  {
        feedbackMovement.initIntegralError(STRAFE_LEFT, robot);

        myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensor, robot,MyColor.BLUE ,STRAFE_LEFT, feedbackMovement);

        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT);

        sleep(50);
    }


}
