package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Strafe To Wall", group = "Mission")
public class RedStrafeToWall extends LinearOpMode {

    private FoundationPushBot robot = new FoundationPushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private FeedbackMovement feedbackMovement = new FeedbackMovement();

    private static final double WHEEL_MOVING_SPEED = 0.35;

    private static final double STRAFE_LEFT = -0.35;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        myColorSensor.enableColorSensor(robot.colorSensorRight, hardwareMap);

        feedbackMovement.initializeImu(hardwareMap);

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensor, robot, MyColor.RED, STRAFE_LEFT, feedbackMovement);

            adjustStrafeLeft();

            counter++;
        }

    }


    /**
     * Strafe little left as adjustment to park under the bridge.
     */
    private void adjustStrafeLeft() {
        feedbackMovement.initIntegralError(STRAFE_LEFT, robot);
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT);
        sleep(50);
    }

}
