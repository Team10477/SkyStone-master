package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Strafe Wall", group = "Mission")
public class BlueStrafeToWall extends LinearOpMode {

    FoundationPushBot robot = new FoundationPushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private FeedbackMovement feedbackMovement = new FeedbackMovement();

    private static final double STRAFE_RIGHT= 0.35;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        myColorSensor.enableColorSensor(robot.colorSensorRight, hardwareMap);

        feedbackMovement.initializeImu(hardwareMap);

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensorRight, robot, MyColor.BLUE, STRAFE_RIGHT, feedbackMovement);

            adjustStrafeRight();

            counter++;
        }

    }

    /**
     * Strafe little left as adjustment under the bridge.
     */
    private void adjustStrafeRight() {
        feedbackMovement.initIntegralError(STRAFE_RIGHT, robot);
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_RIGHT);
        sleep(50);
    }



}
