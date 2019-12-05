package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;


@Autonomous(name="Skystone Red Bridge")
public class SkystoneRedColorSensorBridge extends LinearOpMode {

    SkystonePushBot robot = new SkystonePushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private FeedbackMovement feedbackMovement = new FeedbackMovement();

    private static final double STRAFE_RIGHT = 0.4;
    private static final double STRAFE_LEFT = -0.5;
    private static final double STRAFE_LEFT_BLACK = -0.35;
    private static final double STRAFE_LEFT_SLOW = -0.30;
    private static final double PICKUP_GRAB= 0.9;
    private static final double DRIVE_FORWARD = -0.35;
    private static final double DRIVE_FORWARD_SLOW = -0.2;
    private static final double DRIVE_BACKWARD = 0.35;

    private ElapsedTime elapsedTime = new ElapsedTime();

    private double seconds;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        myColorSensor.enableColorSensor(robot.colorSensorFront, hardwareMap);

        feedbackMovement.initializeImu(hardwareMap);

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            goForwardBlind();

            goForwardNearStone(9);

            stopAtBlackSkystone();

            pickUpSkyStone();

            goBackward();

            strafeRight(500);

            releaseSkyStone();

            strafeLeft();

            huntForSecondSkystone();

            counter++;
        }

    }

    private void goForwardBlind() {
        feedbackMovement.initIntegralError(DRIVE_FORWARD, robot);
        feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD, 0);
        sleep(500);
        robot.stopWheels();
    }

    /**
     *  Go forward Using Distance Sensor.
     */
    private void goForwardNearStone(double givenDistance) {
        // feedbackMovement.initIntegralError(DRIVE_FORWARD, robot);
        double distance = robot.colorSensorFront.getDistance(DistanceUnit.CM);

        while (distance > givenDistance && opModeIsActive()) {
            feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD_SLOW , 0);
            distance = robot.colorSensorFront.getDistance(DistanceUnit.CM);

        }
        telemetry.addData("Distance : ", distance);
        telemetry.update();
        robot.stopWheels();
    }

    private void stopAtBlackSkystone() {

        myColorSensor.strafeToColorByTime(telemetry, this, robot.colorSensorFront, robot, MyColor.BLACK, STRAFE_LEFT_BLACK , feedbackMovement);

        feedbackMovement.initIntegralError(STRAFE_LEFT_SLOW , robot);

        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT_SLOW);
        sleep(500);
        robot.stopWheels();
    }
    private void pickUpSkyStone() {
        robot.pickupArm.setPosition(PICKUP_GRAB);
        sleep(2000);
        robot.stopWheels();
    }

    private void goBackward() {
        feedbackMovement.initIntegralError(DRIVE_BACKWARD, robot);
        feedbackMovement.driveWithFeedback(robot,DRIVE_BACKWARD, 0);
        sleep(450);
        robot.stopWheels();
    }

    private void strafeRight(long msecOvershoot) {
        elapsedTime.reset();

        myColorSensor.strafeToGivenColorFeedback(telemetry, this, robot.colorSensor, robot, MyColor.RED, STRAFE_RIGHT, feedbackMovement );

        // More strafing after detecting Red line under bridge.
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_RIGHT);
        sleep(msecOvershoot);
       // sleep(500);
        seconds = elapsedTime.seconds() ;
        seconds *= .6;
        telemetry.addData("Strafe right time : ", seconds);
        telemetry.update();

        robot.stopWheels();

    }

    private void releaseSkyStone() {
        robot.pickupArm.setPosition(0);
        sleep(1000);
        robot.stopWheels();
    }

    private void strafeLeft() {
        elapsedTime.reset();
        feedbackMovement.initIntegralError(STRAFE_LEFT, robot);
        while (elapsedTime.seconds() < seconds  && opModeIsActive()) {
            feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT);
        }

        robot.stopWheels();
    }

    private void huntForSecondSkystone() {
        goForwardNearStone(7);

        stopAtBlackSkystone();

        pickUpSkyStone();

        robot.resetIfArmTouches();

        goBackward();

        robot.resetIfArmTouches();

        strafeRight(300);

        releaseSkyStone();

        parkUnderBridge();
    }


    private void parkUnderBridge() {
        myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensor, robot, MyColor.RED, STRAFE_LEFT, feedbackMovement);
    }

}