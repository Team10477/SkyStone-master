package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Skystone Blue Bridge")
public class SkystoneBlueColorSensorBridge extends LinearOpMode {

    SkystonePushBot robot = new SkystonePushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private FeedbackMovement feedbackMovement = new FeedbackMovement();

    private static final double STRAFE_RIGHT = 0.5;
    private static final double STRAFE_LEFT = -0.35;
    private static final double STRAFE_LEFT_SLOW =-0.30;
    private static final double STRAFE_RIGHT_SLOW = 0.30;
    private static final double PICKUP_GRAB= 0.9;
    private static final double DRIVE_FORWARD = -0.35;
    private static final double DRIVE_FORWARD_SLOW = -0.15;
    private static final double DRIVE_FORWARD_SLOW_PICKUP = -0.2;
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

            goForwardBlind(500);

            goForwardNearStone(6);

            stopAtBlackSkystone(750);

            pickUpSkyStone();

            goBackward();

            strafeLeft(500);

            releaseSkyStone();

            strafeRight();

            huntForSecondSkystone();

            counter++;
        }

    }

    private void goForwardBlind(long msecsDuration) {
        feedbackMovement.initIntegralError(DRIVE_FORWARD, robot);
        feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD, 0);
        sleep(msecsDuration);
        robot.stopWheels();
    }

    /**
     *  Go forward Using Distance Sensor.
     */
    private void goForwardNearStone(double givenDistance) {
        double distance = robot.colorSensorFront.getDistance(DistanceUnit.CM);

        while (distance > givenDistance && opModeIsActive()) {
            feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD_SLOW , 0);
            distance = robot.colorSensorFront.getDistance(DistanceUnit.CM);

        }
        telemetry.addData("Distance : ", distance);
        telemetry.update();
        robot.stopWheels();
    }

    private void stopAtBlackSkystone(long msecDeltaStrafe) {

        myColorSensor.strafeToColorByTime(telemetry, this, robot.colorSensorFront, robot, MyColor.BLACK, STRAFE_RIGHT_SLOW , feedbackMovement);

        feedbackMovement.initIntegralError(STRAFE_LEFT_SLOW , robot);

        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT_SLOW);
        sleep(msecDeltaStrafe);

        feedbackMovement.initIntegralError(DRIVE_FORWARD_SLOW_PICKUP , robot);
        feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD_SLOW_PICKUP , 0);
        sleep(125);

        robot.stopWheels();
    }

    private void pickUpSkyStone() {
        robot.pickupArm.setPosition(PICKUP_GRAB);
        sleep(2000);
        robot.resetIfArmTouches();
        robot.stopWheels();
    }

    private void goBackward() {
        feedbackMovement.initIntegralError(DRIVE_BACKWARD, robot);
        feedbackMovement.driveWithFeedback(robot,DRIVE_BACKWARD, 0);
        sleep(350);
        robot.resetIfArmTouches();
        robot.stopWheels();
    }

    private void strafeLeft(long msecOvershoot) {
        elapsedTime.reset();

        myColorSensor.strafeToGivenColorFeedbackWithArm(telemetry, this, robot.colorSensorRight, robot, MyColor.BLUE, STRAFE_LEFT, feedbackMovement );

        robot.resetIfArmTouches();
        // More strafing after detecting Red line under bridge.
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT);
        sleep(msecOvershoot);
        // sleep(500);
        seconds = elapsedTime.seconds() ;
        seconds *= .7;
        telemetry.addData("Strafe right time : ", seconds);
        telemetry.update();

        robot.stopWheels();
    }

    private void releaseSkyStone() {
        robot.pickupArm.setPosition(0);
        sleep(1000);
        robot.stopWheels();
    }

    private void strafeRight() {
        elapsedTime.reset();
        feedbackMovement.initIntegralError(STRAFE_RIGHT, robot);
        while (elapsedTime.seconds() < seconds  && opModeIsActive()) {
            feedbackMovement.driveWithFeedback(robot, 0, STRAFE_RIGHT);
        }

        robot.stopWheels();
    }

    private void huntForSecondSkystone() {
        goForwardBlind(50);

        goForwardNearStone(5);

        stopAtBlackSkystone(600);

        pickUpSkyStone();

        goBackward();

        strafeLeft(500);

        releaseSkyStone();

        parkUnderBridge();
    }

    private void parkUnderBridge() {
        myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensorRight, robot, MyColor.BLUE, STRAFE_RIGHT, feedbackMovement);
        feedbackMovement.initIntegralError(DRIVE_FORWARD , robot);
        feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD , 0);
        sleep(350);
    }

}