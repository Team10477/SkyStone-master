package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Skystone Red Wall")
public class SkystoneRedColorSensorWall extends LinearOpMode {

    private SkystonePushBot robot = new SkystonePushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private FeedbackMovement feedbackMovement = new FeedbackMovement();

    private static final double STRAFE_RIGHT = 0.4;
    private static final double STRAFE_LEFT = -0.5;
    private static final double STRAFE_LEFT_BLACK = -0.35;
    private static final double STRAFE_LEFT_SLOW = -0.30;
    private static final double PICKUP_GRAB= 0.9;
    private static final double DRIVE_FORWARD = -0.35;
    private static final double DRIVE_FORWARD_SLOW = -0.15;
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

            goForwardNearStone(5);

            stopAtBlackSkystone();

            pickUpSkyStone();

            robot.resetIfArmTouches();

            goBackward(450);

            strafeRight(500);

            releaseSkyStone();

            strafeLeft();

            huntForSecondSkystone();

            counter++;
        }

    }

    private void goForwardBlind(long msecDuration) {
        feedbackMovement.initIntegralError(DRIVE_FORWARD, robot);
        feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD, 0);
      //  sleep(500);
        sleep(msecDuration);
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
        robot.stopWheels();
    }

    private void stopAtBlackSkystone() {

        myColorSensor.strafeToColorByTime(telemetry, this, robot.colorSensorFront, robot, MyColor.BLACK, STRAFE_LEFT_BLACK, feedbackMovement);

        feedbackMovement.initIntegralError(STRAFE_LEFT_SLOW , robot);
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT_SLOW);
        sleep(450);

        feedbackMovement.initIntegralError(DRIVE_FORWARD_SLOW , robot);
        feedbackMovement.driveWithFeedback(robot, DRIVE_FORWARD_SLOW , 0);
        sleep(100);
        robot.stopWheels();
    }

    private void pickUpSkyStone() {
        robot.pickupArm.setPosition(PICKUP_GRAB);
        sleep(2000);
        robot.resetIfArmTouches();
        robot.stopWheels();
    }

    private void goBackward(long msecOffset) {
        feedbackMovement.initIntegralError(DRIVE_BACKWARD, robot);
        feedbackMovement.driveWithFeedback(robot,DRIVE_BACKWARD, 0);
        sleep(msecOffset);
        robot.resetIfArmTouches();
        //sleep(1350);
        robot.stopWheels();
    }

    private void strafeRight(long msecOvershoot) {
        elapsedTime.reset();

        myColorSensor.strafeToGivenColorFeedbackWithArm(telemetry, this, robot.colorSensor, robot, MyColor.RED, STRAFE_RIGHT, feedbackMovement );

        robot.resetIfArmTouches();
        // More strafing after detecting Red line under bridge.
        feedbackMovement.driveWithFeedback(robot, 0, STRAFE_RIGHT);
        sleep(msecOvershoot);
        seconds = elapsedTime.seconds();
        seconds *= .75;
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
        while (elapsedTime.seconds() < seconds && opModeIsActive()) {
              feedbackMovement.driveWithFeedback(robot, 0, STRAFE_LEFT);
        }

       robot.stopWheels();
    }

    private void huntForSecondSkystone() {
        goForwardBlind(250);

        goForwardNearStone(5);

        stopAtBlackSkystone();

        pickUpSkyStone();

        robot.resetIfArmTouches();

        goBackward(1350);

        robot.resetIfArmTouches();

        strafeRight(300);

        releaseSkyStone();

        parkUnderBridge();
    }

    private void parkUnderBridge() {
        myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensor, robot, MyColor.RED, STRAFE_LEFT, feedbackMovement);
    }

}