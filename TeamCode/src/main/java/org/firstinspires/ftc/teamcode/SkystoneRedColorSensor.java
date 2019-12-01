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


@Autonomous(name="Skystone Red Color Sensor")
public class SkystoneRedColorSensor extends LinearOpMode {

    SkystonePushBot robot = new SkystonePushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    FeedbackMovement feedback = new FeedbackMovement();

    double STRAFE_RIGHT = 0.4;
    double STRAFE_LEFT = -0.4;

    private ElapsedTime elapsedTime = new ElapsedTime();

    private double milliseconds;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        myColorSensor.enableColorSensor(robot.colorSensorFront, hardwareMap);

        feedback.initializeImu(hardwareMap);
        feedback.resetAngle();

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            goForwardBlind();

            goForwardNearStone();

            myColorSensor.strafeToGivenColorFeedback(telemetry, this, robot.colorSensorFront, robot, MyColor.BLACK, STRAFE_LEFT, feedback);

            adjustStrafeToSide();

            pickUpSkyStone();

            goBackward();

            strafeRight();

            //Release the stone
            releaseSkyStone();

            //Strafe to left and stop at Red
            strafeLeft();

            huntForSecondSkystone();

            counter++;
        }

    }

    private void goForwardBlind() {
        robot.setWheelDirectionForward();
        robot.setWheelPower(0.35);
        sleep(700);
        robot.stopWheels();
    }

    /**
     *  Go forward Using Distance Sensor.
     */
    private void goForwardNearStone() {

        robot.setWheelDirectionForward();
        double distance = robot.colorSensorFront.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance", distance);
        telemetry.update();
        while (distance > 9 && opModeIsActive()) {
            robot.setWheelPower(0.2);
            telemetry.addData("Distance", distance);
            telemetry.update();
            distance = robot.colorSensorFront.getDistance(DistanceUnit.CM);

        }
        robot.stopWheels();
    }

    private void adjustStrafeToSide() {
        robot.setWheelPowerForSide(-0.30);
        sleep(550);
        robot.stopWheels();
    }

    private void pickUpSkyStone() {
        robot.pickupArm.setPosition(.9);
        sleep(2000);
        robot.stopWheels();
    }

    private void goBackward() {
        robot.setWheelDirectionReverse();
        robot.setWheelPower(0.35);
        sleep(450);
        robot.stopWheels();
    }

    private void strafeRight() {

        elapsedTime.reset();
        myColorSensor.strafeToGivenColorFeedback(telemetry, this, robot.colorSensor, robot, MyColor.RED, STRAFE_RIGHT, feedback);

        // More strafing after detecting Red line under bridge.
        robot.setWheelPowerForSide(STRAFE_RIGHT);
        sleep(600);
        milliseconds = elapsedTime.seconds();
        telemetry.addData("Strafe right time : ", milliseconds);
        telemetry.update();
        robot.stopWheels();
    }

    private void releaseSkyStone() {
        robot.pickupArm.setPosition(0);
        sleep(1000);
        robot.stopWheels();
    }

    private void strafeLeft() {
     //   robot.setWheelPowerForSide(0.5);
       // myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensor, robot, MyColor.RED, STRAFE_LEFT, feedback);
        elapsedTime.reset();
        while (elapsedTime.seconds() < milliseconds) {
           // robot.setWheelPowerForSide(STRAFE_LEFT);
            feedback.strafeWithFeedback(robot, STRAFE_LEFT);
        }
      //  sleep(new Double(milliseconds * 1000).longValue());
        //  stop at red
       robot.stopWheels();
    }

    private void huntForSecondSkystone() {
        goForwardNearStone();

        myColorSensor.strafeToGivenColorFeedback(telemetry, this, robot.colorSensorFront, robot, MyColor.BLACK, STRAFE_LEFT, feedback);

        adjustStrafeToSide();

        pickUpSkyStone();

        goBackward();

        strafeRight();

        //Release the stone
        releaseSkyStone();

        parkUnderBridge();
    }

    private void parkUnderBridge() {
        myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensor, robot, MyColor.RED, STRAFE_LEFT, feedback);
    }

}