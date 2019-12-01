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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Skystone Red Color Sensor")
public class SkystoneRedColorSensor extends LinearOpMode {

    SkystonePushBot robot = new SkystonePushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    FeedbackMovement feedback = new FeedbackMovement();

    double leftFrontPower;
    double rightFrontPower;
    double leftRearPower;
    double rightRearPower;

    double heading;
    double integralError =0;
    double error =0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;
    double STRAFE_RIGHT = 0.35;
    double STRAFE_LEFT = -0.35;
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();
    double                  globalAngle;

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

         //  myColorSensor.strafeToColorByTime(telemetry, this, robot.colorSensorFront, robot, MyColor.BLACK, STRAFE_LEFT, feedback);

           myColorSensor.strafeToGivenColorFeedback(telemetry, this, robot.colorSensorFront, robot, MyColor.BLACK, STRAFE_LEFT, feedback);

            adjustStrafeToSide();

            pickUpSkyStone();

            goBackward();

           strafeRight();

         //   myColorSensor.strafeToGivenColorFeedback(telemetry, this, robot.colorSensor, robot, MyColor.RED, STRAFE_RIGHT, feedback);


            //Release the stone
            releaseSkyStone();

            //Strafe to left and stop at Red
            strafeLeft();

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

        myColorSensor.strafeToGivenColorFeedback(telemetry, this, robot.colorSensor, robot, MyColor.RED, STRAFE_RIGHT, feedback);


        // More strafing after detecting Red line under bridge.
        robot.setWheelPowerForSide(STRAFE_RIGHT);
        sleep(500);
        robot.stopWheels();
    }

    private void releaseSkyStone() {
        robot.pickupArm.setPosition(0);
        sleep(1000);
        robot.stopWheels();
    }

    private void strafeLeft() {
     //   robot.setWheelPowerForSide(0.5);
        myColorSensor.strafeToGivenColorFeedback(telemetry,this, robot.colorSensor, robot, MyColor.RED, STRAFE_LEFT, feedback);

        //  stop at red
      //  robot.stopWheels();
    }


}