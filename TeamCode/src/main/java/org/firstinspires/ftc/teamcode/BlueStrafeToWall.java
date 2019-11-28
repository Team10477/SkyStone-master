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

    private static final double WHEEL_MOVING_SPEED = 0.35;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        myColorSensor.enableColorSensor(robot.colorSensor, hardwareMap);

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            myColorSensor.strafeToGivenColor(this, robot.colorSensor, robot, MyColor.BLUE ,-0.35);

            adjustStrafeRight();

            counter++;
        }

    }

    /**
     * Strafe little left as adjustment under the bridge.
     */
    private void adjustStrafeRight() {
        robot.setWheelPowerForSide(-0.35);
        sleep(50);
    }



}
