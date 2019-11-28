package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Strafe To Wall", group = "Mission")
public class RedStrafeToWall extends LinearOpMode {

    private FoundationPushBot robot = new FoundationPushBot();

    private MyColorSensor myColorSensor = new MyColorSensor();

    private static final double WHEEL_MOVING_SPEED = 0.35;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        myColorSensor.enableColorSensor(robot.colorSensorRight, hardwareMap);

        waitForStart();

        int counter = 1;

        while (opModeIsActive() && counter == 1) {

            myColorSensor.strafeToGivenColor(this, robot.colorSensorRight, robot,MyColor.RED ,0.35);

            adjustStrafeLeft();

            counter++;
        }

    }


    /**
     * Strafe little left as adjustment to park under the bridge.
     */
    private void adjustStrafeLeft() {
        robot.setWheelPowerForSide(WHEEL_MOVING_SPEED);
        sleep(50);
    }

}
