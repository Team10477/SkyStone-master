package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Foundation Mission Red", group = "Mission")
public class FoundationMissionRed extends LinearOpMode {

    FoundationPushBot robot = new FoundationPushBot();

    private static final double WHEEL_MOVING_SPEED = 0.5;

    private static final double ARM_POSITION = -0.25;

    private ElapsedTime period  = new ElapsedTime();

    double timeTaken;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        period.reset();

        timeTaken = period.startTime();  // Start time from 0.

        waitForStart();

        robot.setWheelPowerForSide(-0.5);       // Move sideways to right.

        sleep(1000);

        while (robot.touchSensor.getState()) {      // Go forward until touch sensor is pressed.
           robot.setWheelPower(WHEEL_MOVING_SPEED);
        }

        robot.stopWheels() ;

        robot.setPosition(ARM_POSITION);        // Foundation Arms down

        sleep(1000);

        timeTaken = period.milliseconds();

        telemetry.addData("Time Taken to go forward", timeTaken);
        telemetry.update();

        robot.setWheelDirectionReverse();

        robot.setWheelPower(WHEEL_MOVING_SPEED );    //Go backwards for a certain time.

        long timeInMs = (long) (timeTaken * 1000);

        telemetry.addData("Time Taken to go forward in millsecs", timeInMs);
        telemetry.update();

        sleep(timeInMs);

        robot.stopWheels() ;

        robot.setPosition(0.25);        //Foundation arm back to start position.

        sleep(1000);

        robot.setWheelPowerForSide(0.5);    //Robot goes sideways to left under the bridge.

        sleep(1000);

        robot.stopWheels();             //Stop the robot.

    }
}
