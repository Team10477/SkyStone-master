package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Foundation Mission Red Wall", group = "Mission")
public class FoundationMissionRedWall extends LinearOpMode {

    FoundationPushBot robot = new FoundationPushBot();

    private static final double WHEEL_MOVING_SPEED = 0.5;

    private static final double ARM_POSITION = 1;

    private ElapsedTime period  = new ElapsedTime();

    double timeTaken;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);  // Mapping between program and Robot.

        period.reset();

        enableColorSensor();

        boolean colorFound = false;

        waitForStart();

        robot.setPosition(0); //Starting Position

        sleep(1000);

        robot.setWheelPowerForSide(-0.5);       // Move sideways to right.

        sleep(1000);

        while (robot.touchSensor.getState()) {      // Go forward until touch sensor is pressed.
           robot.setWheelPower(WHEEL_MOVING_SPEED);
        }

        robot.stopWheels() ;

        robot.setPosition(ARM_POSITION);        // Foundation Arms down

        sleep(1000);

        robot.setWheelDirectionReverse();

        while (robot.touchSensorFront.getState()) {      // Go backward until touch sensor is pressed.
            robot.setWheelPower(WHEEL_MOVING_SPEED);
        }

        robot.stopWheels() ;

        robot.setPosition(0);        //Foundation arm back to start position.

        sleep(1000);

        stopAtRed(false);

        robot.setWheelPowerForSide(-0.5);
        sleep(50);

    }

    public void stopAtRed(boolean colorFound) {
        while (colorFound == false) {
            Color.RGBToHSV((int)(robot.colorSensorRight.red() * 8), (int)(robot.colorSensorRight.green() *8), (int)(robot.colorSensorRight.blue() * 8), hsvValues);

            float hue = hsvValues[0];

            float saturation = hsvValues[1];

            telemetry.addData("Color Red", hue);
            telemetry.update();

            boolean redHue = (hue < 60 || hue > 320) && (saturation > 0.5);
            boolean blueHue = (hue > 180 && hue < 240) && (saturation > 0.5);

            if (redHue)  {
                robot.stopWheels();
                colorFound = true;
            }else {
                robot.setWheelPowerForSide(-0.5);
            }
        }
    }

    public void enableColorSensor() {

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        final View relativeLayout = ((Activity)hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        robot.colorSensorRight.enableLed(true);

    }


}
