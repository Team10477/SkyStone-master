package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;


@Autonomous(name="Stop At Blue")

public class StopAtBlue extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();

    RevColorSensorV3 colorSensor = null;    // Hardware Device Object

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        boolean colorFound = false;

        final View relativeLayout = ((Activity)hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        colorSensor.enableLed(true);

        waitForStart();

       while (colorFound == false) {
            Color.RGBToHSV((int)(colorSensor.red() * 8), (int)(colorSensor.green() *8), (int)(colorSensor.blue() * 8), hsvValues);

            float hue = hsvValues[0];

            telemetry.addData("Color Red", hue);
            telemetry.update();

            boolean redHue = hue < 60 || hue > 320;
            boolean blueHue = hue > 120 && hue < 260;

            if (blueHue)  {
               robot.setWheelPower(0);
               colorFound = true;
           }else {
                robot.setWheelPower(0.5);
                }
            }
       }



    }