package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyColorSensor {

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    ElapsedTime elapsedTime = new ElapsedTime();

    public void enableColorSensor(RevColorSensorV3 colSensor, HardwareMap hwMap) {

        final float values[] = hsvValues;

        final View relativeLayout = ((Activity)hwMap.appContext).findViewById(R.id.RelativeLayout);

        colSensor.enableLed(true);

    }

    public void strafeToGivenColor(LinearOpMode opMode, RevColorSensorV3 colorSensor,  HardwarePushbot robot, MyColor myColor, double power ) {
        boolean colorFound = false;
        while (colorFound == false && opMode.opModeIsActive()) {
            Color.RGBToHSV((int)(colorSensor.red() * 8), (int)(colorSensor.green() *8), (int)(colorSensor.blue() * 8), hsvValues);

            float hue = hsvValues[0];

            float saturation = hsvValues[1];

            float value = hsvValues[2];

            boolean redHue = (hue < 60 || hue > 320) && (saturation > 0.5);

            boolean blueHue = (hue > 180 && hue < 240) && (saturation > 0.5);

            boolean blackHue = hue > 100 && saturation < 0.6 && value > 8 ;

            boolean yellow = hue <100 && saturation > 0.6 && value > 10;

            if ((myColor == MyColor.RED && redHue )  || (myColor == MyColor.BLUE && blueHue) || (myColor == MyColor.BLACK && blackHue)){
                robot.stopWheels();
                colorFound = true;
            }else {
                robot.setWheelPowerForSide(power);
            }
        }
    }

    public void strafeToColorByTime(Telemetry telemetry, LinearOpMode opMode, RevColorSensorV3 colorSensor,  HardwarePushbot robot, MyColor myColor, double power, FeedbackMovement feedbackMovement ) {
        boolean colorFound = false;
        elapsedTime.reset();
        feedbackMovement.initIntegralError(power, robot);

        while (colorFound == false && elapsedTime.seconds() < 4 && opMode.opModeIsActive()) {
            Color.RGBToHSV((int)(colorSensor.red() * 8), (int)(colorSensor.green() *8), (int)(colorSensor.blue() * 8), hsvValues);

            float hue = hsvValues[0];

            float saturation = hsvValues[1];

            float value = hsvValues[2];

            boolean redHue = (hue < 60 || hue > 320) && (saturation > 0.5);

            boolean blueHue = (hue > 180 && hue < 240) && (saturation > 0.5);

            boolean blackHue = hue > 100 && saturation < 0.6 && value > 8 ;

            boolean yellow = hue <100 && saturation > 0.6 && value > 10;

            telemetry.addData("black hue top ", hue);
            telemetry.addData("black saturation top ", saturation);
            telemetry.addData("black value top", value);


            if ((myColor == MyColor.RED && redHue )  || (myColor == MyColor.BLUE && blueHue) || (myColor == MyColor.BLACK && blackHue)){
                robot.stopWheels();
                colorFound = true;
            }else {
               // robot.setWheelPowerForSide(power);
                feedbackMovement.driveWithFeedback(robot, 0, power);
            }
        }
    }

    public void strafeToGivenColorFeedback(Telemetry telemetry, LinearOpMode opMode, RevColorSensorV3 colorSensor, HardwarePushbot robot, MyColor myColor, double power, FeedbackMovement feedbackMovement ) {
        boolean colorFound = false;
        elapsedTime.reset();
        feedbackMovement.initIntegralError(power, robot);
        while (colorFound == false && opMode.opModeIsActive()) {
            Color.RGBToHSV((int)(colorSensor.red() * 8), (int)(colorSensor.green() *8), (int)(colorSensor.blue() * 8), hsvValues);

            float hue = hsvValues[0];

            float saturation = hsvValues[1];

            float value = hsvValues[2];

            telemetry.addData("black hue top ", hue);
            telemetry.addData("black saturation top ", saturation);
            telemetry.addData("black value top", value);

            boolean redHue = (hue < 60 || hue > 320) && (saturation > 0.5);

            boolean blueHue = (hue > 180 && hue < 240) && (saturation > 0.5);

            boolean blackHue = hue > 100 && saturation < 0.6 && value > 8 ;

            boolean yellow = hue <100 && saturation > 0.6 && value > 10;

            telemetry.addData("black hue", hue);
            telemetry.addData("black saturation", saturation);
            telemetry.addData("black value", value);
            telemetry.update();

            if ((myColor == MyColor.RED && redHue )  || (myColor == MyColor.BLUE && blueHue) || (myColor == MyColor.BLACK && blackHue)){
                robot.stopWheels();
                colorFound = true;
            }else {
               // robot.setWheelPowerForSide(power);
                feedbackMovement.driveWithFeedback(robot,0,  power);
            }
        }
    }

}
