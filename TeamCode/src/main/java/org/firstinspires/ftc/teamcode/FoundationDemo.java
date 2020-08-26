package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Foundation Demo")
public class FoundationDemo extends LinearOpMode {

    //Hardware Devices Init
    DcMotor leftFront, rightFront, leftBack, rightBack;

    Servo leftArm, rightArm;

    RevColorSensorV3 rightColorSensor, leftColorSensor;

    DigitalChannel frontTouchSensor, backTouchSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        //Map the hardware Devices.
        //Map DcMotors
        mapHardwareDevices();

        //Set direction
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection((DcMotor.Direction.FORWARD));

        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        int counter = 1;

        waitForStart();

        while (opModeIsActive() && counter == 1) {
            //Arms move up
            leftArm.setPosition(0);
            rightArm.setPosition(0);

            //Go forward until the touch sensor is pressed
            while (!frontTouchSensor.getState()) {
                setDcMotorPower(0.5);
            }

            //Stop the wheels
            setDcMotorPower(0);

            //Move the arms down
            leftArm.setPosition(1);
            rightArm.setPosition(1);

            //Go back until the touch sensor is pressed
            while (!backTouchSensor.getState()) {
                setDcMotorPower(-0.5);

            }

            //Stop the wheels
            setDcMotorPower(0);

            //Move the arms up
            leftArm.setPosition(0);
            rightArm.setPosition(0);

            //Strafe to the left until the color sensor Red
            float[] hsvValues = {0, 0, 0};
            Color.RGBToHSV(leftColorSensor.red() * 255, leftColorSensor.green() * 255, leftColorSensor.blue() * 255, hsvValues);
            boolean redHue = hsvValues[0] < 60 || hsvValues[0] > 360;

            //Set Direction For strafing

            while (!redHue) {
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(-0.5);
            }

            setDcMotorPower(0);

        }
    }

    public void mapHardwareDevices() {
        //Map DcMotors
        leftFront = hardwareMap.get(DcMotor.class, "left-front");
        rightFront = hardwareMap.get(DcMotor.class, "right-front");
        leftBack = hardwareMap.get(DcMotor.class, "left-back");
        rightBack = hardwareMap.get(DcMotor.class, "right-back");

        //Map servos
        leftArm = hardwareMap.get(Servo.class, "left-arm");
        rightArm = hardwareMap.get(Servo.class, "right-arm");

        //Map ColorSensors.
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "right-color-sensor");
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "left-color-sensor");

        //Map touch Sensor
        frontTouchSensor = hardwareMap.get(DigitalChannel.class, "front-touch-sensor");
        backTouchSensor = hardwareMap.get(DigitalChannel.class, "back-touch-sensor");
    }

    public void setDcMotorPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
}
