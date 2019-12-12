/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative_Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor linearSlide = null;
    private Servo leftHand = null;
    private Servo rightHand = null;
    private Servo capstoneLeft = null;
    private Servo capstoneRight = null;
    private Servo frontArm = null;
    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDistance = null;

    double masterPowerScaleDrive = 0.4;
    double masterPowerScaleTurn = 0.35;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    //final double SCALE_FACTOR = 255;
    final double SCALE_FACTOR = 8;
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
 //   int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
 //   final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear");
        linearSlide = hardwareMap.get(DcMotor.class,"linear_slide");
        leftHand = hardwareMap.get(Servo.class,"left_hand");
        rightHand = hardwareMap.get(Servo.class,"right_hand");
        frontArm = hardwareMap.get(Servo.class,"front_arm");
        capstoneLeft=hardwareMap.get(Servo.class,"capstone_left");
        capstoneRight=hardwareMap.get(Servo.class,"capstone_right");
        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor_front");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor_front");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        leftHand.setDirection(Servo.Direction.REVERSE);
        rightHand.setDirection(Servo.Direction.FORWARD);
        capstoneLeft.setDirection(Servo.Direction.FORWARD);
        capstoneRight.setDirection(Servo.Direction.FORWARD);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override


    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;
        double linearSlidePower;
        double servoPower;
        double frontArmPosition;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn  =  -gamepad1.right_stick_x;
        double upDown = gamepad2.left_stick_y;

       if(gamepad1.left_trigger>0||gamepad1.right_trigger>0){
           masterPowerScaleDrive = 1.0; //Range.clip(masterPowerScaleDrive + 0.2, 0.2, 1.0);
       }
       if(gamepad1.left_bumper||gamepad1.right_bumper){
           masterPowerScaleDrive = 0.4; //Range.clip(masterPowerScaleDrive - 0.2, 0.2, 1.0);
       }
       if(gamepad1.start){
           masterPowerScaleTurn = 0.35; // Range.clip(masterPowerScaleTurn + 0.2, 0.2, 1.0);
       }
        if(gamepad1.back){
            masterPowerScaleTurn = 0.2; //Range.clip(masterPowerScaleTurn - 0.2, 0.2, 1.0);
        }

       drive = drive*masterPowerScaleDrive;
       strafe = strafe*masterPowerScaleDrive;
       turn = turn*masterPowerScaleTurn;

        leftFrontPower   = Range.clip(drive+turn-strafe , -1.0, 1.0);
        rightFrontPower  = Range.clip(drive-turn+strafe , -1.0, 1.0);
        leftRearPower    = Range.clip(drive+turn+strafe , -1.0, 1.0);
        rightRearPower   = Range.clip(drive-turn-strafe , -1.0, 1.0);


        if (gamepad2.start){
            //set the front arm to an initial position
            frontArm.setPosition(0.15);
        }
        if (gamepad2.back){
            //set the front arm to an initial position
            frontArm.setPosition(0.0);
        }
        if (gamepad2.a||gamepad2.b){
            //set the front arm to an initial position
            frontArm.setPosition(0.85);
        }
        linearSlidePower = Range.clip(upDown,-1.0, 1.0);
       if(linearSlide.getCurrentPosition()>= -400&&upDown>0)
        {
            linearSlidePower = 0.2;
        }
        else if(linearSlide.getCurrentPosition()<-2900&& upDown<0)
        {
            linearSlidePower = -0.2;
        }
        if (gamepad1.a){
            capstoneLeft.setPosition(1.0);
            capstoneRight.setPosition(0.0);
        }
        if (gamepad1.y){
            capstoneLeft.setPosition(0.0);
            capstoneRight.setPosition(1.0);
        }
        if (gamepad1.b){
            capstoneLeft.setPosition(0.5);
            capstoneRight.setPosition(0.35125);
        }


/***

        //      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe



***/
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
        linearSlide.setPower(linearSlidePower);

        // Servo Control
        if(gamepad2.dpad_up||gamepad1.dpad_up)
        {
            leftHand.setPosition(0);
            rightHand.setPosition(0);}
        else if (gamepad2.dpad_down||gamepad1.dpad_down)
        {
            leftHand.setPosition(1);
            rightHand.setPosition(1);}


        // right_stick_y value will be a value between -1 and 1
        if (gamepad2.right_stick_y>0.1)
        {
            frontArm.setPosition(frontArm.getPosition()-0.1);
        }
        if (gamepad2.right_stick_y<-0.1)
        {
            frontArm.setPosition(frontArm.getPosition()+0.1);
        }
    //    rightHand.setPosition(gamepad2.left_stick_y);
    //    leftHand.setPosition(gamepad2.left_stick_y);

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Slide:",linearSlide.getCurrentPosition());
        telemetry.addData("RGB:", "Red %d, Grn %d, Blu %d",sensorColor.red(),sensorColor.green(), sensorColor.blue());
        telemetry.addData("HSV:", "Hue %.0f, Sat %.1f, Val %.0f",hsvValues[0], hsvValues[1], hsvValues[2]);
        telemetry.addData("FrontArm",frontArm.getPosition());
        telemetry.addData("MaterDrive",masterPowerScaleDrive);
        telemetry.addData("MasterTurn",masterPowerScaleTurn);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
 /*       relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

  */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "LF (%.2f), RF (%.2f), LR (%.2f), RR (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
