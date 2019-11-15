package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Skystone Pickup Blue Bridge")
public class SkystonePickupBlueBridge extends LinearOpMode {

    SkystonePushBot robot = new SkystonePushBot();
    SkyStoneIdentification skyStoneIdentification = new SkyStoneIdentification();

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private static final double MAXIMUM_POSITION = 1.0;

    private static final double MINIMUM_POSITION = 0;

    double leftFrontPower;
    double rightFrontPower;
    double leftRearPower;
    double rightRearPower;

    double heading;
    double LEFT = 0.2;
    double RIGHT = -0.2;

    boolean isRed = false;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // The IMU sensor object
    BNO055IMU imu;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle;


     @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.setWheelDirectionReverse();
        initGryo();
        skyStoneIdentification.initCamera(hardwareMap, robot.webcamName);
         enableColorSensor();

        waitForStart();

        robot.pickupArm.setPosition(0);
       int counter = 1;
        while (opModeIsActive() && counter == 1) {
            robot.setWheelPower(0.5);
            sleep(1250);

            robot.stopWheels();

            boolean isVisible = skyStoneIdentification.identifyTarget(telemetry, robot, isRed);

            if (!isVisible) {
                robot.setWheelDirectionReverse();
                robot.setWheelPower(0.15);
                sleep(3000);

              /*  robot.setWheelPower(0.15);
                sleep(500);*/

                robot.pickupArm.setPosition(0.85);   // Arm to grab.
                sleep(1000);
                robot.stopWheels();

                robot.setWheelDirectionForward();   // Go backward after picking up the block.
                robot.setWheelPower(0.35);
                sleep(1200);
                robot.stopWheels();

                //

            } else {

                robot.setWheelPowerForSide(0.15);
                sleep(500);

                robot.setWheelDirectionReverse();
                robot.setWheelPower(0.15);
                sleep(3000);

           }

            robot.pickupArm.setPosition(0.85);   // Arm to grab.
            sleep(1000);
            robot.stopWheels();

            robot.setWheelDirectionForward();   // Go backward after picking up the block.
            robot.setWheelPower(0.35);
            sleep(1200);
            robot.stopWheels();

           turnLeft90WithGryro();     // Turn Right 90 degrees.
           sleep(200);
           robot.stopWheels();

           robot.setWheelDirectionReverse();   // Go forward crossing the bridge.*/
           stopAtBlue(false);
           robot.setWheelPower(0.5);
           sleep(100);
           robot.stopWheels();


           robot.pickupArm.setPosition(0);
           sleep(600);
           robot.stopWheels();

           //  Come back under the bridge
           robot.setWheelDirectionForward();   // Go backward after picking up the block.
           stopAtBlue(false);

            counter++;
       }
//
    }

    /**
     * Initialize Gyro
     */
     public void initGryo() {
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        resetAngle(); // set heading to zero
    }

    /**
     * Reset Angle at start.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Turn Right 90 degrees.
     */
    public void turnLeft90WithGryro() {
        heading = getAngle();
        while (heading<85.0) {
            heading = getAngle();
            setMecanumPower(0, Math.PI/4, (LEFT*Math.abs(91-heading)/90)+0.1);
        }
    }

    /**
     * Get Current Angle.
     * @return
     */
     public double getAngle() {
         // We experimentally determined the Z axis is the axis we want to use for heading angle.
         // We have to process the angle because the imu works in euler angles so the Z axis is
         // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
         // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

         Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

         double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

         if (deltaAngle < -180)
             deltaAngle += 360;
         else if (deltaAngle > 180)
             deltaAngle -= 360;

         globalAngle += deltaAngle;

         lastAngles = angles;

         return globalAngle;
    }

    /**
     *  Set Wheel power as it turns.
     * @param Vd
     * @param Theta
     * @param turn
     */
    public void setMecanumPower(double Vd, double Theta, double turn){

        leftFrontPower   = Range.clip(Vd*Math.sin(Theta)+turn , -1.0, 1.0) ;
        rightFrontPower  = Range.clip(Vd*Math.cos(Theta)-turn , -1.0, 1.0) ;
        leftRearPower    = Range.clip(Vd*Math.cos(Theta)+turn , -1.0, 1.0) ;
        rightRearPower   = Range.clip(Vd*Math.sin(Theta)-turn , -1.0, 1.0) ;

        robot.leftFrontWheel.setPower(leftFrontPower);
        robot.rightFrontWheel.setPower(rightFrontPower);
        robot.leftBackWheel.setPower(leftRearPower);
        robot.rightBackWheel.setPower(rightRearPower);
    }

    public void stopAtBlue(boolean colorFound) {
        while (colorFound == false) {
            Color.RGBToHSV((int)(robot.colorSensor.red() * 8), (int)(robot.colorSensor.green() *8), (int)(robot.colorSensor.blue() * 8), hsvValues);

            float hue = hsvValues[0];

            float saturation = hsvValues[1];

            telemetry.addData("Color Blue", hue);
            telemetry.update();

            boolean redHue = (hue < 60 || hue > 320) && (saturation > 0.5);
            boolean blueHue = (hue > 180 && hue < 240) && (saturation > 0.5);

            if (blueHue)  {
                robot.stopWheels();
                colorFound = true;
            }else {
                robot.setWheelPower(0.5);
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
