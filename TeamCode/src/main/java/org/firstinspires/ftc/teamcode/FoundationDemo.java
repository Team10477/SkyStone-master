package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Foundation Demo")
public class FoundationDemo extends LinearOpMode {

    //Hardware Devices Init
    DcMotor leftFront, rightFront, leftBack, rightBack;

    Servo leftArm, rightArm;

    RevColorSensorV3 rightColorSensor, leftColorSensor;

    DigitalChannel frontTouchSensor, backTouchSensor;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;


    double leftFrontPower;
    double rightFrontPower;
    double leftRearPower;
    double rightRearPower;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();
    double                  globalAngle;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        ElapsedTime runtime = new ElapsedTime();
//comment
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int counter = 1;

        double heading=0;
        double desired_heading, error, deltaTurnPower;
        double PROPORTIONAL_GAIN = 0.015;

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        resetAngle();// This will ensure that the direction the robot is pointing is "zero angle"


        while (opModeIsActive() && counter == 1) {

            /*
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
*/

            // turn right 90 degrees
            desired_heading = 90;
            while (heading >=90) {
                heading = getAngle();
                error = desired_heading - heading;
                deltaTurnPower = PROPORTIONAL_GAIN * error;
                StrafeWithAngle(0, deltaTurnPower);
                sleep(25);
            }

            // go straight using IMU to maintain a 0 degree heading
            runtime.reset();
            desired_heading = 0;
            while ( runtime.seconds() < 3) {
                //end condition is for example, a fixed time, say 3 seconds, OR a touch sensor changing from 0 to 1 OR a color sensor changes from no color to a RED color
                heading = getAngle();
                error = desired_heading - heading;
                deltaTurnPower = PROPORTIONAL_GAIN * error;
                StrafeWithAngle(0.4, deltaTurnPower);
            }

            // Turn left to 60 degrees
            desired_heading = -60;
            while (heading <=-60) {
                heading = getAngle();
                error = desired_heading - heading;
                deltaTurnPower = PROPORTIONAL_GAIN * error;
                StrafeWithAngle(0, deltaTurnPower);
                sleep(25);
            }

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

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
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

    public void StrafeWithAngle(double strafe, double turn){

        leftFrontPower   = Range.clip(turn-strafe , -1.0, 1.0);
        rightFrontPower  = Range.clip(-turn+strafe , -1.0, 1.0);
        leftRearPower    = Range.clip(turn+strafe , -1.0, 1.0);
        rightRearPower   = Range.clip(-turn-strafe , -1.0, 1.0);
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }
}
