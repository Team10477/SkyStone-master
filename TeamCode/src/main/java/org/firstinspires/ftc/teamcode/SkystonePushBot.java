package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class SkystonePushBot extends HardwarePushbot {

    public Servo pickupArm = null;
    public DigitalChannel touchSensorPickupArm = null;
    public WebcamName webcamName = null;

    public SkystonePushBot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        super.init(hwMap);
        pickupArm = hwMap.get(Servo.class, "front_arm");
        touchSensorPickupArm = hwMap.get(DigitalChannel.class, "touch_sensor_arm");
        webcamName = hwMap.get(WebcamName.class, "Webcam1");


    }

    public void setPosition(double position) {
        pickupArm.setPosition(position);

        if (position > 0) {
            resetIfArmTouches();
        }

    }

    public void resetIfArmTouches() {
        if ( !touchSensorPickupArm.getState()) {      // Go to rest position if touch sensor is pressed.
            pickupArm.setPosition(0);
        }
    }

}
