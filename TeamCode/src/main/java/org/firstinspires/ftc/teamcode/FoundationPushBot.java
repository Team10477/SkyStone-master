package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class FoundationPushBot extends HardwarePushbot {

    public Servo leftArm = null;
    public Servo rightArm = null;
    public DigitalChannel touchSensor = null;

    public FoundationPushBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        super.init(hwMap);
        leftArm = hwMap.get(Servo.class, "left_hand");
        rightArm = hwMap.get(Servo.class, "right_hand");
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");

    }

    public void setPosition(double position) {
       leftArm.setPosition(position);
       rightArm.setPosition(position);

    }


}
