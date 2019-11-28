package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationPushBot extends HardwarePushbot {

    public Servo leftArm = null;
    public Servo rightArm = null;
    public DigitalChannel touchSensor = null;
    public DigitalChannel touchSensorFront = null;

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
        touchSensorFront = hwMap.get(DigitalChannel.class, "touch_sensor_front");

        rightArm.setDirection(Servo.Direction.FORWARD);
        leftArm.setDirection(Servo.Direction.REVERSE);

    }

    public void setPosition(double position) {
       leftArm.setPosition(position);
       rightArm.setPosition(position);

    }

    public boolean isArmsUp() {
        if ((leftArm.getPosition() > 0) || (rightArm.getPosition() > 0))
            return false;
        return true;

    }

    public boolean isArmsDown() {
        if ((leftArm.getPosition() == 1) && (rightArm.getPosition() == 1))
            return true;
        return false;

    }
}
