package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Skystone Pickup Red")
public class SkystonePickupRed extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();
    SkyStoneIdentification skyStoneIdentification = new SkyStoneIdentification();
    Servo pickUpArm;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private static final double MAXIMUM_POSITION = 1.0;

    private static final double MINIMUM_POSITION = 0;


     @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        pickUpArm = hardwareMap.get(Servo.class, "front_arm");
        robot.setWheelDirectionReverse();
        skyStoneIdentification.initCamera(hardwareMap);
        waitForStart();
        robot.setWheelPower(0.25);
        sleep(2000);

        robot.stopWheels();

        skyStoneIdentification.identifyTarget(telemetry,robot);

        robot.setWheelDirectionReverse();
        robot.setWheelPower(0.25);
        sleep(550);
        robot.stopWheels();















































        pickUpArm.setPosition(1);

    }

}
