package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class OpticalTracker {
    SparkFunOTOS myOtos;

    // Get a reference to the sensor
//    myOtos =hardwareMap.get(SparkFunOTOS .class,"sensor_otos");
//
//    // All the configuration for the OTOS is done in this helper method, check it out!
//    configureOtos();

    {
        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
    }
}