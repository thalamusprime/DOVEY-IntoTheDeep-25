package org.firstinspires.ftc.teamcode.ftc6205.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class DoveyTelemetry {

    public void sendTelemetry(Telemetry telemetry, AprilTagProcessor tagProcessor) throws InterruptedException {
        String tag11_x = "";
        String tag11_y = "";
        String tag11_z = "";
        String tag12_x = "";
        String tag12_y = "";
        String tag12_z = "";
        String tag13_x = "";
        String tag13_y = "";
        String tag13_z = "";
        String tag14_x = "";
        String tag14_y = "";
        String tag14_z = "";
        String tag15_x = "";
        String tag15_y = "";
        String tag15_z = "";
        String tag16_x = "";
        String tag16_y = "";
        String tag16_z = "";

//        telemetry.addLine(String.format(
//                "Throttle y:x:rz | %5.2f : %5.2f : %5.2f",
//                y,
//                x,
//                rz
//        ));

//        telemetry.addLine(String.format(
//                "REF|BOT|PID %5.2f %5.2f %5.2f",
//                refHeading,
//                botHeading,
//                pidOutput
//        ));

//        telemetry.addLine(String.format(
//                "ENCODER L|R %5.2f %5.2f"//,
//                encLeftValue * 0.017255, // 0.0075
//                encRightValue * 0.017255
//        ));

//        telemetry.addLine(String.format(
//                "DIST F|B %5.2f"//,
//                distFront.getDistance(DistanceUnit.INCH)
//        ));

        //Vision Processing: April Tag detection
        int tags = tagProcessor.getDetections().size();
        if (tagProcessor.getDetections().size() > 0) {
            ArrayList tagList = tagProcessor.getDetections();
            if (tagList != null) {
                for (Object tagItem: tagList) {
                    AprilTagDetection tag = (AprilTagDetection) tagItem;
                    if (tag.id == 11) {
                        tag11_x = String.format("%7.2f", tag.ftcPose.x);
                        tag11_y = String.format("%7.2f", tag.ftcPose.y);
                        tag11_z = String.format("%7.2f", tag.ftcPose.z);
                    }
                    if (tag.id == 12) {
                        tag12_x = String.format("%7.2f", tag.ftcPose.x);
                        tag12_y = String.format("%7.2f", tag.ftcPose.y);
                        tag12_z = String.format("%7.2f", tag.ftcPose.z);
                    }
                    if (tag.id == 13) {
                        tag13_x = String.format("%7.2f", tag.ftcPose.x);
                        tag13_y = String.format("%7.2f", tag.ftcPose.y);
                        tag13_z = String.format("%7.2f", tag.ftcPose.z);
                    }
                    if (tag.id == 14) {
                        tag14_x = String.format("%7.2f", tag.ftcPose.x);
                        tag14_y = String.format("%7.2f", tag.ftcPose.y);
                        tag14_z = String.format("%7.2f", tag.ftcPose.z);
                    }
                    if (tag.id == 15) {
                        tag15_x = String.format("%7.2f", tag.ftcPose.x);
                        tag15_y = String.format("%7.2f", tag.ftcPose.y);
                        tag15_z = String.format("%7.2f", tag.ftcPose.z);
                    }
                    if (tag.id == 16) {
                        tag16_x = String.format("%7.2f", tag.ftcPose.x);
                        tag16_y = String.format("%7.2f", tag.ftcPose.y);
                        tag16_z = String.format("%7.2f", tag.ftcPose.z);
                    }
                }
            }

            telemetry.addLine("---");
            telemetry.addLine("ID (11)" + tag11_x +  tag11_y + tag11_z);
            telemetry.addLine("ID (12)" + tag12_x +  tag12_y + tag12_z);
            telemetry.addLine("ID (13)" + tag13_x +  tag13_y + tag13_z);
            telemetry.addLine("ID (14)" + tag14_x +  tag14_y + tag14_z);
            telemetry.addLine("ID (15)" + tag15_x +  tag15_y + tag15_z);
            telemetry.addLine("ID (16)" + tag16_x +  tag16_y + tag16_z);
        }
        telemetry.update();
    }


}
