package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "AprilTagOdometryTest")

public class ConceptAprilTag extends LinearOpMode {
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;
    float fieldX;
    float fieldY;
    float adjustedYaw;
    float odometryX;
    float odometryY;
    float odometryYaw;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    private LaserRangeSensor2 rangeSensor;
    private LaserRangeSensor3 rangeSensor2;

    @Override
    public void runOpMode() {
        rangeSensor = hardwareMap.get(LaserRangeSensor2.class, "dist_sensor1");
        rangeSensor2 = hardwareMap.get(LaserRangeSensor3.class, "dist_sensor2");

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();
                /** for averaging values to try and get a more consistent reading*/
                /*
                int[] distances1 = new int[5];
                int[] distances2 = new int[5];
                int sum1 = 0;
                int sum2 = 0;

                for(int i = 0; i < 5; i++){
                    distances1[i] = rangeSensor.getDistance();
                    sum1 += rangeSensor.getDistance();
                    distances2[i] = rangeSensor2.getDistance();
                    sum2 += rangeSensor2.getDistance();
                }
                int distance1 = sum1 / 5;
                int distance2 = sum2 / 5;
                 */

                int distance1 = rangeSensor.getDistance();
                int distance2 = rangeSensor2.getDistance();


                int robotWidth = 245; // mm from center of dist sensor 1 to center of dist sensor 2 (double check)

                telemetry.addData("Distance from left distance sensor (mm)", distance2);
                telemetry.addData("Distance from right distance sensor (mm)", distance1);
                double estAngle = 0;

                /**
                    for values of distance1 and distance2 that are close together, this should be almost 0
                    conversely values of distance1 and distance2 that are far apart should return a large number
                 */
                if(distance1 > distance2){
                    estAngle = Math.atan2((distance1-distance2),robotWidth) * (180/Math.PI);
                }
                else{
                    estAngle = Math.atan2((distance2-distance1),robotWidth) * (180/Math.PI);
                }

                telemetry.addData("Estimated angle offset (deg): ", estAngle);
                telemetry.update();


                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        visionPortal.close();

    }

    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortal = myVisionPortalBuilder.build();
    }


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        AprilTagDetection myAprilTagDetection;
        for (AprilTagDetection myAprilTagDetection_item : currentDetections) {
            myAprilTagDetection = myAprilTagDetection_item;

            double distInCm = (((int)myAprilTagDetection.ftcPose.y) * 2.54);
            distInCm += rangeSensor.getDistance() / 10;
            distInCm /= 2;
            telemetry.addLine("Adjusted distance: " + distInCm);

            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
                telemetry.addLine("" + JavaUtil.formatNumber(myAprilTagDetection.metadata.fieldPosition.get(0), 6, 1) + "" + JavaUtil.formatNumber(myAprilTagDetection.metadata.fieldPosition.get(1), 6, 1));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", myAprilTagDetection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", myAprilTagDetection.center.x, myAprilTagDetection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
    public static AprilTagLibrary getCenterStageTagLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF( 60.25f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF( 60.25f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF( 60.25f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF( 60.25f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF( 60.25f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(60.25f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF( -70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion( 0.7071f, 0, 0, -7.071f, 0))
                .addTag (8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.7071f, 0, 0, -7.071f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF( -70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion( 0.7071f, 0, 0, -7.071f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion( 0.7071f, 0, 0, -7.071f, 0))
                .build();
    }

}   // end class
