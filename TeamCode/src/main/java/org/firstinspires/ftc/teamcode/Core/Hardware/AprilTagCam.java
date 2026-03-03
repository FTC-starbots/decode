package org.firstinspires.ftc.teamcode.Core.Hardware;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
@TeleOp
public class AprilTagCam extends LinearOpMode implements InterfaceSimple  {
    double ErroX;
    double ErroY;
    double ErroYaw;
   public double LeituraX;
    public double LeituraY;
    public  double LeituraYaw;
    public double LeituraPixels;
    public int NumTagDetec;
    public boolean tagdetected;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 8, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, -90, 0, 0);
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    AprilTagDetection myAprilTagDetection;

    AprilTagDetection detection;


    public void setup(HardwareMap hardware) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardware.get(WebcamName.class, "camera"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

       // builder.setAutoStartStreamOnBuild(true);
        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    @Override
    public void runOpMode() throws InterruptedException {
        if(!opModeIsActive()){
            visionPortal.close();
        }
    }

    /**
     * Add telemetry about AprilTag detections.
     */

    @Override
    public void update() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()){
            NumTagDetec = 0;
            LeituraX =0;
            LeituraY =0;
            LeituraYaw = 0;
            LeituraPixels = 0;
            tagdetected = false;
            telemetry.addData("lista vazia", tagdetected);

        }
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
               NumTagDetec = detection.id;
               LeituraX = detection.robotPose.getPosition().x;
               LeituraY = detection.robotPose.getPosition().y;
               LeituraYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
               LeituraPixels = detection.center.x;
               tagdetected = true;
                telemetry.addData("validationON", tagdetected);
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

}   // end class
