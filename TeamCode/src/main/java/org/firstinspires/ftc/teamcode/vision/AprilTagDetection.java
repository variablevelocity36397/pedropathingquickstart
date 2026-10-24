/*package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

public class AprilTagDetection {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- MEASURE YOUR PRINTED TAGS ---
    // Enter the exact width of the black square of your printed tags in inches.
    // The SDK needs this exact number to calculate distance and bearing angles.
    private final double TAG_SIZE_INCHES = 5.1;

    public void initAprilTag(HardwareMap hardwareMap) {

        // 1. Build the custom BioBuzz tag library (Tags 30-45)
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();

        // Loop through and automatically register your Red and Blue team IDs
        for (int id = 30; id <= 45; id++) {
            libraryBuilder.addTag(id, "BioBuzz Tag " + id, TAG_SIZE_INCHES, DistanceUnit.INCH);
        }

        AprilTagLibrary bioBuzzLibrary = libraryBuilder.build();

        //offset

        double camOffsetX = 0.0;
        double camOffsetY = 0.0;
        double camOffsetZ = 0.0;

        Position cameraPosition = new Position(DistanceUnit.INCH, camOffsetX, camOffsetY, camOffsetZ, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        //offset

        // 2. Initialize the AprilTag Processor with the custom library
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(270.260, 260.270, 173.207, 120.382)
                .setTagLibrary(bioBuzzLibrary) // <-- Tells SDK to use your BioBuzz tags
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(2);

        // 3. Start the Vision Portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(320, 240));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> getDetections() {
        if (aprilTag != null) {
            return aprilTag.getDetections();
        }
        return null;


    }

    //camera fps
    public float getCameraFps() {
        if (visionPortal != null) {
            return visionPortal.getFps();
        }
        return 0.0f;
    }
    //camera fps code end

    public void closePortal() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", getDetections().size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}*/