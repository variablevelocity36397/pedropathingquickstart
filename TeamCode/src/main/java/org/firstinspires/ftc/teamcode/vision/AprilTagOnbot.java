package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.List;

public class AprilTagOnbot {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- MEASURE YOUR PRINTED TAGS ---
    // Enter the exact width of the black square of your printed tags in inches.
    // The SDK needs this exact number to calculate distance and bearing angles.
    private final double TAG_SIZE_INCHES = 4.0;

    public void initAprilTag(HardwareMap hardwareMap) {

        // 1. Build the custom BioBuzz tag library (Tags 30-45)
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();

        // Loop through and automatically register your Red and Blue team IDs
        for (int id = 30; id <= 45; id++) {
            libraryBuilder.addTag(id, "BioBuzz Tag " + id, TAG_SIZE_INCHES, DistanceUnit.INCH);
        }

        AprilTagLibrary bioBuzzLibrary = libraryBuilder.build();

        // 2. Initialize the AprilTag Processor with the custom library
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(545.584, 545.584, 350.588, 223.150)
                .setTagLibrary(bioBuzzLibrary) // <-- Tells SDK to use your BioBuzz tags
                .build();

        aprilTag.setDecimation(2);

        // 3. Start the Vision Portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag != null) {
            return aprilTag.getDetections();
        }
        return null;
    }

    public void closePortal() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}