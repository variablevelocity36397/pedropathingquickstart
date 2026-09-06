/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Optimized goBilda AprilTag", group = "Concept")
public class AprilTagOnbot extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initAprilTag();

        // Example of forcing the tag size to 5 inches (converted to meters for the SDK)
        double tagSizeMeters = 4.0 * 0.0254;

        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        // --- GLOBAL SHUTTER OPTIMIZATION ---
        // Wait for the camera to start streaming before sending exposure controls
        while (opModeIsActive() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        // Set to manual exposure to eliminate motion blur
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }

        // 10-15ms is usually the sweet spot for FTC field lighting
        exposureControl.setExposure(12, TimeUnit.MILLISECONDS);

        // Adjust gain to compensate for the fast shutter speed
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(150);
        // ------------------------------------

        aprilTag.setDecimation(2);

        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();
            sleep(20);
        }

        // Save CPU resources when camera is no longer needed.
        visionPortal.close();
        // Define your current settings
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                // Updated scaled intrinsics for 640x480 resolution
                .setLensIntrinsics(545.584,545.584,350.588,223.150)
                .build();


        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Update these to match exactly where your camera is mounted relative to the robot's center
        final double CAMERA_X_OFFSET = 2.0;
        final double CAMERA_Y_OFFSET = 4.0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Calculate the tag's true position relative to the center of the robot
                double robotX = detection.ftcPose.x + CAMERA_X_OFFSET;
                double robotY = detection.ftcPose.y + CAMERA_Y_OFFSET;

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("Camera to Tag: X %6.1f Y %6.1f (inch)", detection.ftcPose.x, detection.ftcPose.y));
                telemetry.addLine(String.format("Robot to Tag:  X %6.1f Y %6.1f (inch)", robotX, robotY));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));
            }
            telemetry.addData("Camera FPS", visionPortal.getFps());
        }
    }
}