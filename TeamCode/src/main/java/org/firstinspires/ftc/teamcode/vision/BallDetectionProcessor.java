package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import java.util.ArrayList;
import java.util.List;

public class BallDetectionProcessor implements VisionProcessor {

    // Immutable snapshot class to prevent thread-safety race conditions (torn reads)
    public static class BallPosition {
        public final boolean detected;
        public final double x;
        public final double y;

        public BallPosition(boolean detected, double x, double y) {
            this.detected = detected;
            this.x = x;
            this.y = y;
        }
    }

    // OpenCV matrices declared once at the class level to prevent memory leaks
    private final Mat workingMat = new Mat();
    private final Mat blurredMat = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();

    // Target 320x240 size
    private final Size targetSize = new Size(320, 240);
    private final Size blurSize = new Size(5, 5);

    // Single volatile reference guarantees atomic updates and reads across threads
    private volatile BallPosition currentPosition = new BallPosition(false, -1, -1);

    // Adjust these HSV values to match your ball's color
    private final Scalar lowerBound = new Scalar(20, 100, 100); // Example: Yellow
    private final Scalar upperBound = new Scalar(30, 255, 255);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // 1. DECIMATE / ENSURE 320x240 RESOLUTION WITHOUT ALIASING MEMORY
        Mat inputMat = frame;
        if (frame.cols() != 320 || frame.rows() != 240) {
            Imgproc.resize(frame, workingMat, targetSize, 0, 0, Imgproc.INTER_LINEAR);
            inputMat = workingMat;
        }

        // 2. BLUR IN RGB FIRST, THEN CONVERT TO HSV
        Imgproc.GaussianBlur(inputMat, blurredMat, blurSize, 0);
        Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        // 3. THRESHOLD COLOR
        Core.inRange(hsvMat, lowerBound, upperBound, mask);

        // 4. FIND CONTOURS
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea && area > 150) {
                maxArea = area;
                largestContour = contour;
            }
        }

        // 5. CALCULATE CENTER COORDINATES ATOMICALLY
        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            if (moments.get_m00() != 0) {
                double centerX = moments.get_m10() / moments.get_m00();
                double centerY = moments.get_m01() / moments.get_m00();
                currentPosition = new BallPosition(true, centerX, centerY);
            } else {
                currentPosition = new BallPosition(false, -1, -1);
            }
        } else {
            currentPosition = new BallPosition(false, -1, -1);
        }

        // Release contour native memory
        for (MatOfPoint contour : contours) {
            contour.release();
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    // Atomic getter for the OpMode to read
    public BallPosition getBallPosition() {
        return currentPosition;
    }
}