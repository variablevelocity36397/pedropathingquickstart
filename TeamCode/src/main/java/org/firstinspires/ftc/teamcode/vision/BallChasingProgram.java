package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Ball Tracker and Chase", group = "Vision")
public class BallChasingProgram extends LinearOpMode {

    // =========================================================
    // TUNING CONSTANTS (For 320x240 effective resolution)
    // =========================================================
    public static double KP_TURN = 0.003;
    public static double KP_FORWARD = 0.004;
    public static double DEADBAND_X = 25.0;
    public static double DEADBAND_Y = 25.0;

    // Static friction feedforward & max speed limits
    public static double MIN_POWER = 0.12;
    public static double MAX_POWER = 0.60;

    // 320x240 resolution coordinates
    public static final double CAMERA_CENTER_X = 160.0;
    public static final double TARGET_Y = 190.0; // Target near bottom of frame when ball is close

    private BallDetectionProcessor ballDetectionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        MecanumDrive robotDrive = new MecanumDrive();
        robotDrive.init(hardwareMap);

        ballDetectionProcessor = new BallDetectionProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(ballDetectionProcessor)
                .build();

        telemetry.addData("Status", "Initialized at 320x240. Waiting for Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. GET ATOMIC SNAPSHOT FROM THE PROCESSOR
            BallDetectionProcessor.BallPosition ball = ballDetectionProcessor.getBallPosition();

            double forwardPower = 0.0;
            double rotatePower = 0.0;

            if (ball.detected) {
                // 2. CALCULATE ERROR
                double errorX = ball.x - CAMERA_CENTER_X;
                double errorY = TARGET_Y - ball.y;

                // 3. PROPORTIONAL CONTROL + FEEDFORWARD FOR TURNING
                if (Math.abs(errorX) > DEADBAND_X) {
                    rotatePower = (errorX * KP_TURN) + Math.copySign(MIN_POWER, errorX);
                    rotatePower = Range.clip(rotatePower, -MAX_POWER, MAX_POWER);
                }

                // 4. PROPORTIONAL CONTROL + FEEDFORWARD FOR CHASING
                if (Math.abs(errorY) > DEADBAND_Y) {
                    forwardPower = (errorY * KP_FORWARD) + Math.copySign(MIN_POWER, errorY);
                    forwardPower = Range.clip(forwardPower, -MAX_POWER, MAX_POWER);
                }

                telemetry.addData("Tracking Status", "Chasing Target");
                telemetry.addData("Ball X", "%.1f", ball.x);
                telemetry.addData("Ball Y", "%.1f", ball.y);
                telemetry.addData("Error X", "%.1f", errorX);
                telemetry.addData("Error Y", "%.1f", errorY);
            } else {
                telemetry.addData("Tracking Status", "Ball Not Detected");
            }

            // 5. APPLY POWER TO DRIVETRAIN
            robotDrive.drive(forwardPower, 0.0, rotatePower);

            telemetry.addData("Motor Power - Forward", "%.2f", forwardPower);
            telemetry.addData("Motor Power - Turn", "%.2f", rotatePower);
            telemetry.addData("Camera FPS", "%.1f", visionPortal.getFps());
            telemetry.update();
        }
        robotDrive.drive(0.0, 0.0, 0.0);
        visionPortal.close();
    }
}