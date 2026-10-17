/*package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@TeleOp
public class AprilTagAutoAlignMecanumDrivetrain extends OpMode {

    private final AprilTagOnbot aprilTagOnbot = new AprilTagOnbot();
    private final MecanumDrive drive = new MecanumDrive();

    //----------------------PD Variables--------------------------//

    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; // offset here//
    double angleTolerance = 0.4;
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    //------------------Driving Setup----------------------//

    double forward, strafe, rotate;

    //----------------Controller based PID tuning----------------//

    double[] stepSizes = {1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 2;

    public List<Integer> red = List.of(
            30, 31, 32, 33, 34, 35, 36, 37
    );

    public List<Integer> blue = List.of(
            38, 39, 40, 41, 42, 43, 44, 45
    );

    public static boolean IS_TEAM_BLUE = false;

    public List<Integer> getMyTeamIds() {
        return IS_TEAM_BLUE ? blue : red;
    }


    @Override
    public void init() {
        aprilTagOnbot.initAprilTag(hardwareMap); // Passes hardwareMap explicitly
        drive.init(hardwareMap);
        telemetry.addLine("Initialized and starting runOpMode");

        // DELETE THIS LINE: aprilTagOnbot.runOpMode();
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }


    @Override
    public void loop() {
        //=---------Get Mecanum Drive inputs=----------//
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        //---------- Get apriltag info------------//

        List<AprilTagDetection> detections = aprilTagOnbot.getDetections();

        if (gamepad1.left_trigger > 0.3) {

            if (detections != null && !detections.isEmpty()) {
                List<Integer> myTeamIds = this.getMyTeamIds();
                for (AprilTagDetection detection : detections) {
                    if (!myTeamIds.contains((Integer) detection.id)) {
                        telemetry.addData("skipping April tag", detection.id);
                        continue;
                    }
                    telemetry.addData("detected April tag", detection.id);
                    error = goalX - detection.ftcPose.bearing;
                    if (Math.abs(error) < angleTolerance) {
                        rotate = 0;
                    } else {
                        double pTerm = error + kP;

                        curTime = getRuntime();
                        double dT = curTime - lastTime;
                        double dTerm = ((error - lastError) / dT) * kD;

                        rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                        lastError = error;
                        lastTime = curTime;
                    }
                    telemetry.addData("parameters updated to ", rotate);

                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
                //TODO : inspect the else condition
                rotate = rotate;
            }

        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        //drive motors//
        drive.drive(forward, strafe, rotate);

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        //-----telemetry------//

        if (detections != null) {
            if (gamepad1.left_trigger > 0.3) {
                telemetry.addLine("AUTO ALIGN");
            }
            telemetry.addData("ERROR", error);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addLine("--------------------");
        telemetry.addData("Tuning P", "%.4f (D-PAD L/R) kP");
        telemetry.addData("Tuning D", "%.4f (D-PAD U/D) kD");
        telemetry.addData("Step Size", "%.4f(B Button)", stepSizes[stepIndex]);

    }
}
*/