package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;


@Autonomous
public class AutoAlignWithoutController extends OpMode {

    private final AprilTagDetection aprilTagDetection = new AprilTagDetection();
    private final MecanumDrive drive = new MecanumDrive();

    //----------------------PD Variables--------------------------//

    double kP = 0.005;
    double error = 0;
    double lastError = 0;
    double goalX = 0; // offset here//
    double angleTolerance = 1;
    double kD = 0.002;
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
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        aprilTagDetection.initAprilTag(hardwareMap); // Passes hardwareMap explicitly
        drive.init(hardwareMap);
        telemetry.addLine("Initialized and starting runOpMode");
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
        lastTime = getRuntime();
        timer.reset();
    }


    @Override
    public void loop() {
        //=---------Get Mecanum Drive inputs=----------//
        forward = 0.0;
        strafe = 0.0;
        rotate = 0.0;

        //---------- Get apriltag info------------//

        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detections = aprilTagDetection.getDetections();



        if (timer.seconds() > 1.0) {

            if (detections != null && !detections.isEmpty()) {
                List<Integer> myTeamIds = this.getMyTeamIds();
                for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : detections) {
                    if (!myTeamIds.contains((Integer) detection.id)) {
                        telemetry.addData("skipping April tag", detection.id);
                        continue;
                    }
                    // ADD THIS SAFETY CHECK: Skip if the 3D position hasn't calculated yet
                    if (detection.ftcPose == null) {
                        continue;
                    }

                    double roll = detection.ftcPose.roll;
                    boolean isCellUp = Math.abs(roll) < 90.0;

                    telemetry.addData("Detected Tag ID", detection.id);
                    telemetry.addData("Roll Angle", "%.1f deg", roll);
                    telemetry.addData("CELL State", isCellUp);

                    if (!isCellUp) {
                        continue;
                    }
                    boolean foundValidTag = true;
                    //added camera fps
                    telemetry.addData("Camera FPS", aprilTagDetection.getCameraFps());

                    // adding for offset
                    telemetry.addData("X offset (Left/Right)", detection.ftcPose.x);
                    telemetry.addData("Y offset (Forward/Back)", detection.ftcPose.y);
                    telemetry.addData("Z Offset", detection.ftcPose.z);
                    //end of offset

                    telemetry.addData("detected April tag", detection.id);
                    error = goalX - detection.ftcPose.bearing;
                    if (Math.abs(error) < angleTolerance) {
                        rotate = 0;
                    } else {
                        double pTerm = error * kP;

                        curTime = getRuntime();
                        double dT = curTime - lastTime;
                        if (dT <= 0) dT = 0.001;
                        double dTerm = ((error - lastError) / dT) * kD;

                        rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                        lastError = error;
                        lastTime = curTime;
                    }
                    telemetry.addData("parameters updated to ", rotate);
                    break;

                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
                //TODO : inspect the else condition
                rotate = 0.0;
            }

        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        //drive motors//
        drive.drive(forward, strafe, rotate);

        //-----telemetry------//

        if (detections != null) {
            if (timer.seconds() > 1) {
                telemetry.addLine("AUTO ALIGN");
            }
            telemetry.addData("ERROR", error);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addLine("--------------------");
        telemetry.addData("Tuning P", kP);
        telemetry.addData("Tuning D", kD);
    }
}
