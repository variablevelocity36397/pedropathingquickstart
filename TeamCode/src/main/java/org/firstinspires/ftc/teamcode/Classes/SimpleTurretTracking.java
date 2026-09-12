package org.firstinspires.ftc.teamcode.Classes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SimpleTurret", group = "Testing")

public class SimpleTurretTracking extends LinearOpMode {

    private DcMotorEx turretmtr; // the motor that turns the turntable/turret
    private Limelight3A limelight;
    final double ticks_degree = (8192.0 * 8.0) / 360;
    final double min_degree = -90;
    final double max_degree = 90;
    final int AprilTagPipeline = 0;

    public double getCurrentDeg() {
        return turretmtr.getCurrentPosition() / ticks_degree;
    }
    //private Servo hood;

    @Override
    public void runOpMode() {

        turretmtr = hardwareMap.get(DcMotorEx.class, "turretmtr");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretmtr.setDirection(DcMotorSimple.Direction.FORWARD);
        turretmtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretmtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretmtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(AprilTagPipeline);
        limelight.start();

        if (limelight.isRunning()) {
            telemetry.addLine("Limelight is running");
        }


        double turretpower = 0.5;
        double slowdown_zone = 20;

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            double tx = 0;
            int id = 0;

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    id = fiducial.getFiducialId();
                    if (id == 24) {
                        tx = fiducial.getTargetXDegrees();
                        telemetry.addData("TX", tx);
                        break;
                    }
                }
            }

            double currentDeg = getCurrentDeg();

            if (id == 24) {
                double targetPower;

                if (tx > 3) {
                    targetPower = -turretpower;
                } else if (tx < -3) {
                    targetPower = turretpower;
                } else {
                    targetPower = 0;
                }

                // --- slowdown zone logic ---
                double distToMax = max_degree - currentDeg;
                double distToMin = currentDeg - min_degree;

                if (targetPower > 0 && distToMax < slowdown_zone) {
                    double scale = distToMax / slowdown_zone;
                    scale = Math.max(scale, 0);
                    targetPower *= scale;
                }

                if (targetPower < 0 && distToMin < slowdown_zone) {
                    double scale = distToMin / slowdown_zone;
                    scale = Math.max(scale, 0);
                    targetPower *= scale;
                }
                // --- end slowdown zone ---

                turretmtr.setPower(targetPower);

            } else {
                turretmtr.setPower(0);
            }

            telemetry.addData("Degree: ", currentDeg);

            telemetry.update();
        }
    }
}