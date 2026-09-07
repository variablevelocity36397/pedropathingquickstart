package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {

    private DcMotor intakemotor;   // intake motor
    private DcMotor frwheel; // front right wheel
    private DcMotor flwheel; // front left wheel
    private DcMotor brwheel; // back right wheel
    private DcMotor blwheel; // back left wheel
    private DcMotorEx leftshooter; // one of the 2 shooter motors
    private DcMotorEx rightshooter; // the second of the 2 shooter motors
    private DcMotorEx turretmtr; // the motor that turns the turntable/turret
    private CRServo intakeservo; // the middle intake powered via servo
    private Servo servokicker; // the servo kicker
    private Servo gate;

    final double ticks_degree = (8192.0 * 8.0) / 360;
    final double min_degree = -210;
    final double max_degree = 150;

    public double getCurrentDeg() {
        return turretmtr.getCurrentPosition() / ticks_degree;
    }
    //private Servo hood;

    @Override
    public void runOpMode() {

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");
        frwheel = hardwareMap.get(DcMotor.class, "frwheel");
        flwheel = hardwareMap.get(DcMotor.class, "flwheel");
        flwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        brwheel = hardwareMap.get(DcMotor.class, "brwheel");
        blwheel = hardwareMap.get(DcMotor.class, "blwheel");
        blwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftshooter = hardwareMap.get(DcMotorEx.class, "leftshooter");
        rightshooter = hardwareMap.get(DcMotorEx.class, "rightshooter");
        turretmtr = hardwareMap.get(DcMotorEx.class, "turretmtr");
        servokicker = hardwareMap.get(Servo.class, "servokicker");
        gate = hardwareMap.get(Servo.class, "gate");
        //hood = hardwareMap.get(Servo.class, "hood");
        leftshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeservo.setDirection(DcMotorSimple.Direction.REVERSE);
        turretmtr.setDirection(DcMotorSimple.Direction.FORWARD);
        turretmtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretmtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretmtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch
        leftshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch

        telemetry.addLine("--------  GAMEPAD CONTROLS --------");
        telemetry.addLine("Press A to toggle intake");
        telemetry.addLine("Press B to kick (auto-returns after 0.75s)");
        telemetry.addLine("Press X to toggle both flywheel motors");
        telemetry.addLine("Press Y to toggle shooter power (1.0 / 0.7)");
        telemetry.addLine("Press Dpad Up to toggle the gate open/closed");
        telemetry.addLine("Press Dpad Down to intake opposite direction");
        telemetry.addLine("Left joystick to move, Right joystick to turn");
        telemetry.addLine("Hold the Right Trigger to drive at 20% speed");
        telemetry.addLine("Left and Right Bumper to manually move turret");



        telemetry.update();

        boolean flag = true;
        boolean flag3 = false;
        boolean flag4 = true;
        boolean flag5 = true;
        boolean reverseFlag = true;
        final double servokickerkick = 0.05;
        final double servokickerrest = 0.456;
        final double gateopen = 0.5;
        final double gateclose = 0.25;
        double shootervel = 750;
        ElapsedTime kickTimer = new ElapsedTime();
        double slowdown_zone = 12;
        double currentTurretPower = 0; // still needs to persist outside the loop
        double stopRampRate = 0.35;    // how slowly it decays to 0 on release — tune this





        gate.setPosition(gateclose);


        waitForStart();


        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y ;       // left joystick control forward/back movement
            double x = gamepad1.left_stick_x ;     // right joystick control right/left turning
            double rx = gamepad1.right_stick_x * 0.8;
            double speed = 1;
            if (gamepad1.right_trigger > 0.1) {
                speed = 0.2;
            }

            double leftFrontPower = (y + x + rx) ;
            double leftBackPower = (y - x + rx) ;
            double rightFrontPower = (y - x - rx) ;
            double rightBackPower = (y + x - rx) ;

            flwheel.setPower(leftFrontPower * speed);
            blwheel.setPower(leftBackPower * speed);
            frwheel.setPower(rightFrontPower * speed);
            brwheel.setPower(rightBackPower * speed);

            if (gamepad1.aWasPressed()) {  // click the button a to toggle the intake
                if (flag) {
                    intakemotor.setPower(1);
                    intakeservo.setPower(1);
                } else {
                    intakeservo.setPower(0);
                    intakemotor.setPower(0);
                }
                flag = !flag;
            }
            if (gamepad1.dpadDownWasPressed()) { // toggles intake opposite direction
                if (reverseFlag) {
                    intakemotor.setPower(-1);
                    intakeservo.setPower(-1);
                } else {
                    intakemotor.setPower(1);
                    intakeservo.setPower(1);
                    flag = false;
                }
                reverseFlag = !reverseFlag;
            }
            if (gamepad1.bWasPressed()) {
                kickTimer.reset();
                servokicker.setPosition(servokickerkick);
                kickTimer.reset();
            }
            if (kickTimer.seconds() >= 0.75) {
                servokicker.setPosition(servokickerrest);
            }

            if (gamepad1.yWasPressed()) { // click y to toggle shooter power level
                if (flag4) {
                    shootervel = 1200;
                } else {
                    shootervel = 750;
                }
                flag4 = !flag4;
                if (flag3) { // if shooter is running and power is changed, power is immediately updated
                    leftshooter.setVelocity(shootervel);
                    rightshooter.setVelocity(shootervel);
                }
            }
            if (gamepad1.xWasPressed()) { // click the button x to toggle the shooter
                if (!flag3) {
                    leftshooter.setVelocity(shootervel);
                    rightshooter.setVelocity(shootervel);
                }
                else {
                    rightshooter.setPower(0);
                    leftshooter.setPower(0);
                }
                flag3 = !flag3;
            }
            if (gamepad1.dpadUpWasPressed()) { // click dpad up to switch between states gate open and close
                if (flag5) {
                    gate.setPosition(gateopen);
                }
                else {
                    gate.setPosition(gateclose);
                }
                flag5 = !flag5;
            }


            double currentDeg = getCurrentDeg();
            double targetPower = 0;

            if (gamepad1.left_bumper) {
                targetPower = 0.4;
            }
            if (gamepad1.right_bumper) {
                targetPower = -0.4;
            }

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

            if (targetPower == 0) {
                // button released to ramp power down slowly
                if (currentTurretPower > 0) {
                    currentTurretPower = Math.max(currentTurretPower - stopRampRate, 0);
                } else if (currentTurretPower < 0) {
                    currentTurretPower = Math.min(currentTurretPower + stopRampRate, 0);
                }
            } else {
                currentTurretPower = targetPower;
            }

            turretmtr.setPower(currentTurretPower);


            telemetry.addLine("--------  GAMEPAD CONTROLS --------");
            telemetry.addLine("Press A to toggle intake");
            telemetry.addLine("Press B to kick (auto-returns after 0.75s)");
            telemetry.addLine("Press X to toggle both flywheel motors");
            telemetry.addLine("Press Y to toggle shooter power (1.0 / 0.7)");
            telemetry.addLine("Press Dpad Up to toggle the gate open/closed");
            telemetry.addLine("Press Dpad Down to intake opposite direction");
            telemetry.addLine("Left joystick to move, Right joystick to turn");
            telemetry.addLine("Hold the Right Trigger to drive at 20% speed");
            telemetry.addLine("Left and Right Bumper to manually move turret");
            telemetry.addData("Current Position: ", getCurrentDeg());
            telemetry.addData("Shooter power: ", shootervel);
            telemetry.update();
        }
    }
}