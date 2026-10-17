package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpEasy", group = "TeleOp")

public class TeleopEasy extends LinearOpMode {

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
    private Servo hood;

    final double ticks_degree = (8192.0 * 8.0) / 360;
    final double min_degree = -210;
    final double max_degree = 150;

    public double getCurrentDeg() {
        return turretmtr.getCurrentPosition() / ticks_degree;
    }
    //private Servo hood;

    // sets both halves of the intake together so the servo never gets forgotten
    private void setIntake(double power) {
        intakemotor.setPower(power);
        intakeservo.setPower(power);
    }

    private void addControlsTelemetry() {
        telemetry.addLine("--------  GAMEPAD CONTROLS --------");
        telemetry.addLine("Press A to toggle intake");
        telemetry.addLine("Press Dpad Down to toggle intake reverse");
        telemetry.addLine("Press B to shoot (press again to cancel)");
        telemetry.addLine("Press Y to toggle shooter velocity (1960 / 2520)");
        telemetry.addLine("Left joystick to move, Right joystick to turn");
        telemetry.addLine("Hold the Right Trigger to drive at 20% speed");
        telemetry.addLine("Left and Right Bumper to manually move turret");
        telemetry.addLine("Dpad Left and Right to move hood");
    }

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
        hood = hardwareMap.get(Servo.class, "hood");
        leftshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeservo.setDirection(DcMotorSimple.Direction.REVERSE);
        turretmtr.setDirection(DcMotorSimple.Direction.FORWARD);
        turretmtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretmtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretmtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch
        leftshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch
        rightshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        addControlsTelemetry();
        telemetry.update();

        int intakeState = 0; // 0 = off, 1 = forward, -1 = reverse
        int shootStep = 0;   // 0 = idle, 1 = spinning up, 2 = feeding, 3 = kicking
        boolean shooterOn = false;
        final double velTolerance = 50; // how close to target velocity counts as "up to speed"
        final double shootervelNear = 1960; // 4200 RPM (28 ticks/rev)
        final double shootervelFar = 2520; // 5400 RPM (28 ticks/rev)
        final double servokickerkick = 0.05;
        final double servokickerrest = 0.456;
        final double gateopen = 0.5;
        final double gateclose = 0.25;
        double shootervel = shootervelNear;
        ElapsedTime shootertimer = new ElapsedTime();
        ElapsedTime kickertimer = new ElapsedTime();
        double slowdown_zone = 12;
        double currentTurretPower = 0; // still needs to persist outside the loop
        double stopRampRate = 0.35;    // how slowly it decays to 0 on release — tune this
        double hoodPos = 0.5;

        gate.setPosition(gateclose);
        hood.setPosition(hoodPos);
        servokicker.setPosition(servokickerrest);
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
                if (intakeState == 1) {
                    intakeState = 0;
                } else {
                    intakeState = 1;
                }
            }
            if (gamepad1.dpadDownWasPressed()) { // toggles intake opposite direction
                if (intakeState == -1) {
                    intakeState = 1;
                } else {
                    intakeState = -1;
                }
            }

            // shoot sequence: each step waits for its condition without blocking the loop
            if (gamepad1.bWasPressed()) {
                if (shootStep == 0) {
                    gate.setPosition(gateclose);
                    leftshooter.setVelocity(shootervel);
                    rightshooter.setVelocity(shootervel);
                    shooterOn = true;
                    shootStep = 1;
                } else { // pressing b mid-sequence cancels it, shooters keep spinning
                    intakeState = 0;
                    servokicker.setPosition(servokickerrest);
                    gate.setPosition(gateclose);
                    shootStep = 0;
                }
            }
            if (shootStep == 1 && (leftshooter.getVelocity() >= shootervel - velTolerance
                    || rightshooter.getVelocity() >= shootervel - velTolerance)) {
                gate.setPosition(gateopen);
                intakeState = 1;
                shootertimer.reset();
                shootStep = 2;
            }
            if (shootStep == 2 && shootertimer.seconds() >= 0.85) {
                servokicker.setPosition(servokickerkick);
                kickertimer.reset();
                shootStep = 3;
            }
            if (shootStep == 3 && kickertimer.seconds() >= 0.75) {
                servokicker.setPosition(servokickerrest);
                shootStep = 0;
            }

            if (gamepad1.yWasPressed()) { // click y to toggle shooter velocity
                if (shootervel == shootervelNear) {
                    shootervel = shootervelFar;
                } else {
                    shootervel = shootervelNear;
                }
                if (shooterOn) { // if shooter is running, velocity is immediately updated
                    leftshooter.setVelocity(shootervel);
                    rightshooter.setVelocity(shootervel);
                }
            }

            setIntake(intakeState);

            double currentDeg = getCurrentDeg();
            double targetPower = 0;

            if (gamepad1.left_bumper) {
                targetPower = 0.4;
            }
            if (gamepad1.right_bumper) {
                targetPower = -0.4;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                if (hoodPos >= 0.1) {
                    hoodPos = hoodPos - 0.1;
                    hood.setPosition(hoodPos);
                }
            }
            if (gamepad1.dpadRightWasPressed()) {
                if (hoodPos <= 0.9) {
                    hoodPos = hoodPos +0.1;
                    hood.setPosition(hoodPos);
                }
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


            String[] stepNames = {"IDLE", "SPINNING UP", "FEEDING", "KICKING"};
            String intakeName = intakeState == 1 ? "ON" : intakeState == -1 ? "REVERSE" : "OFF";

            addControlsTelemetry();
            telemetry.addLine("--------  STATUS --------");
            telemetry.addData("Shoot step: ", stepNames[shootStep]);
            telemetry.addData("Target shooter velocity: ", shootervel);
            telemetry.addData("Left shooter velocity: ", leftshooter.getVelocity());
            telemetry.addData("Right shooter velocity: ", rightshooter.getVelocity());
            telemetry.addData("Turret position (deg): ", currentDeg);
            telemetry.addData("Hood position: ", hoodPos);
            telemetry.addData("Intake: ", intakeName);
            telemetry.addData("Slow mode: ", speed < 1 ? "ON" : "OFF");
            telemetry.update();
        }
    }

    /*
     * ---- DRIVER-ASSIST IDEAS (on hold, not implemented) ----
     * Free buttons: X, Dpad Up, Left Trigger, stick buttons, back/options, all of gamepad2.
     *
     * - Auto-aim turret: hold a button and the turret tracks the goal AprilTag (reuse vision/AutoAlign.java)
     * - Turret recenter: one press drives the turret back to 0 deg using the slowdown-zone logic
     * - Hood presets: near/far buttons set hood position and shootervel together
     * - Rapid fire: repeat shoot steps 2-3 N times to empty the robot
     * - Rumble when ready: gamepad1.rumble() when the shooter reaches target velocity
     * - Field-centric toggle + heading reset: use the Pinpoint heading (same localizer as pedro/Constants.java)
     * - Heading lock: while held, hold the current heading so strafing doesn't rotate the robot
     * - Auto-park / return-to-shoot-spot: one press runs a short PedroPathing path from
     *   Constants.create(hardwareMap); any stick movement cancels it
     * - Panic stop: stop shooters, intake, and the shoot sequence, and reset the servos
     * - Move turret/hood/shooting to gamepad2 so driver 1 only drives
     */
}