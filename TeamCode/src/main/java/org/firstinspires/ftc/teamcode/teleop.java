package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop", group = "Linear OpMode")

public class teleop extends LinearOpMode {

    private DcMotor intakemotor;   // intake motor
    private DcMotor frwheel; // front right wheel
    private DcMotor flwheel; // front left wheel
    private DcMotor brwheel; // back right wheel
    private DcMotor blwheel; // back left wheel
    private DcMotor shooter1; // one of the 2 shooter motors
    private DcMotor shooter2; // the second of the 2 shooter motors
    //private DcMotor turretmtr; // the motor that turns the turntable/turret
    private CRServo intakeservo; // the middle intake powered via servo
    private Servo servokicker; // the servo kicker
    private Servo gate;
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
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        //turretmtr = hardwareMap.get(DcMotor.class, "turretmtr");
        servokicker = hardwareMap.get(Servo.class, "servokicker");
        gate = hardwareMap.get(Servo.class, "gate");
        //hood = hardwareMap.get(Servo.class, "hood");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeservo.setDirection(DcMotorSimple.Direction.REVERSE);


        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Dont touch

        telemetry.addLine("--------  GAMEPAD1 CONTROLS --------");
        telemetry.addLine("Press A to toggle intake");
        telemetry.addLine("Press B to kick (auto-returns after 0.75s)");
        telemetry.addLine("Press X to toggle both flywheel motors");
        telemetry.addLine("Press Y to toggle shooter power (1.0 / 0.7)");
        telemetry.addLine("Left joystick to move, Right joystick to turn");
        telemetry.addLine("Hold the Right Trigger to drive at 20% speed");
        telemetry.addLine();
        telemetry.addLine("--------  GAMEPAD2 CONTROLS --------");
        telemetry.addLine("Press A to toggle intake direction");
        telemetry.addLine("Press X to toggle the gate open/closed");


        telemetry.update();

        boolean flag = true;
        boolean flag2 = true;
        boolean flag3 = true;
        boolean flag4 = true;
        boolean flag5 = true;
        boolean reverseFlag = true;
        final double servokickerrest = 0.05;
        final double servokickerkick = 0.456;
        final double gateopen = 0.5;
        final double gateclose = 0.1;
        double shooterpower = 1;
        ElapsedTime kickTimer = new ElapsedTime();
        boolean isKicking = false;


        gate.setPosition(gateclose);
        waitForStart();


        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y ;       // left joystick control forward/back movement
            double x = -gamepad1.left_stick_x ;     // right joystick control right/left turning
            double rx = -gamepad1.right_stick_x * 0.8;
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
            if (gamepad2.aWasPressed()) { // toggles intake opposite direction
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
            if (gamepad1.bWasPressed() && !isKicking) { // press B to kick
                servokicker.setPosition(servokickerkick);
                kickTimer.reset();
                isKicking = true;
            }

            if (isKicking && kickTimer.seconds() >= 0.75) { // auto-return after 0.75s
                servokicker.setPosition(servokickerrest);
                isKicking = false;
            }

            if (gamepad1.yWasPressed()) { // click y to toggle shooter power level
                if (flag4) {
                    shooterpower = 1.0;
                } else {
                    shooterpower = 0.7;
                }
                flag4 = !flag4;
                if (flag3) { // if shooter is running and power is changed, power is immediately updated
                    shooter1.setPower(shooterpower);
                    shooter2.setPower(shooterpower);
                }
            }
            if (gamepad1.xWasPressed()) { // click the button x to toggle the shooter
                if (flag3) {
                    shooter1.setPower(shooterpower);
                    shooter2.setPower(shooterpower);
                }
                else {
                    shooter2.setPower(0);
                    shooter1.setPower(0);
                }
                flag3 = !flag3;
            }
            if (gamepad2.xWasPressed()) { // click gamepad2 x to switch between states gate open and close
                if (flag5) {
                    gate.setPosition(gateopen);
                }
                else {
                    gate.setPosition(gateclose);
                }
                flag5 = !flag5;
            }

        }
    }
}