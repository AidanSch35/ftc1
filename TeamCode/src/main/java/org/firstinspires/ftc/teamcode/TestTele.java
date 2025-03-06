package org.firstinspires.ftc.teamcode;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;



/*
ROTATION PORT 1 control hub
Grabber port 2 control hub
right side pivot port 3 control hub
left side pivot port 1 expansion hub
*/
class Claw {
    private final double STEP = 0.001;
    private Servo clawRotateServo;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private Servo grabberServo;
    private Telemetry telemetry;
    public Claw(Servo clawRotateServo, Servo leftArmServo, Servo rightArmServo, Servo grabberServo, Telemetry telemetry) {
        this.clawRotateServo = clawRotateServo;
        this.leftArmServo = leftArmServo;
        this.rightArmServo = rightArmServo;
        this.grabberServo = grabberServo;
        this.telemetry = telemetry;
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);
    }
    public void reset() {
        leftArmServo.setPosition(0);
        rightArmServo.setPosition(0);
    }
    public void grab() {
        grabberServo.setPosition(0);
        telemetry.addData("grabbing", true);
    }
    public void open() {
        telemetry.addData("opening", true);
        grabberServo.setPosition(1);
    }
    public void rotateArm(double amt) {
        rightArmServo.setPosition(rightArmServo.getPosition()+amt);
        leftArmServo.setPosition(leftArmServo.getPosition()+amt);
    }
    public void spinClaw(double amt) {
        clawRotateServo.setPosition(clawRotateServo.getPosition()+amt);
        telemetry.addData("Claw Pos:", clawRotateServo.getPosition());
    }

    public void handle(Gamepad gamepad) {
        if(gamepad.a) {
            spinClaw(STEP);

        }
        if(gamepad.b) spinClaw(-STEP);
        if(gamepad.x) rotateArm(STEP);
        if(gamepad.y) rotateArm(-STEP);
        if(gamepad.left_trigger > 0) grab();
        if(gamepad.right_trigger > 0) open();
    }

}
class HSlides {
    private Servo left;
    private Servo right;
    private final double STEP = 0.01;

    // these servos are giving me cancer.
    // True ranges are
    // LEFT SERVO 0.050 -> 0.308
    // RIGHT SERVO 0.508->0.783
    // normalizes value in range [0, 1] to range [x, y]
    public static double mapToRange(double value, double min, double max) {
        if (value < 0 || value > 1) {
            throw new IllegalArgumentException("Value must be in range [0, 1]");
        }
        double epsilon = 1e-10; // Small offset to avoid boundaries
        return min + value * (max - min) * (1 - 2 * epsilon) + epsilon * (max - min);
    }
    private final double left_min = 0.050, left_max = 0.308, right_min = 0.508, right_max = 0.783;
    public double pos = 0;

    double clamp(double x, double lower, double higher) {
        x = max(x, lower);
        x = min(x, higher);
        return x;
    }
    public HSlides(Servo left, Servo right) {
        this.left = left;
        this.right = right;
        left.setDirection(Servo.Direction.REVERSE);
        right.setDirection(Servo.Direction.FORWARD);
    }
    public void handle(Gamepad gamepad) {

        if(gamepad.a) {
            pos += STEP;
        }
        if(gamepad.b) {
            pos -= STEP;
        }
        pos = clamp(pos, 0, 1);

        double mappedLeft = mapToRange(pos, left_min, left_max);
        double mappedRight = mapToRange(pos, right_min, right_max);
        left.setPosition(mappedLeft);
        right.setPosition(mappedRight);


        //if(gamepad.a) extend();
        //if(gamepad.b) reset();
    }
    public void print(Telemetry telemetry, Gamepad gamepad) {
        telemetry.addData("H Slide Position:", pos);
        telemetry.addData("Left HSlide Pos:", left.getPosition());
        telemetry.addData("Left HSlide Pos:", right.getPosition());
        telemetry.addData("A pressed?", gamepad.a);
        telemetry.addData("B pressed?", gamepad.b);
    }
    public void reset() {
        left.setPosition(0);
        right.setPosition(0);
    }
    public void extend() {
        left.setPosition(1);
        right.setPosition(1);
    }
}

class VSlides {
    private DcMotor left;
    private DcMotor right;
    private int pos = 50;
    private final int ticks = 1;

    public VSlides(DcMotor left, DcMotor right) {
        this.left = left;
        this.right = right;
        this.left.setDirection(DcMotor.Direction.REVERSE);
        this.right.setDirection(DcMotor.Direction.FORWARD);
        this.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void handle(Gamepad gamepad) {
        // vertical positions
        if(gamepad.x){
            pos += ticks;
        }
        if(gamepad.y){
            pos -= ticks;
        }
        left.setTargetPosition(pos);
        right.setTargetPosition(pos);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(0.7);
        right.setPower(0.7);
    }
    public void print(Telemetry telemetry, Gamepad gamepad) {
        telemetry.addData("V Slide Position:", pos);
        telemetry.addData("Actual Left V Slide Pos", left.getCurrentPosition());
        telemetry.addData("Actual Right V Slide Pos", right.getCurrentPosition());
        telemetry.addData("X pressed?", gamepad.x);
        telemetry.addData("Y pressed?", gamepad.y);
    }
    public void reset() {
        pos = 50;
    }
}
class Drive {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public Drive(DcMotor leftFrontDrive, DcMotor leftBackDrive, DcMotor rightFrontDrive, DcMotor rightBackDrive) {
        this.leftFrontDrive = leftFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.rightBackDrive = rightBackDrive;
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void go(Gamepad gamepad) {
        double axial = -gamepad.left_stick_y;  // Forward/backward
        double lateral = gamepad.left_stick_x;  // Strafing
        double yaw = gamepad.right_stick_x;  // Rotation

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize drivetrain power
        double max = max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = max(max, Math.abs(leftBackPower));
        max = max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
// EXPANSION HUB PORT ZERO
@TeleOp(name = "TestTele2", group = "Linear OpMode")
public class TestTele2 extends RobotLinearOpMode {
    private void printGamepad() {
        // Left Stick
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);

        // Right Stick
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);

        // Triggers
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);

        // Bumpers
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);

        // Buttons
        telemetry.addData("A Button", gamepad1.a);
        telemetry.addData("B Button", gamepad1.b);
        telemetry.addData("X Button", gamepad1.x);
        telemetry.addData("Y Button", gamepad1.y);

        // D-Pad
        telemetry.addData("D-Pad Up", gamepad1.dpad_up);
        telemetry.addData("D-Pad Down", gamepad1.dpad_down);
        telemetry.addData("D-Pad Left", gamepad1.dpad_left);
        telemetry.addData("D-Pad Right", gamepad1.dpad_right);

        // Start, Back, and Guide Buttons
        telemetry.addData("Start", gamepad1.start);
        telemetry.addData("Back", gamepad1.back);
        telemetry.addData("Guide (PS/Home)", gamepad1.guide);

        // Stick Buttons
        telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
        telemetry.addData("Right Stick Button", gamepad1.right_stick_button);

    }
    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo leftHSlideServo = null;
    private Servo rightHSlideServo = null;
    private DcMotor leftVSlideDrive = null;
    private DcMotor rightVSlideDrive = null;
    private Servo clawRotateServo;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private Servo grabberServo;

    Drive drive;
    HSlides hslides;
    VSlides vslides;
    Claw claw;
    private void initiate() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftHSlideServo = hardwareMap.get(Servo.class, "leftHSlideServo");
        rightHSlideServo = hardwareMap.get(Servo.class, "rightHSlideServo");
        rightVSlideDrive = hardwareMap.get(DcMotor.class, "rightVSlideDrive");
        leftVSlideDrive = hardwareMap.get(DcMotor.class, "leftVSlideDrive");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        clawRotateServo = hardwareMap.get(Servo.class, "clawRotateServo");
        drive = new Drive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);
        hslides = new HSlides(leftHSlideServo, rightHSlideServo);
        vslides = new VSlides(leftVSlideDrive, rightVSlideDrive);

        claw = new Claw(clawRotateServo, leftArmServo, rightArmServo, grabberServo, telemetry);
        //reset();
    }


    private void reset() {
        hslides.reset();
        vslides.reset();
        claw.reset();
    }
    @Override
    public void runOpMode() {
        initiate();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        clawRotateServo.setPosition(1);
        while (opModeIsActive()) {
            if(gamepad1.left_bumper) {
                break;
            }
            printGamepad();
            drive.go(gamepad1);
            claw.handle(gamepad1);
            //hslides.handle(gamepad1);
            //hslides.print(telemetry, gamepad1);
            //vslides.handle(gamepad1);
            //vslides.print(telemetry, gamepad1);

            // Debugging telemetry
            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();
        }
        reset();
    }
}

