package org.firstinspires.ftc.teamcode;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class HSlides {
    private Servo left;
    private Servo right;
    private final double SERVO_MIN = 0.431;
    private final double SERVO_MAX = 0.7;
    private double pos = SERVO_MIN;
    private final double STEP = 0.001;

    double clamp(double x, double lower, double higher) {
        x = max(x, lower);
        x = min(x, higher);
        return x;
    }
    public HSlides(Servo left, Servo right) {
        this.left = left;
        this.right = right;
        left.setDirection(Servo.Direction.REVERSE);
        right.setDirection(Servo.Direction.REVERSE);
    }
    public void handle(Gamepad gamepad) {
        if(gamepad.a) {
            pos += STEP;
        }
        if(gamepad.b) {
            pos -= STEP;
        }
        pos = clamp(pos, 0, 1);
        left.setPosition(pos);
        right.setPosition(pos);

    }
    public void print(Telemetry telemetry, Gamepad gamepad) {
        telemetry.addData("H Slide Position:", pos);
        telemetry.addData("Left HSlide Pos:", left.getPosition());
        telemetry.addData("Left HSlide Pos:", right.getPosition());
        telemetry.addData("A pressed?", gamepad.a);
        telemetry.addData("B pressed?", gamepad.b);
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
public class TestTele extends RobotLinearOpMode {

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




    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftHSlideServo = hardwareMap.get(Servo.class, "leftHSlideServo");
        rightHSlideServo = hardwareMap.get(Servo.class, "rightHSlideServo");
        rightVSlideDrive = hardwareMap.get(DcMotor.class, "rightVSlideDrive");
        leftVSlideDrive = hardwareMap.get(DcMotor.class, "leftVSlideDrive");

        Drive drive = new Drive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);
        HSlides hslides = new HSlides(leftHSlideServo, rightHSlideServo);
        VSlides vslides = new VSlides(leftVSlideDrive, rightVSlideDrive);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if(gamepad1.left_bumper) return;

            drive.go(gamepad1);
            hslides.handle(gamepad1);
            hslides.print(telemetry, gamepad1);
            vslides.handle(gamepad1);
            vslides.print(telemetry, gamepad1);
            // Debugging telemetry
            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();
        }
    }
}