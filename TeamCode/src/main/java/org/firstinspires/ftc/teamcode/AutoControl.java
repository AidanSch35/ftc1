package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoControl {
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private Servo leftHSlideServo, rightHSlideServo; //hslides
    private DcMotor leftVSlideDrive, rightVSlideDrive; //vslides
    private ElapsedTime runtime = new ElapsedTime();
    private static final double TICKS_PER_CM = 10;//FIX
    private static final double TICKS_PER_180 = 0;

    public AutoControl(DcMotor leftFrontDrive, DcMotor leftBackDrive, DcMotor rightFrontDrive, DcMotor rightBackDrive, Servo rightHSlideServo, Servo leftHslideServo, DcMotor rightVSlideDrive, DcMotor leftVSlideDrive ) {
        this.leftFrontDrive = leftFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.rightBackDrive = rightBackDrive;

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.rightHSlideServo = rightHSlideServo;
        this.leftHSlideServo = leftHslideServo;

        leftHSlideServo.setDirection(Servo.Direction.REVERSE);
        rightHSlideServo.setDirection(Servo.Direction.REVERSE);

        this.rightVSlideDrive = rightVSlideDrive;
        this.leftVSlideDrive = leftVSlideDrive;

        this.leftVSlideDrive.setDirection(DcMotor.Direction.REVERSE);
        this.rightVSlideDrive.setDirection(DcMotor.Direction.FORWARD);
        this.leftVSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightVSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void encoderDrive(double speed, int leftFrontTarget, int leftBackTarget, int rightFrontTarget, int rightBackTarget, int timeout) {
        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        rightBackDrive.setTargetPosition(rightBackTarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        runtime.reset();
        while (runtime.seconds() < timeout && leftFrontDrive.isBusy() && rightFrontDrive.isBusy()) {}

        stopMotors();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveStraight(double speed, double distance, int timeout) {
        int target = (int) (distance * TICKS_PER_CM);
        encoderDrive(speed, target, target, target, target, timeout);
    }

    public void strafe(double speed, double distance, int timeout) {
        int target = (int) (distance * TICKS_PER_CM);
        encoderDrive(speed, target, -target, -target, target, timeout);
    }

    public void turn(double speed, double degrees, int timeout) {
        int target = (int) (degrees * TICKS_PER_180);
        encoderDrive(speed, -target, -target, target, target, timeout);
    }

    public void dispense() {
        //fill in
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightVSlideDrive.setPower(0);
        leftVSlideDrive.setPower(0);
    }

    private void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
