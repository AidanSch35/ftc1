package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TestAuto", group = "Linear Opmode")
public class TestAuto extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo leftHSlideServo = null;
    private Servo rightHSlideServo = null;
    private DcMotor leftVSlideDrive = null;
    private DcMotor rightVSlideDrive = null;
    private AutoControl ac;

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

        ac = new AutoControl(leftFrontDrive,  leftBackDrive,  rightFrontDrive, rightBackDrive,  rightHSlideServo,  leftHSlideServo, rightVSlideDrive,  leftVSlideDrive );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Autonomous sequence
        if(gamepad1.a){
            ac.moveStraight(0.5, 10, 5);
            ac.moveStraight(0.5, -10, 5);
            ac.strafe(0.5, 10, 3);
            ac.strafe(0.5, -10, 3);
            ac.turn(0.5, 90, 3); // Turn 90 degrees
            //ac.dispense(); // Activate dispenser
            //ac.stopMotors(); // Stop all movement
        }


        telemetry.addData("Status", "Completed");
        telemetry.update();
    }
}