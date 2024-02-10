package org.firstinspires.ftc.teamcode.autocode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PlaceLinePixel;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "RedAuto2")

public class RedAuto2 extends PlaceLinePixel {

    @Override

    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        try {

            initTfod();

            waitForStart();

            if (opModeIsActive()) {

                RobotMoveFarward();
                TimeUnit.MILLISECONDS.sleep(250);

                RobotStop();
                TimeUnit.MILLISECONDS.sleep(250);

                armUp();

                TimeUnit.SECONDS.sleep(4);
                telemetryTfod();
                telemetry.update();

                if (Location1 == true) {
                    PixelLocation1();
                } else if (Location2 == true) {
                    PixelLocation2();
                } else if (Location3 == true) {
                    PixelLocation3();
                } else {
                    Location2 = true;
                    PixelLocation2();
                }

                TimeUnit.MILLISECONDS.sleep(500);

                RedLocation2();

                TimeUnit.MILLISECONDS.sleep(100);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }
}
