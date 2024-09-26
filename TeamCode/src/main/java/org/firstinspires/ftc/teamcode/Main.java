package org.firstinspires.ftc.teamcode;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Main")
public class Main extends LinearOpMode {
    //INIT
    DcMotorEx motor;

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            motor = hardwareMap.get(DcMotorEx.class, "motor0");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive()) {
                motor.setPower(0.2);
                sleep(2500);
                motor.setPower(0);
                break;
                // useless
            }
        }
    }
}
