package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmDown;
import org.firstinspires.ftc.teamcode.commands.DriveCmd;
import org.firstinspires.ftc.teamcode.commands.IntakeCmd;
import org.firstinspires.ftc.teamcode.commands.ArmUp;
import org.firstinspires.ftc.teamcode.commands.MoveLinearSlide;
import org.firstinspires.ftc.teamcode.commands.MoveWristLeft;
import org.firstinspires.ftc.teamcode.commands.MoveWristRight;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;
import org.firstinspires.ftc.teamcode.subsystems.ImuSub;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSub;
import org.firstinspires.ftc.teamcode.subsystems.WristSub;


@TeleOp(name = "Tele-op 2024-25")
public class TeleOp25 extends CommandOpMode {

    private GamepadEx driverOp;
    private GamepadEx toolOp;

    private DrivetrainSub drive;
    private DriveCmd driveCmd;
    private boolean fieldCentric = false;
    private ImuSub robotImu;

    private IntakeSub intake;
    private IntakeCmd intakeIn;
    private IntakeCmd intakeOut;
    private IntakeCmd intakeOff;

    private WristSub wrist;
    private MoveWristLeft wristLeft;
    private MoveWristRight wristRight;

    private ArmSub armSub;
    private ArmUp armUp;
    private ArmDown armDown;

    private LinearSlideSub linearSlideSub;
    private MoveLinearSlide linearSlideUp;
    private MoveLinearSlide linearSlideDown;
    private MoveLinearSlide linearSlideOff;



    @Override
    public void initialize() {
        robotImu = new ImuSub(hardwareMap, telemetry);

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);


        // Drive
        drive = new DrivetrainSub(hardwareMap, telemetry);
        driveCmd = new DriveCmd(drive, driverOp, robotImu::getAngle, this::getFieldCentric);

        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(this::toggleFieldCentric));

        register(drive);
        drive.setDefaultCommand(driveCmd);


        // Intake
        intake = new IntakeSub(hardwareMap, telemetry);
        intakeIn = new IntakeCmd(intake, 1);
        intakeOut = new IntakeCmd(intake, -1);
        intakeOff = new IntakeCmd(intake, 0);

        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(intakeOut);
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenReleased(intakeOff);
        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(intakeIn);
        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenReleased(intakeOff);


        // Wrist
        wrist = new WristSub(hardwareMap, telemetry);
        wristRight = new MoveWristRight(wrist);
        wristLeft = new MoveWristLeft(wrist,telemetry);

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(wristLeft);
        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(wristRight);


        // Arm
        armSub = new ArmSub(hardwareMap, telemetry);
        armUp = new ArmUp(armSub, toolOp, telemetry);
        armDown = new ArmDown(armSub, toolOp, telemetry);

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(armUp);
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(armDown);


        // Linear Slide
        linearSlideSub = new LinearSlideSub(hardwareMap, telemetry);
        linearSlideUp = new MoveLinearSlide(linearSlideSub, telemetry, 0.5);
        linearSlideDown = new MoveLinearSlide(linearSlideSub, telemetry, -0.5);
        linearSlideOff = new MoveLinearSlide(linearSlideSub, telemetry, 0);

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(linearSlideUp);
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(linearSlideDown);
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(linearSlideOff);
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(linearSlideOff);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Field Centric?", fieldCentric);
        telemetry.update();
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
        if (fieldCentric) {
            robotImu.resetAngle();
        }
    }
}
