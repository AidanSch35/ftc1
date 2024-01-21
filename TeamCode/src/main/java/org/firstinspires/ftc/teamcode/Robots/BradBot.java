package org.firstinspires.ftc.teamcode.Robots;

import static org.apache.commons.math3.util.FastMath.PI;
import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.max;
import static org.apache.commons.math3.util.FastMath.min;
import static org.firstinspires.ftc.teamcode.Components.Arm.ArmStates.*;
import static org.firstinspires.ftc.teamcode.Components.Arm.DROP_POS;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.types.PathType;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.Arm.ArmStates;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.Hanger;
import org.firstinspires.ftc.teamcode.Components.Hopper;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.Preloader;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Components.Twrist;
import org.firstinspires.ftc.teamcode.Components.Ultrasonics;
import org.firstinspires.ftc.teamcode.Components.Wrist;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/** Warren Robot class to contain all the season's functions */
public class BradBot extends BasicRobot {
  Arm arm;
  Claw claw;
  CVMaster cv;
  Hanger hanger;
  Intake intake;
  Launcher launcher;
  Lift lift;
  Magazine magazine;
  Preloader preloader;
  public SampleMecanumDrive roadrun;
  Twrist twrist;
  Ultrasonics ultras;
  Wrist wrist;
  Path path;

  MecanumDrive drive;

  /**
   * Instatiates all the hardware and sets up initial states of some software Logs that this
   * function is being called to general surface
   *
   * @param p_op opMode
   * @param p_is_Teleop is the program a teleop program
   */
  public BradBot(LinearOpMode p_op, boolean p_is_Teleop) {
    super(p_op, p_is_Teleop);
    LOGGER.setLogLevel(RFLogger.Severity.INFO);
    LOGGER.log("Initializing Components!");
    arm = new Arm();
    if (!isTeleop) {
      cv = new CVMaster();
    }
    claw = new Claw();
    magazine = new Magazine();
    hanger = new Hanger();
    intake = new Intake();
    launcher = new Launcher();
    lift = new Lift();
    preloader = new Preloader();
    roadrun = new SampleMecanumDrive(p_op.hardwareMap);
    twrist = new Twrist();
    //        ultras = new Ultrasonics();
    ultras = new Ultrasonics();
    wrist = new Wrist();
    path = new Path();
  }

  public int getSpikePos() {
    return cv.getPosition() - 1;
  }

  public int getRightSpikePos() {
    return cv.getRightPosition() - 1;
  }

  public int getBlueSpikePos() {
    return cv.getBluePosition() - 1;
  }

  public int getBlueRightSpikePos() {
    return cv.getBlueRightPosition() - 1;
  }

  /**
   * starts the intake, for autonomous, intake.update() will handle the rest Logs that this function
   * called to general surface
   */
  public void startIntakeAuto() {
    if (queuer.queue(true, Intake.IntakeStates.STOPPED.getState())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("intaking until 2 pixels are stored");
      }
    }
  }

  public void preloadAuto() {
    if (queuer.queue(false, !preloader.getLoaded())) {
      if (!queuer.isExecuted()) {
        preloader.deposit();
        LOGGER.log("depositing preload");
      }
    }
  }

  public void purpurAuto() {
    if (queuer.queue(
        true, !Lift.LiftMovingStates.LOW.state&&abs(lift.getCurrentPosition() - 150) < 80)) {
      if (DROP.getState()) {
        arm.purpurPigzl();
        Lift.LiftMovingStates.LOW.state = false;
        lift.iterateDown();
        LOGGER.log("purpuring");
      }
    }
  }

  public void dropAuto(int servo) {
    if (queuer.queue(false, queuer.isExecuted())) {
//      if (!queuer.isExecuted()) {
        if (servo == 1) {
          claw.moveOne(false);
        } else {
          claw.moveTwo(false);
        }

        LOGGER.log("dropping claw " + servo);
      }
//    }
  }

  public void upAuto() {
    if (queuer.queue(true, DROP.getState())) {
      if (!Lift.LiftMovingStates.LOW.state) {
        lift.setPosition(Lift.LiftPositionStates.LOW_SET_LINE);
        intake.stopIntake();
        arm.flipTo(DROP);
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        LOGGER.log("ocrud");
      }
    }
  }

  public void lowAuto() {
    if (queuer.queue(true, Wrist.WristStates.DROP.getState())) {
      if (!Lift.LiftMovingStates.LOW.state&&currentPose.getX()>12) {
        lift.setPosition(Lift.LiftPositionStates.LOW_SET_LINE);
        intake.stopIntake();
        arm.flipTo(DROP);
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        LOGGER.log("ocook");
      }
    }
  }

  public void grabAuto() {
    if (queuer.queue(false, Claw.clawStates.GRAB.getState())) {
      if (!queuer.isExecuted()) {
        arm.flipTo(GRAB);
        wrist.flipTo(Wrist.WristTargetStates.LOCK);
        LOGGER.log("grabbing");
      }
      if( GRAB.getState() && !Claw.clawStates.GRAB.getState()){
        claw.moveTwo(true);
        claw.moveOne(true);
        Claw.clawTargetStates.GRAB.setStateTrue();
      }
    }
  }

  public void loadAuto() {
    if (queuer.queue(true, preloader.getLoaded())) {
      if (!queuer.isExecuted()) {
        preloader.load();
        LOGGER.log("loading preload");
      }
    }
  }

  public void flipAuto() {
    if (queuer.queue(true, ArmStates.DROP.getState())) {
      if (!queuer.isExecuted()) {
        arm.flipTo(DROP);
        lift.iterateUp();
      }
    }
  }

  public void resetAuto() {
    if (queuer.queue(false, lift.getCurrentPosition()<10)) {
      if (!Lift.LiftMovingStates.AT_ZERO.state) {
        twrist.flipTo(Twrist.twristTargetStates.GRAB);
        wrist.flipTo(Wrist.WristTargetStates.GRAB);
        arm.flipTo(HOVER);
        lift.update();
        wrist.update();
        twrist.update();
        lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
        intake.stopIntake();
        LOGGER.log("buh");
      }
//      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
//      Lift.LiftMovingStates.AT_ZERO.clearTargets();
    }
  }

  public void intakeAuto(int height) {
    if (queuer.queue(false, Intake.IntakeStates.STOPPED.getState())) {
      if (!queuer.isExecuted()) {
        intake.setHeight(height);
        intake.intake();
      }
    }
  }

  public void resetLift() {
    if (queuer.queue(true, lift.getCurrentPosition() < 5)) {
      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
      lift.setPosition(0);
    }
  }

  public void dropWrist() {
    if (queuer.queue(false, Wrist.WristStates.DROP.getState())) {
      if (!queuer.isExecuted()) {
        wrist.flipTo(Wrist.WristTargetStates.DROP);
      }
    }
  }

  public void drop() {
    if (queuer.queue(false, Claw.clawStates.CLOSE.getState())) {
      if (!queuer.isExecuted()) {
        claw.moveTwo(false);
        claw.moveOne(false);
        Claw.clawStates.CLOSE.setStateTrue();
      }
    }
  }

  /**
   * Calls other lift auto Logs that function is called
   *
   * @param p_liftPosition target position
   */
  public void liftAuto(Lift.LiftPositionStates p_liftPosition) {
    if (queuer.queue(true, lift.atTargetPosition())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("lifting to: " + p_liftPosition.name());
        liftAuto(p_liftPosition.getPosition());
      }
    }
  }

  /**
   * Auto lifts lift to this position, lift.update() will handle the rest Logs that this function
   * called to general surface
   *
   * @param p_position what position to go to
   */
  public void liftAuto(double p_position) {
    if (queuer.queue(true, lift.atTargetPosition())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("lifting to: " + p_position);
        liftAuto(p_position);
      }
    }
  }

  public boolean checkAlliance() {
    if (currentPose.getX() > 27) {
      if (ultras.checkAlliance()) {
        roadrun.breakFollowing();
        roadrun.setMotorPowers(0, 0, 0, 0);
    }
      return ultras.checkAlliance();
    }
    return false;
  }


  public boolean checkMovingCloser() {
    return ultras.movingCloser();
  }

  /**
   * follows inputted trajectory Logs that this function is called as well as initial and target
   * pose to general surface
   *
   * @param p_traj inputted trajectory
   */
  public void followTrajSeq(TrajectorySequence p_traj) {
    if (queuer.queue(false, !roadrun.isBusy())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("going from: " + p_traj.start() + " to: " + p_traj.end());
        roadrun.followTrajectorySequenceAsync(p_traj);
      }
    }
  }

  public void followPPPath(Path p_path, boolean linear) {
    boolean equals = false;
    if (path.size() > 0) {
      equals = path.get(0).getPose().equals(p_path.get(0).getPose());
    }
    if (queuer.queue(
        false,
        equals
            && (((path.isFinished() || path.timedOut()))
                || currentPose
                        .vec()
                        .distTo(
                            new Vector2d(
                                path.get(path.size() - 1).getPose().getX(),
                                path.get(path.size() - 1).getPose().getY()))
                    < 1.5))) {
      if (!queuer.isExecuted()) {
        if (!p_path.equals(path)) path = p_path;
        if (linear) {
          path.setPathType(PathType.WAYPOINT_ORDERING_CONTROLLED);
        } else {
          path.setPathType(PathType.HEADING_CONTROLLED);
        }
        path.init();
      }
      double[] speeds =
          path.loop(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
      packet.put("xSpeed", speeds[0]);
      packet.put("ySpeed", speeds[1]);
      packet.put("aSpeed", speeds[2]);
      packet.put("timedOUt", path.timedOut());
      packet.put("isFinished", path.isFinished());
      packet.put("equals", equals);
      roadrun.driveRobotCentric(speeds[0], speeds[1], speeds[2]);
}
    if(equals && path.isFinished()){
      roadrun.drive.stop();
    }
  }

  public void followTrajSeq(TrajectorySequence p_traj, boolean isOptional) {
    if (queuer.isFirstLoop()) {
      queuer.queue(false, false, true);
    } else {
      if (queuer.queue(false, queuer.isStarted() && (!roadrun.isBusy()), isOptional)) {
        if (!roadrun.isBusy()) {
          roadrun.followTrajectorySequenceAsync(p_traj);
          LOGGER.log("going from: " + p_traj.start() + " to: " + p_traj.end());
        }
      }
    }
  }

  /** What is run each loop in teleOp Logs that this function is being called to general surface */
  public void teleOp() {
    boolean isA = gampad.readGamepad(op.gamepad2.a, "gamepad1_a", "resetOuttake");
    boolean rightBumper =
        gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper", "startIntake");
    boolean leftBumper =
        gampad.readGamepad(op.gamepad1.a, "gamepad1_left_bumper", "reverseIntake");
    boolean isB = gampad.readGamepad(op.gamepad1.b, "gamepad1_b", "shoot");
    boolean isB2 = gampad.readGamepad(op.gamepad2.b, "gamepad2_b", "lockPower");

    boolean isX = gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "toggleFieldCentricSlow");
    boolean isY = gampad.readGamepad(op.gamepad1.y, "gamepad1_y", "deposit");
    boolean up = gampad.readGamepad(op.gamepad2.dpad_up, "gamepad2_dpad_up", "lift Up");
    boolean down = gampad.readGamepad(op.gamepad2.dpad_down, "gamepad2_dpad_down", "lift down");
    boolean right2 = gampad.readGamepad(op.gamepad2.dpad_right, "gamepad2_dpad_right", "tilt right");
    boolean left2 = gampad.readGamepad(op.gamepad2.dpad_left, "gamepad2_dpad_left", "tilt left");
    boolean right =
        gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "toggleButterfly");
    boolean left = gampad.readGamepad(op.gamepad1.left_bumper, "gamepad1_dpad_left", "toggleClamp");

    boolean isX2 = gampad.readGamepad(op.gamepad2.x, "gamepad2_x", "toggleFieldCentricSlow");
    boolean isY2 = gampad.readGamepad(op.gamepad2.y, "gamepad2_y", "toggleFieldCentricSlow");

    float manualUp = op.gamepad1.right_trigger;
    float manualDown = op.gamepad1.left_trigger;
    float hangUp = op.gamepad2.right_trigger;
    float hangDown = op.gamepad2.left_trigger;
    boolean intUp =
        gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "intake iterate up");
    boolean intDown =
            gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "intake iterate down");
    boolean isRightBumper2 =
        gampad.readGamepad(op.gamepad2.right_bumper, "gamepad2_right_bumper", "startIntake");
    ;
    if (isA) {
      twrist.flipTo(Twrist.twristTargetStates.GRAB);
      wrist.flipTo(Wrist.WristTargetStates.GRAB);
      arm.flipTo(HOVER, -.07);
      lift.update();
      wrist.update();
      twrist.update();
      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
      intake.stopIntake();
    }
    if (isB) {
      if (launcher.getLoaded()) launcher.shoot();
      else launcher.load();
    }
    if (isX2) {
      lift.resetPosition();
    }
    if (rightBumper) {
      if (Intake.IntakeStates.STOPPED.getState()) {
        //        magazine.clampTo(Magazine.MagazineTargetStates.CLOSE);
        intake.intake();
      } else {
        //        magazine.clampTo(Magazine.MagazineTargetStates.OPEN);
        intake.stopIntake();
      }
    }
    if (left) {
      if (HOVER.getState() && !Arm.ArmTargetStates.GRAB.getState()) {
        arm.flipTo(GRAB);
        wrist.flipTo(Wrist.WristTargetStates.LOCK);
      }
      if (Claw.clawStates.CLOSE.getState()) {
        claw.flipTo(Claw.clawTargetStates.GRAB);
      } else {
        claw.flipTo(Claw.clawTargetStates.CLOSE);
      }
    }
    if (leftBumper) {
      intake.reverseIntake();
    }
    if (left2) {
      twrist.flipTo(Twrist.twristTargetStates.LEFT_TILT);
    }
    if (right2) {
      twrist.flipTo(Twrist.twristTargetStates.RIGHT_TILT);
    }
    if (up) {
      lift.iterateUp();
      intake.stopIntake();
      arm.flipTo(DROP);
      wrist.flipTo(Wrist.WristTargetStates.DROP);
      twrist.flipTo(Twrist.twristTargetStates.DROP);
    }
    if (intUp) {
      intake.toggleIntakeHeight();
    }
    if(intDown)
      intake.toggleIntakeHeightDown();
    if (down) {
      arm.flipTo(DROP);
      wrist.flipTo(Wrist.WristTargetStates.DROP);
      twrist.flipTo(Twrist.twristTargetStates.DROP);
      lift.iterateDown();
      intake.stopIntake();
    }
    if (abs(op.gamepad2.left_stick_y) > 0.05) {
      lift.manualExtend(-op.gamepad2.left_stick_y);
    }
    //    if (abs(hangUp - hangDown) > 0.05) {
    hanger.setPower(hangUp - hangDown);
    //    }
    if (isY) {
      wrist.flipTo(Wrist.WristTargetStates.DROP);
    }
    if (isB2) {
      hanger.setPermaPower(hangUp - hangDown);
    }
    if (right) {
      roadrun.toggleButtered();
    }
    if (isX) {
      intake.setHeight(1);
    }
    if (isX2) {
      preloader.deposit();
    }
    if (isY2) {
      preloader.load();
    }

    roadrun.setWeightedDrivePower(
        new Pose2d(
            -op.gamepad1.left_stick_y, -op.gamepad1.left_stick_x, -op.gamepad1.right_stick_x));
    update();
  }

  public void done() {
    queuer.done();
  }

  /**
   * updates the states of all the following Logs that this function is being called to surface
   * general log All else is logged in each respective function
   */
  public void update() {
    LOGGER.setLogLevel(RFLogger.Severity.FINER);
    LOGGER.log("updating each component");
    super.update();
    arm.update();
    if (!isTeleop) {
      cv.update();
      if (op.isStarted()) queuer.setFirstLoop(false);
    }
    intake.update();
    lift.update();
    roadrun.update();
    wrist.update();
    ultras.update();
    twrist.update();
    claw.update();
    magazine.updateColor();
  }

  public void stop() {
    LOGGER.log("the program has stopped normally");
    cv.stop();
  }
}
