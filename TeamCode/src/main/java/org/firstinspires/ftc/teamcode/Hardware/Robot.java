package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointPositionPixel;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointPositionScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointPositionTransfer;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointWaitTime;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointWaitTime2;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.clawPositionClosed;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.clawPositionOpen;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.pixelDropperReal;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.pixelDropperRelease;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.slideHangClaw;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.slideHangV4b;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.v4bPositionDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.v4bPositionUp;

import static org.firstinspires.ftc.teamcode.Hardware.Robot.DriveMode.*;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.outtakeWaitTime;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.wristConstantTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.linearOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Localization.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.AprilTagHeadingFixer;

/**
 * A class for containing an FTC Mecanum robot
 */
public class Robot {
   @Config
   public static class OuttakeConstants {
   public static double armZero = 0.02;
   public static double armExtend = .49;

   public static double wristZero = .15;
   public static double wristExtend = .05;

   public static double lOCOpen = 0.118;
   public static double lOCClose = 0.065;

   public static double rOCOpen = 0.092;
   public static double rOCClose = 0.031;

   public static double outtakeWaitTime = 0.5;

   public static double wristConstantTime = 0.1;
}


   public CAOSMecanumDrive drivetrain;
   public CAOSHardware caosHardware;


//   Servo leftV4b;
//   Servo rightV4b;

   public DcMotor leftLift;
   public DcMotor rightLift;
   public AprilTagHeadingFixer colesHeadingFixer;
   public DriveMode currentMode = FIELD_CENTRIC_DRIVE;

   private ElapsedTime runtime = new ElapsedTime();

   boolean slowLift = false;
   boolean slideHangToggle = false;
   int leftLiftZero, rightLiftZero = 0;
   public Movement movement;

   Controller gp1;
    Controller gp2;
    public Intake intake;

    public Outtake outtake;

   public enum DriveMode {
      FIELD_CENTRIC_DRIVE,
      LOCK_HEADING_DRIVE,
      ROBOT_CENTRIC_DRIVE
   }

   public Robot() {
      initRobot();
   }
   public Robot(boolean advancedScoring){

      initRobot();
   }

   public double[] Pos;
   public void initRobot() {

      drivetrain = new CAOSMecanumDrive(hardwareMap);

//      localizer = new StandardTrackingWheelLocalizer(hardwareMap);

      colesHeadingFixer = new AprilTagHeadingFixer(hardwareMap);
      CAOSHardware caosHardware = new CAOSHardware(hardwareMap);
      movement = new Movement(0,0,Math.PI/2, caosHardware);
      leftLift = caosHardware.liftLeft;
      rightLift = caosHardware.liftRight;
      leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       DcMotor.RunMode defaultMode = leftLift.getMode();
      leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      leftLift.setMode(defaultMode);
      rightLift.setMode(defaultMode);

      intake = new Intake();
      outtake = new Outtake();

   }

   public void update() {
//      colesHeadingFixer.update();
      movement.odo.localize();
      intake.update();
//      double newHeading = colesHeadingFixer.getNearestTagHeading();
//      if (newHeading > 0) movement.odo.setHeading(newHeading);
      Pos = movement.odo.getLocation();

      //TESTING: Auto transfer
      if(intake.readYForTransfer) {
         toTransferPos();
         intake.readYForTransfer = false;
      }
   }

   public void toDepositPos() {
      outtake.depositPosition();
   }

    public void toTransferPos() {
       outtake.openClaw();
        outtake.transferPosition();

       runtime.reset();
       while (linearOpMode.opModeIsActive() && runtime.seconds() < outtakeWaitTime) {
       }
       //UNCOMMENT IF IT BREAKS
       intake.goToArmTransfer();
   }

   public void initClaws() {
      outtake.closeClaw();
      intake.openClaw();
   }

   public void cycleFieldCentricMode() {
        if(currentMode == FIELD_CENTRIC_DRIVE) {
             currentMode = ROBOT_CENTRIC_DRIVE;
        } else if(currentMode == ROBOT_CENTRIC_DRIVE) {
             currentMode = FIELD_CENTRIC_DRIVE;
        }
   }
   public void setDrivePower(double x, double y, double h) {
      double tagHeading = colesHeadingFixer.getNearestTagHeading();
      if((x == 0 && y == 0 && h == 0)) {
         if(tagHeading > 0) {
            movement.odo.setHeading(tagHeading);
            gp1.rumble(0.7, 200);
            gp1.rumble(0.35, 200);
         }
      } else if(currentMode == LOCK_HEADING_DRIVE) {
         movement.odo.setHeading(tagHeading);
      }

      switch (currentMode) {
         case FIELD_CENTRIC_DRIVE:
            drivetrain.setFieldCentricDrivePower(y, -x, h,
                    Pos[2]);
            break;
         case LOCK_HEADING_DRIVE:
            drivetrain.setFieldCentricDrivePower(y, -x, movement.headingPID.update(movement.catchJump(getOffsetFromSide(), Pos[2])), Pos[2]);
            break;
         case ROBOT_CENTRIC_DRIVE:
            drivetrain.setDrivePower(new Pose2d(y,-x, -h));
            break;
      }
   }

   public void setHeading(double heading) {
      movement.odo.setHeading(heading);
   }

   
   public void runSlides(double slidePower) {
      if(!slowLift) {
         leftLift.setPower(slidePower);
         rightLift.setPower(slidePower);
      } else {
         leftLift.setPower(slidePower/4);
         rightLift.setPower(slidePower/4);
      }
   }



   public void resetSlideTicks() {
      leftLiftZero = leftLift.getCurrentPosition();
      rightLiftZero = rightLift.getCurrentPosition();
   }
   public int getSlideTicks() {
      return ((leftLift.getCurrentPosition()-leftLiftZero) + (rightLift.getCurrentPosition()-leftLiftZero))/2;
   }

   public void toggleLiftSpeed() {
      slowLift = !slowLift;
   }

   public void toggleLockHeading() {
      currentMode = LOCK_HEADING_DRIVE;
   }
   public double getOffsetFromSide() {
      if(Side.red) {
         return Math.PI*3/2;
      } else {
         return Math.PI/2;
      }
   }

   public void setController1(Controller controller) {
        gp1 = controller;
   }
    public void setController2(Controller controller) {
          gp2 = controller;
    }

    public void setDriveMode(DriveMode mode) {
        currentMode = mode;
    }

    public void cycleIntakeClaw() {
        intake.cycleClaw();
    }

    public void cycleOuttakeClaw() {
        outtake.cycleOuttakeClaw();
    }

    public void cycleOuttake() {
        if(outtake.l4b.getPosition() == OuttakeConstants.armZero) {
           outtake.depositPosition();
        } else {
           outtake.transferPosition();
        }
    }


    public void depositPosition() {
       intake.openClaw();
       double bufferTime = runtime.seconds();
        while (linearOpMode.opModeIsActive() && runtime.seconds() - bufferTime < wristConstantTime) {
        }
        intake.jointZero();
        outtake.depositPosition();
    }

    public void transferPosition() {
       outtake.transferNoClawPos();
    }

    public void cycleIntakeArm() {
        if(intake.transfered) {
            intake.goToArmZero();
            intake.openClaw();
        } else {
            intake.goToArmTransfer();
        }
    }

}