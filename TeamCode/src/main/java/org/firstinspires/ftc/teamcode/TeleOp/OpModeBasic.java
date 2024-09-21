package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.maxSlideTicks;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AcrossOpModes.VariablisticVars;
import org.firstinspires.ftc.teamcode.Control.Localization.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.GamepadControls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Utilities.Side;

@Config
@TeleOp (name = "Red TeleOp")
public class OpModeBasic extends LinearOpMode {
    double basisHeading;

    Controller gp1;
    Controller gp2;
    boolean wasDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        TouchSensor touchsensorLeft = hardwareMap.get(TouchSensor.class, "lts");
        TouchSensor touchsensorright = hardwareMap.get(TouchSensor.class, "rts");
        setOpMode(this);
        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);
        this.setBasisHeading();
        Robot robot = new Robot();
        robot.toTransferPos();
        robot.initClaws();
        robot.intake.jointZero();
        robot.outtake.closeClaw();

        robot.setController1(gp1);
        robot.setController2(gp2);
        waitForStart();

        robot.colesHeadingFixer.setBasisHeading(basisHeading);
        while (opModeIsActive() && !isStopRequested()) {

            robot.update();
            Controller.update();


            if (gp1.get(ButtonControls.Input.LB1, ButtonControls.ButtonState.TAP)) {
                robot.cycleFieldCentricMode();
            }

            robot.setDrivePower(
                    gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X),
                    gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),
                    gp1.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X));

            if (gp1.get(ButtonControls.Input.RB1, ButtonControls.ButtonState.TAP)) {
                robot.cycleIntakeClaw();
            }
            if(gp1.get(ButtonControls.Input.RB2, ButtonControls.ButtonState.TAP)) {
                robot.cycleIntakeArm();
            }

            if (gp1.get(ButtonControls.Input.DPAD_UP, ButtonControls.ButtonState.TAP))
                robot.setHeading(Math.toRadians(0));
            if (gp1.get(ButtonControls.Input.DPAD_R, ButtonControls.ButtonState.TAP))
                robot.setHeading(Math.toRadians(270));
            if (gp1.get(ButtonControls.Input.DPAD_DN, ButtonControls.ButtonState.TAP))
                robot.setHeading(Math.toRadians(180));
            if (gp1.get(ButtonControls.Input.DPAD_L, ButtonControls.ButtonState.TAP))
                robot.setHeading(90);





            //CONTROLLER 2
            if (gp2.get(ButtonControls.Input.TRIANGLE, ButtonControls.ButtonState.TAP)) {
                robot.toTransferPos();
            }

            if (gp2.get(ButtonControls.Input.SQUARE, ButtonControls.ButtonState.TAP)) {
                robot.cycleOuttakeClaw();
            }

            if(gp2.get(ButtonControls.Input.CIRCLE, ButtonControls.ButtonState.TAP)) {
                robot.depositPosition();
            }
            if(gp2.get(ButtonControls.Input.LB1, ButtonControls.ButtonState.TAP)) {
                robot.toggleLiftSpeed();
            }

//            if (gp1.get(ButtonControls.Input.CIRCLE, ButtonControls.ButtonState.TAP)) {
//                if(armJointRotated) {
//                    armJoint.setPosition(armJointPositionPixel);
//                    armJointRotated = false;
//                    telemetry.addData("Wrist Resting", "True");
//
//                } else {
//                    armJoint.setPosition(armJointPositionScore);
//                    armJointRotated = true;
//                    telemetry.addData("Wrist Resting", "False");
//po
//                }
//                telemetry.update();
//            }
//
//            if(gp1.get(ButtonControls.Input.TRIANGLE, ButtonControls.ButtonState.TAP)) {
//                if(pdUp) {
//                    pixelDrop.setPosition(pixelDropperRelease);
//                    telemetry.addData("pixel dropper position", "released");
//                    pdUp = false;
//                } else {
//                    pixelDrop.setPosition(pixelDropperContain);
//                    telemetry.addData("pixel dropper position", "contained");
//                    pdUp = true;
//                }

            if((touchsensorLeft.getValue() > 0 || touchsensorright.getValue() > 0) ) {
                robot.resetSlideTicks();
                robot.runSlides(Range.clip(gp2.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),-0.9,0.0));

                if(!wasDown) {
                    robot.intake.jointZero();
                    robot.intake.openClaw();
                    robot.outtake.closeClaw();
                    robot.outtake.transferNoClawPos();
                    wasDown = true;
                }
                wasDown = true;
            } else if(robot.getSlideTicks() < -maxSlideTicks) {
                robot.runSlides(Range.clip(gp2.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),0,.9));
            } else {
                robot.runSlides(gp2.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y)*0.9);
            }
            if (robot.getSlideTicks() > 15) wasDown = false;
        }

        if(gp1.get(ButtonControls.Input.TOUCHPAD, ButtonControls.ButtonState.TAP) || gp2.get(ButtonControls.Input.TOUCHPAD, ButtonControls.ButtonState.TAP)) {
            robot.drivetrain.setPoseEstimate(VariablisticVars.currentPose);
        }
        }

    public void setBasisHeading() {
        this.basisHeading=180;
    }

//    Gamepad.RumbleEffect rumbleEffect;
//    public void rumbleController() {
//        rumbleEffect = new Gamepad.RumbleEffect.Builder()
//                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
//                .addStep(0.0, 0.0, 300)
//                .build();
//    }
    }
