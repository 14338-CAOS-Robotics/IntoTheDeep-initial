package org.firstinspires.ftc.teamcode.Hardware;


import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.armExtend;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.armZero;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.lOCClose;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.lOCOpen;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.rOCClose;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.rOCOpen;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.wristExtend;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.wristZero;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    public boolean v4bZero = false;
    boolean wristZero = true;
    boolean clawsZero = false;
    Servo l4b, r4b, armJoint, rightOuttakeClaw, leftOuttakeClaw;
    public Outtake() {
        l4b = hardwareMap.servo.get("l4b");
        r4b = hardwareMap.servo.get("r4b");
        armJoint = hardwareMap.servo.get("aj");
        rightOuttakeClaw = hardwareMap.servo.get("roc");
        leftOuttakeClaw = hardwareMap.servo.get("loc");

        r4b.setDirection(Servo.Direction.REVERSE);
        rightOuttakeClaw.setDirection(Servo.Direction.REVERSE);

    }

    public void cycleV4b() {
        if (!v4bZero) {
            l4b.setPosition(armZero);
            r4b.setPosition(armZero);
            v4bZero = true;
        } else {
            l4b.setPosition(armExtend);
            r4b.setPosition(armExtend);
            v4bZero = false;
        }
    }

    public void cycleWrist() {
        if(wristZero) {
            armJoint.setPosition(wristExtend);
            wristZero = false;
        } else {
            armJoint.setPosition(Robot.OuttakeConstants.wristZero);
            wristZero = true;
        }
    }

    public void cycleOuttakeClaw() {
        if(clawsZero) {
            leftOuttakeClaw.setPosition(lOCOpen);
            rightOuttakeClaw.setPosition(rOCOpen);
            clawsZero = false;
        } else {
            leftOuttakeClaw.setPosition(lOCClose);
            rightOuttakeClaw.setPosition(rOCClose);
            clawsZero = true;
        }
    }
    public void transferPosition() {
        l4b.setPosition(armZero);
        r4b.setPosition(armZero);
        armJoint.setPosition(Robot.OuttakeConstants.wristZero);
        closeClaw();
    }
    public void depositPosition() {
        l4b.setPosition(armExtend);
        r4b.setPosition(armExtend);
        armJoint.setPosition(wristExtend);
        openClaw();
    }
    public void transferNoClawPos() {
        l4b.setPosition(armZero);
        r4b.setPosition(armZero);
        armJoint.setPosition(Robot.OuttakeConstants.wristZero);
    }
    public void closeClaw() {
        leftOuttakeClaw.setPosition(lOCClose);
        rightOuttakeClaw.setPosition(rOCClose);
        clawsZero = true;
    }

    public void openClaw() {
        leftOuttakeClaw.setPosition(lOCOpen);
        rightOuttakeClaw.setPosition(rOCOpen);
        clawsZero = false;
    }
}