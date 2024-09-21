package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.IntakePositions.*;
import static org.firstinspires.ftc.teamcode.Hardware.Intake.PIDCoefficients.*;
import static org.firstinspires.ftc.teamcode.Testing.ServoCalibration.*;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.*;
import static org.firstinspires.ftc.teamcode.Control.MathUtils.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Distance_Sensor;


import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Distance_Sensor;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;

@Config
public class Intake {

    public boolean readYForTransfer = false;
    @Config
    public static class PIDCoefficients {
        public static double above45P = 0.0;
        public static double above45I = 0.0;
        public static double above45D = 0.0;
        public static double below45P = 0.0;
        public static double below45I = 0.0;
        public static double below45D = 0.0;

    }
    public enum ArmStates {
        TRANSFERING,
        ZERO
    }

    public static double rClawOpen = 0;
    public static double rClawClosed = 0.335;
    public static double lClawOpen = 0;
    public static double lClawClosed = 0.225;


    public static double leftDistMin = 15;
    public static double rightDistMin = 15;

    Servo lClaw, rClaw, wrist;
    Distance_Sensor leftIntakeDist, rightIntakeDist;
    boolean rClawisOpen = false;
    boolean lClawisOpen = false;

    public static double desiredAngle = 0;
    public DcMotorEx armMotor;

    InterpLUT PCoefficients;
    InterpLUT ICoefficients;
    InterpLUT DCoefficients;
    public Intake(){
        CAOSHardware caosHardware = new CAOSHardware(hardwareMap);

        lClaw = hardwareMap.servo.get("leftClaw");
        rClaw = hardwareMap.servo.get("rightClaw");
        leftIntakeDist = new Distance_Sensor("leftDist");
        rightIntakeDist  = new Distance_Sensor("rightDist");
        lClaw.setDirection(Servo.Direction.REVERSE);
        rClaw.setDirection(Servo.Direction.FORWARD);
        wrist = hardwareMap.servo.get("intakeWrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(intakeWristOpen);
        armMotor = (DcMotorEx) caosHardware.intakeArm;
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armFeedforward = new ArmFeedforward(kS, kCos, kV);
    }
    public void update() {
        //TODO: was causing lag in teleop. I assume this is due to the broken wire cord thingy(plug in new I2C cord)

        leftIntakeDist.distanceUpdate();
        rightIntakeDist.distanceUpdate();
        double ld = leftIntakeDist.getMM();
        double rd = rightIntakeDist.getMM();
        if(!transfered && ld < leftDistMin && lClawisOpen) {
            lClaw.setPosition(lClawClosed);
            lClawisOpen = false;
        }
        if(!transfered && rd < rightDistMin && rClawisOpen) {
            rClaw.setPosition(rClawClosed);
            rClawisOpen = false;
        }

        if(ld < leftDistMin && rd < rightDistMin && !lClawisOpen && !rClawisOpen) {
            readYForTransfer = true;
        } else {
            readYForTransfer = false;
        }
        //hacky fix to gravity
        if(transfered) {
            armUpdate();
        } else if(!transfered && getCorrectedTicks() <zeroOffset + zero) {
            armMotor.setPower(0);
        }

    }
    public void cycleClaw() {
        if(!rClawisOpen ^ !lClawisOpen) {closeClaw();}
        else if(lClawisOpen && rClawisOpen){closeClaw();}
        else if(!lClawisOpen && !rClawisOpen) {openClaw();}
    }

    public void openClaw() {
        rClaw.setPosition(rClawOpen);
        lClaw.setPosition(lClawOpen);
        rClawisOpen = true;
        lClawisOpen = true;
    }
    public void closeClaw() {
        rClaw.setPosition(rClawClosed);
        lClaw.setPosition(lClawClosed);
        rClawisOpen = false;
        lClawisOpen = false;
    }
    boolean transfered = false;
    public void setArmState(ArmStates states) {
        switch (states) {
            case TRANSFERING:
                goToArmTransfer();
                break;
            case ZERO:
                goToArmZero();
                break;
        }

    }
    public boolean armIsAtState(ArmStates state, double threshold) {
        switch (state) {
            case TRANSFERING:
                return Math.abs(getCorrectedTicks() - transfering) <= threshold;
            case ZERO:
                return Math.abs(getCorrectedTicks() - zero) <= threshold;
        }
        return false;
    }
    public void toggleRotation() {
        if (!transfered) {
            goToArmTransfer();
        } else {
            goToArmZero();
        }
    }

    public void goToArmTransfer() {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist.setPosition(intakeWristTransfer);
        transfered = true;
    }


    public void goToArmZero() {
        armMotor.setTargetPosition(zero + zeroOffset);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setPower(.1);
        wrist.setPosition(intakeWristOpen);
        transfered = false;
    }

    public void jointToTransfer() {
        wrist.setPosition(intakeWristTransfer);
        wristZero = false;
    }
    public void jointToGrab() {
        wrist.setPosition(intakeWristOpen);
        wristZero = true;
    }


    //balls
    ArmFeedforward armFeedforward;



    public void armUpdate() {
        double armAngle = armZeroAngle + 2*Math.PI*(getCorrectedTicks()/(537.7*angleMod));
        armMotor.setPower(armFeedforward.calculate(armAngle, 0)+constantIg);
    }

    //test 2 - shoot me in my balls and slap me silly
    PIDController above45;
    PIDController below45;
    public void test() {
        above45 = new PIDController(above45P, above45I, above45D, 1);
        below45 = new PIDController(below45P, below45I, below45D, 1);
    }

    public void testUpdate(){
        double armAngle = catchJump(armZeroAngle + 2*Math.PI*(getCorrectedTicks()/(537.7*angleMod)));
        if(armAngle > Math.PI/4) {
            armMotor.setPower(above45.update(desiredAngle - armAngle));
        } else {
            armMotor.setPower(below45.update(desiredAngle - armAngle));
        }
    }

    //if testupdate works PLEASE GOD PLEASE

//    public void interSetup() {
//        PCoefficients = new InterpLUT();
//        ICoefficients = new InterpLUT();
//        DCoefficients = new InterpLUT();
//        PCoefficients.add(0, below45P);
//        PCoefficients.add(Math.PI/4, above45P);
//        PCoefficients.createLUT();
//
//        ICoefficients.add(0, below45I);
//        ICoefficients.add(Math.PI/4, above45I);
//        ICoefficients.createLUT();
//
//        DCoefficients.add(0, below45D);
//        DCoefficients.add(Math.PI/4, above45D);
//        DCoefficients.createLUT();
//    }
    public void betterUpdate() {
        double armAngle = catchJump(armZeroAngle + 2*Math.PI*(getCorrectedTicks()/(537.7*angleMod)));
        PIDController armPID = new PIDController(interpolateRanges(armAngle, 0, Math.PI/4, below45P, above45P),
                interpolateRanges(armAngle, 0, Math.PI/4, below45I, above45I),
                interpolateRanges(armAngle, 0, Math.PI/4, below45D, above45D), 1);
        armMotor.setPower(armPID.update(desiredAngle - armAngle));
    }

    //if the above fails add feedforward
    public void interSetupFeedforward() {
        armFeedforward = new ArmFeedforward(kS, kCos, kV);
    }
    public void betterUpdateFeedforward() {
        double armAngle = catchJump(armZeroAngle + 2*Math.PI*(getCorrectedTicks()/(537.7*angleMod)));
        PIDController armPID = new PIDController(interpolateRanges(armAngle, 0, Math.PI/4, below45P, above45P),
                interpolateRanges(armAngle, 0, Math.PI/4, below45I, above45I),
                interpolateRanges(armAngle, 0, Math.PI/4, below45D, above45D), 1);
        armMotor.setPower(armFeedforward.calculate(armAngle, armPID.update(desiredAngle - armAngle)));
    }

    public void updateCoefficients() {
        if(below45.getP() != below45P || below45.getI() != below45I || below45.getD() != below45D ||
                above45.getP() != above45P || above45.getI() != above45I || above45.getD() != above45D) {
            below45.setConstants(below45P, below45I, below45D);
            above45.setConstants(above45P, above45I, above45D);
        }
    }
    //
    public double catchJump(double diff) {
        while(diff > Math.PI){
            diff -= Math.PI * 2;
        }
        while(diff < -Math.PI){
            diff += Math.PI * 2;
        }
        return diff;
    }

    public void jointZero() {
        wrist.setPosition(intakeWristOpen);
        wristZero = true;
    }
    
    public int getCorrectedTicks() {
        return armMotor.getCurrentPosition();

    }


}
