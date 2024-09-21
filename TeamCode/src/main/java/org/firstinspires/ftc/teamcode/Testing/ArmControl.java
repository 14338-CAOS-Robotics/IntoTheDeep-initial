package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Utilities.PID;

public class ArmControl {

    DcMotor arm;

    PID pid;

    Servo wrist;

    int armhomedthreshold = 8;
    private int targetposition = 0;

    private int max = 900;
    @Config
    public static class ArmDash {
        public static double proportional = 0.005, integral = 0.0001, derivative = 0.001, scoringWristPosition = 0.91, homeWristPosition = 0.9;
    }
    public ArmControl(CAOSHardware hardware){
        arm = hardwareMap.dcMotor.get("armMotor");
        DcMotor.RunMode mode = arm.getMode();
        pid = new PID(0, 0, 0);
        wrist = hardwareMap.servo.get("intakeWrist");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(mode);
    }
    double returnError = 0;
    public double getError() {
        return returnError;
    }
    public void setposition(int target){
        targetposition = target;
    }
    public void update(){
        pid.setWeights(ArmDash.proportional, ArmDash.integral, ArmDash.derivative);
        int currentPosition = arm.getCurrentPosition();
        targetposition = Range.clip(targetposition, 0, max);
        double error =   targetposition - currentPosition;

        double power = pid.update(error, false);
        if (SlideControl.slidehome && currentPosition <= armhomedthreshold && error <= 0) {
            power = 0;
        }
        returnError = error;
        telemetry.addData("motor ticks", currentPosition);
        telemetry.addData("error", error);
        telemetry.addData("power", power);
        telemetry.update();
        arm.setPower(power);

    }
    public void setHeight(SlideControl.PoleHeight height) {
        switch (height) {
            case Short:
                setposition(850);
                wrist.setPosition(ArmDash.scoringWristPosition);
                break;
            case Med:
            case Tall:
                setposition(750);
                wrist.setPosition(ArmDash.scoringWristPosition);
                break;
            case Home:
                setposition(0);
                wrist.setPosition(ArmDash.homeWristPosition);
                break;
        }
    }

}
