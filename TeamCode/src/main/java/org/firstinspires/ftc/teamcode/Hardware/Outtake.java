package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;

/**
 * This class just handles releasing the Pez.
 * To move the outtake arm, use Transfer class
 */
@Config
public class Outtake {
    public static double CLAW_OPEN_POSITION = 0.4;
    public static double CLAW_CLOSED_POSITION = 0.64;
    Servo pitch, roll, effect;
    DcMotorEx lift;
    public Outtake(CAOSHardware hardware) {
        pitch = hardwareMap.servo.get("op");
        roll = hardwareMap.servo.get("or");
        effect = hardwareMap.servo.get("oe");

        lift = (DcMotorEx) hardware.lift;
    }

    /**
     * Lift/lower the outtake apparatus
     * @param power Positive for up, negative for down (-1 to 1)
     */
    public void doTheLifts(double power) {

    }

    /**
     * Close the claw to grab a Pez
     */
    public void closeClaw() {
        effect.setPosition(CLAW_CLOSED_POSITION);
    }

    /**
     * Drop a Pez
     */
    public void openClaw() {
        effect.setPosition(CLAW_OPEN_POSITION);
    }

    /**
     * Move the outtake like a hinge
     * @param pitchServoValue Servo something idk
     */
    public void setPitch(double pitchServoValue) {

    }

    /**
     *
     * Move the outtake like a wrist
     * @param rollServoValue We'll figure that out later (ticks?)
     */
    public void setRoll(double rollServoValue) {

    }



}