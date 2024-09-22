package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for taking care of the transfer of Pez from the intake to the outtake
 */
public class Transfer {

    Intake intake;
    Outtake outtake;
    Servo armLeft, armRight, intakePitchLeft, intakePitchRight;


    public Transfer(Intake intake, Outtake outtake) {
        armLeft = hardwareMap.servo.get("oal");
        armRight = hardwareMap.servo.get("oar");
        intakePitchLeft = hardwareMap.servo.get("ipl");
        intakePitchRight = hardwareMap.servo.get("ipr");

        this.intake = intake;
        this.outtake = outtake;
    }

    /**
     * Move the outtake (arm, pitch, and roll) into position for transfer
     */
    public void outtakeToTransfer() {

    }

    /**
     * Move the outtake (arm, pitch, roll) into position to drop into basket
     * You still gotta do the lift yourself, though
     */
    public void outtakeToDrop() {

    }

    /**
     * Bring in extension, pitch intake up
     */
    public void intakeToTransfer() {

    }

    /**
     * Put the intake down to the floor
     */
    public void intakeToActivePos() {

    }

    /**
     * Move a Pez from the intake to the outtake (only run if the lifts are down)
     */
    public void doTheTransfer(){

    }



}
