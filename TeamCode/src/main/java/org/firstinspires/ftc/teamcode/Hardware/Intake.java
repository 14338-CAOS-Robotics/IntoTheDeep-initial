package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GameLogic.PezColor;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color_Sensor;

/**
 * Wrapper class for the active intake, extension, and color sensor.
 * (use Transfer class for pitching the intake)
 */
public class Intake {
    CAOSHardware hardware;
    private Color_Sensor colorSensor;
    private DcMotorEx activeIntake, horizontalExtension;
    public Intake(CAOSHardware hardware) {
        activeIntake = (DcMotorEx) hardware.activeIntake;
        horizontalExtension = (DcMotorEx) hardware.intakeExtend;
        colorSensor = new Color_Sensor();
        colorSensor.init("ic");
    }

    /**
     * Push the active intake horizontally to grab Pez
     * @param power Positive to move forward, negative to retract (-1 to 1)
     */
    public void extend(double power) {}

    /**
     * Turn on the spinners in the active intake
     */
    public void activateIntake() {}

    /**
     * Turn off the spinners in the active intake
     */
    public void deactivateIntake() {}

    /**
     * Get color of the Pez in the intake
     * @return PezColor enum (RED, BLUE, YELLOW, NO_PEZ) using color sensor
     */
    public PezColor getColor() {
        return PezColor.NO_PEZ;
    }

    /**
     * Check whether or not the intake has a Pez, as per the color sensor
     * @return true if there is a Pez in the intake, false otherwise
     */
    public boolean hasPez() {
        return false;
    }


}