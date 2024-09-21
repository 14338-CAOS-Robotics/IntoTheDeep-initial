package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance_Sensor {
    public DistanceSensor distanceSensor;
    public Distance_Sensor(String mapName) {

        distanceSensor = hardwareMap.get(DistanceSensor.class, mapName);
    }
    double millimeters;
    double centimeters;
    double meters;
    double updateIteration = 19;
    double currentDist;
    public boolean isChanging;
    double mostRecentDist;



    public double getMM(){
        return millimeters;
    }

    public double getCM(){
        return centimeters;
    }

    public double getMeter(){
        return meters;
    }

    public void distanceUpdate() {
        if(distanceSensor.getDistance(DistanceUnit.MM) != DistanceSensor.distanceOutOfRange) {
            millimeters = distanceSensor.getDistance(DistanceUnit.MM);
            centimeters = millimeters / 10;
            meters = centimeters / 100;
        }
        updateIteration ++;
        if(updateIteration == 20) {
            updateIteration = 0;
            currentDist = getCM();
            isChanging = Math.abs(mostRecentDist - currentDist) > .1;
            mostRecentDist = currentDist;
        }
    }

}
