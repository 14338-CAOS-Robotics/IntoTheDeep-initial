package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class BaseServo {
    public Servo myServo;

    public BaseServo(){
        myServo = hardwareMap.get(Servo.class, "servoNameInConfig");
    }

    public void moveToPos1(){
        //Tune this value
        myServo.setPosition(0);
    }

    public void moveToPos2(){
        //Tune this value
        myServo.setPosition(1);
    }

}
