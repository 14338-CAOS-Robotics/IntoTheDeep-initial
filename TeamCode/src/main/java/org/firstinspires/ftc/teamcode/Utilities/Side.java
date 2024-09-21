package org.firstinspires.ftc.teamcode.Utilities;

public class Side {
    public static boolean red = false;

    public static void setBlue(){
        red = false;
    }

    public static void setRed(){
        red = true;
    }

    public static String getSide(){
        if(red){
            return "Red";
        }else{
            return "Blue";
        }
    }
}
