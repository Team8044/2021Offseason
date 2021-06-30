package frc.lib.math;


public class Boundaries {

    public static double to180Boundaries(double degrees){
        degrees %= 360;
        if(degrees > 180){
            degrees -= 360;
        }else if(degrees < -180){
            degrees += 360;
        }
        return degrees;
    }

    public static double to360Boundaries(double degrees){
        degrees %= 360;
        degrees = (degrees < 0) ? (degrees + 360) : degrees; 
        return degrees;
    }
}