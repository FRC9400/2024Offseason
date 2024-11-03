package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {

    public static final PIDController choreoTransController = new PIDController(3, 0, 0);
    public static final PIDController choreoRotController = new PIDController(1.25,0,0);

    public static double subwooferVelM = 70;
    public static double subwooferRatioM = 0.6;
    public static double subwooferDegM = 45; 

    public static double subwooferVelR = 60;
    public static double subwooferRatioR = 0.7;
    public static double subwooferDegR = 45; 

    public static double subwooferVelL = 45.6;
    public static double subwooferRatioL = 1.5;
    public static double subwooferDegL = 45;

    public static double VelM = 70;
    public static double RatioM = 0.6;
    public static double DegM = 27;

    public static double BlueVelR = 70;
    public static double BlueRatioR = 0.6;
    public static double BlueDegR = 27;

    public static double BlueVelL = 70;
    public static double BlueRatioL = 0.6;
    public static double BlueDegL = 31;

    public static double RedVelL = 70;
    public static double RedRatioL = 0.7;
    public static double RedAngleL = 27;

    public static double RedVelR = 60; //60
    public static double RedRatioR = 0.7; //0.7
    public static double RedAngleR = 31;

    
}