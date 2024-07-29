package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class otbIntakeConstants {

    /* Motor Inverts */

    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive; //NOT REAL
    public static final InvertedValue intakeInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue indexerInvert = InvertedValue.CounterClockwise_Positive;

    /* Current Limits */

    public static final double pivotCurrentLimit = 50; //NOT REAL
    public static final double intakeCurrentLimit = 50; //NOT REAL
    public static final double indexerCurrentLimit = 50; //NOT REAL

    public static final double gearRatio = 76.1904761905;

}
