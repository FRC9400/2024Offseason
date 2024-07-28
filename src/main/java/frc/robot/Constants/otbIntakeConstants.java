package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class otbIntakeConstants {
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue intakeInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue indexerInvert = InvertedValue.CounterClockwise_Positive;

    public static final double pivotCurrentLimit = 50;
    public static final double intakeCurrentLimit = 50;
    public static final double indexerCurrentLimit = 50;

    public static final double gearRatio = 23.625;

}
