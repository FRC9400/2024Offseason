
package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
      }
    
    public void spinIntake(double volts){
        intakeIO.setVoltage(volts);
    }


    @Override
    public void periodic(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}