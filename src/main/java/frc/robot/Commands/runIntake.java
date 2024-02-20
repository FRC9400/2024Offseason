package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.Intake.Intake;

public class runIntake extends Command{
    private final Intake Intake;
    private boolean intakeOutake;

    public runIntake(Intake Intake, boolean intakeOutake){
        this.Intake = Intake;
        this.intakeOutake = intakeOutake;
        addRequirements(Intake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double output = 3;
        if(intakeOutake){
  
            output *= -1;
        }
  
        Intake.spinIntake(output);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        Intake.spinIntake(0);
    }


    
}
