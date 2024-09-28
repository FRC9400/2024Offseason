package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.Logger;

public class Amp {
    private final AmpIO ampIO;
    private AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();
    private AmpStates state = AmpStates.IDLE;
    private double volts = 0;

    public enum AmpStates{
        IDLE,
        AMP,
        SHOOT
    }

    public Amp(AmpIO amp){
        this.ampIO = amp;
    }

    public void Loop(){
        AmpIO.updateInputs(inputs);
        Logger.processInputs("Amp", inputs);
        Logger.recordOutput("Amp State", this.state);
        switch(state){
            case IDLE:
                ampIO.setAmpRollerVoltage(0);
                break;
            case AMP:
                ampIO.setAmpRollerVoltage(volts);
                break;
            case SHOOT:
                ampIO.setAmpRollerVoltage(volts);
                break;

        }
    }

    public void requestIdle(){
        setState(AmpStates.IDLE);
    }

    public void requestAmp(double volts){
        this.volts = volts;
        setState(AmpStates.AMP);
    }

    public void requestShoot(double volts){
        this.volts = volts;
        setState(AmpStates.SHOOT);
    }

    public void setState(AmpStates next){
        this.state = next;
    }

    public AmpStates getState(){
        return this.state;
    }

    public double getAmpCurrent(){
        return inputs.ampRollerCurrent;
    }
}
