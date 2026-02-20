package frc.robot.subsystems.intakepivot;

import edu.wpi.first.epilogue.Logged;

public interface IntakePivotIO {

  @Logged
  class IntakePivotIOInputs {
    public double positionRot;     
    public double targetRot;       
    public double appliedVolts;
    public double currentAmps;
    public double tempC;
    public boolean hasCancoder;
    public double cancoderAbsRot; 
  }

  void updateInputs(IntakePivotIOInputs inputs);

  void setPositionRot(double rot);

  void stop();


  default void syncToAbsolute() {}

  default void simulationPeriodic() {}
}