package frc.robot.subsystems.intakepivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

@Logged
public class IntakePivotSubsystem extends SubsystemBase {

  public enum WantedState {
    STOWED,
    DEPLOYED
  }

  private final IntakePivotIO io = new IntakePivotIOTalonFX();
  private final IntakePivotIO.IntakePivotIOInputs inputs = new IntakePivotIO.IntakePivotIOInputs();

  private WantedState wanted = WantedState.STOWED;
  private double targetRot = Constants.IntakePivotConstants.STOW_ROT;

  public IntakePivotSubsystem() {
    // Optional dashboard debug
    SmartDashboard.putString("IntakePivot/Wanted", wanted.name());
  }

  public void setWantedState(WantedState state) {
    wanted = state;
  }

  public WantedState getWantedState() {
    return wanted;
  }

  /** Call once when robot enables (or in RobotContainer after construction). */
  public void syncAbsolute() {
    io.syncToAbsolute();
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean atTarget() {
    return Math.abs(inputs.positionRot - targetRot) <= Constants.IntakePivotConstants.READY_TOL_ROT;
  }

  @NotLogged
  public double getPositionRot() {
    return inputs.positionRot;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Choose target from state
    switch (wanted) {
      case STOWED -> targetRot = Constants.IntakePivotConstants.STOW_ROT;
      case DEPLOYED -> targetRot = Constants.IntakePivotConstants.DEPLOY_ROT;
    }

    // Safety clamp
    targetRot = Math.max(Constants.IntakePivotConstants.MIN_ROT,
               Math.min(Constants.IntakePivotConstants.MAX_ROT, targetRot));

    io.setPositionRot(targetRot);

    SmartDashboard.putString("IntakePivot/Wanted", wanted.name());
    SmartDashboard.putNumber("IntakePivot/TargetRot", targetRot);
    SmartDashboard.putNumber("IntakePivot/PosRot", inputs.positionRot);
    if (inputs.hasCancoder) {
      SmartDashboard.putNumber("IntakePivot/CANcoderAbsRot", inputs.cancoderAbsRot);
    }
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }
}