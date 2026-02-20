package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX pivot = new TalonFX(Constants.IntakePivotConstants.PIVOT_ID);

  private final boolean hasCancoder = Constants.IntakePivotConstants.PIVOT_CANCODER_ID >= 0;
  private final CANcoder cancoder = hasCancoder ? new CANcoder(Constants.IntakePivotConstants.PIVOT_CANCODER_ID) : null;

  private final PositionVoltage posCtrl = new PositionVoltage(0).withEnableFOC(true);

  private final StatusSignal<Angle> pos = pivot.getPosition();     
  private final StatusSignal<Voltage> volts = pivot.getMotorVoltage();
  private final StatusSignal<Current> amps = pivot.getSupplyCurrent();
  private final StatusSignal<Temperature> temp = pivot.getDeviceTemp();

  private final StatusSignal<Angle> cancoderAbs = hasCancoder ? cancoder.getAbsolutePosition() : null;

  private double lastTargetRot = 0.0;

  public IntakePivotIOTalonFX() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = Constants.IntakePivotConstants.PIVOT_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    
    cfg.Feedback.SensorToMechanismRatio = Constants.IntakePivotConstants.PIVOT_GEAR_RATIO;

    cfg.Slot0.kP = Constants.IntakePivotConstants.kP;
    cfg.Slot0.kI = Constants.IntakePivotConstants.kI;
    cfg.Slot0.kD = Constants.IntakePivotConstants.kD;
    cfg.Slot0.kV = Constants.IntakePivotConstants.kV;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivotConstants.PIVOT_SUPPLY_LIMIT_A;

    pivot.getConfigurator().apply(cfg);

    if (hasCancoder) {
      CANcoderConfiguration cc = new CANcoderConfiguration();
      cancoder.getConfigurator().apply(cc);
    }

    if (hasCancoder) {
      BaseStatusSignal.setUpdateFrequencyForAll(50, pos, volts, amps, temp, cancoderAbs);
    } else {
      BaseStatusSignal.setUpdateFrequencyForAll(50, pos, volts, amps, temp);
    }
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (hasCancoder) {
      BaseStatusSignal.refreshAll(pos, volts, amps, temp, cancoderAbs);
      inputs.cancoderAbsRot = cancoderAbs.getValueAsDouble() + Constants.IntakePivotConstants.CANCODER_OFFSET_ROT;
    } else {
      BaseStatusSignal.refreshAll(pos, volts, amps, temp);
      inputs.cancoderAbsRot = -1.0;
    }

    inputs.hasCancoder = hasCancoder;
    inputs.positionRot = pos.getValueAsDouble();
    inputs.targetRot = lastTargetRot;
    inputs.appliedVolts = volts.getValueAsDouble();
    inputs.currentAmps = amps.getValueAsDouble();
    inputs.tempC = temp.getValueAsDouble();
  }

  @Override
  public void setPositionRot(double rot) {
    lastTargetRot = rot;
    pivot.setControl(posCtrl.withPosition(rot));
  }

  @Override
  public void stop() {
    pivot.stopMotor();
  }

  @Override
  public void syncToAbsolute() {
    if (!hasCancoder) return;

    // absolute position is in rotations (0..1) from CANcoder
    double absRot = cancoderAbs.getValueAsDouble() + Constants.IntakePivotConstants.CANCODER_OFFSET_ROT;

    // Set TalonFX integrated sensor to match absolute
    pivot.setPosition(absRot);
  }
}