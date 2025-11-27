package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.PhoenixUtil.*;

public class IntakeIOComp implements IntakeIO {

  private static final int ID = 99;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60);

  private final TalonFX motor = new TalonFX(ID);

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public IntakeIOComp() {
    tryUntilOk(
        5,
        () ->
            motor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(currentLimits)));

    BaseStatusSignal.setUpdateFrequencyForAll(20, velocity, voltage, current);
    motor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, velocity, voltage, current);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motor.setControl(voltageRequest);
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.angularVelocity = velocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    voltageRequest.withOutput(volts);
  }
}
