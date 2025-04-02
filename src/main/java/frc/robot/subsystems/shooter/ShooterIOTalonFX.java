// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * This shooter implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooter = new TalonFX(motorId);
  private final StatusSignal<AngularVelocity> velocityRotPerSec = shooter.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = shooter.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = shooter.getSupplyCurrent();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooter.getConfigurator().apply(config, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocityRotPerSec, appliedVolts, currentAmps);
    shooter.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityRotPerSec, appliedVolts, currentAmps);

    inputs.velocityRotPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    shooter.setControl(velocityRequest.withVelocity(velocity));
  }
}