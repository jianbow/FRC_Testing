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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, motorReduction),
          DCMotor.getKrakenX60(1));

  // rots/sec
  private double appliedVelocity = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // convert to radians per sec
    sim.setAngularVelocity(appliedVelocity * 2 * Math.PI);
    sim.update(0.02);

    inputs.velocityRotPerSec = appliedVelocity;
    inputs.appliedVolts = sim.getInputVoltage();
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double velocity) {
    appliedVelocity = velocity;
  }
}