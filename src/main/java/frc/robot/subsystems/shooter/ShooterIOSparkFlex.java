// Copyright 2021-2024 FRC 6328
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

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;

public class ShooterIOSparkFlex implements ShooterIO {
  private final SparkFlex leftMotor = new SparkFlex(12, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final SparkClosedLoopController leftPID = leftMotor.getClosedLoopController();
  private final SparkMaxConfig leftConfig = new SparkMaxConfig();
  private final SparkFlex rightMotor = new SparkFlex(13, MotorType.kBrushless);
  private final RelativeEncoder rightEncoder = leftMotor.getEncoder();
  private final SparkClosedLoopController rightPID = leftMotor.getClosedLoopController();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  public ShooterIOSparkFlex() {
    leftConfig
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .voltageCompensation(12.0)
        .smartCurrentLimit(75, 30)
        .secondaryCurrentLimit(85.0);
    SparkUtil.tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    rightConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .voltageCompensation(12.0)
        .smartCurrentLimit(75, 30)
        .secondaryCurrentLimit(85.0);
    SparkUtil.tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionsRadians =
        new double[] {
          Units.rotationsToRadians(leftEncoder.getPosition()) / Shooter.Constants.GEAR_RATIO,
          Units.rotationsToRadians(rightEncoder.getPosition()) / Shooter.Constants.GEAR_RATIO
        };
    inputs.velocitiesRadPerSec =
        new double[] {
          Units.rotationsPerMinuteToRadiansPerSecond(
              leftEncoder.getVelocity() / Shooter.Constants.GEAR_RATIO),
          Units.rotationsPerMinuteToRadiansPerSecond(
              rightEncoder.getVelocity() / Shooter.Constants.GEAR_RATIO)
        };
    inputs.appliedVolts =
        new double[] {
          leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(),
          rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()
        };
    inputs.currentAmps = new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leftPID.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
            * Shooter.Constants.GEAR_RATIO,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
    rightPID.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
            * Shooter.Constants.GEAR_RATIO,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leftConfig.closedLoop.pid(kP, kI, kD);
    SparkUtil.tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    rightConfig.closedLoop.pid(kP, kI, kD);
    SparkUtil.tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
