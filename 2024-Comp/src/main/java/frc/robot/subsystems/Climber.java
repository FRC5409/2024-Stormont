// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final SparkPIDController m_controller;
  private final RelativeEncoder s_encoder;

  private final ShuffleboardTab sb_climberTab;
  private final GenericEntry kP, kI, kD, position;

  /** Creates a new Climber. */
  public Climber() {
    m_motor = new CANSparkMax(Constants.kClimber.id_motor, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(Constants.kClimber.currentLimit);

    m_controller = m_motor.getPIDController();
    configPID();
    // m_controller.setOutputRange(-1, 1);

    s_encoder = m_motor.getEncoder();

    sb_climberTab = Shuffleboard.getTab("Climber");
    kP = sb_climberTab.add("kP", Constants.kClimber.kP).getEntry();
    kI = sb_climberTab.add("kI", Constants.kClimber.kI).getEntry();
    kD = sb_climberTab.add("kD", Constants.kClimber.kD).getEntry();
    position = sb_climberTab.add("position", 0).getEntry();

    m_motor.burnFlash();
  }

  public void configPID() {
    m_controller.setP(Constants.kClimber.kP);
    m_controller.setI(Constants.kClimber.kI);
    m_controller.setD(Constants.kClimber.kD);
  }

  public void zeroEncoder() {
    s_encoder.setPosition(0);
  }

  public void setpoint(double setpoint) {
    m_controller.setReference(setpoint, ControlType.kPosition);
  }

  public void manualExtend(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    position.setDouble(s_encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
