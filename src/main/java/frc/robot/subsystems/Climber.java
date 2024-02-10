// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_main;
  private final CANSparkMax m_follower;
  // private final SparkPIDController m_controller;
  private final RelativeEncoder s_encoder;
  // private DigitalInput limitSwitch;
  private DigitalInput irSwitch;

  private final ShuffleboardTab sb_climberTab;
  private final GenericEntry kP, kI, kD, position, irSwitchValue;

  /** Creates a new Climber. */
  public Climber() {
    m_main = new CANSparkMax(Constants.kClimber.id_main, MotorType.kBrushless);
    m_main.restoreFactoryDefaults();
    m_main.setIdleMode(IdleMode.kBrake);
    m_main.setSmartCurrentLimit(Constants.kClimber.currentLimit);

    m_follower = new CANSparkMax(Constants.kClimber.id_follower, MotorType.kBrushless);
    m_follower.restoreFactoryDefaults();
    m_follower.follow(m_main, true);
    m_follower.setIdleMode(IdleMode.kBrake);
    m_follower.setSmartCurrentLimit(Constants.kClimber.currentLimit);

    // m_controller = m_main.getPIDController();
    // configPID();
    // m_controller.setOutputRange(-1, 1);

    s_encoder = m_main.getEncoder();
    zeroEncoder();

    // limitSwitch = new DigitalInput(Constants.kClimber.port_limitSwitch);
    irSwitch = new DigitalInput(Constants.kClimber.port_irSwitch);

    sb_climberTab = Shuffleboard.getTab("Climber");
    kP = sb_climberTab.add("kP", Constants.kClimber.kP).getEntry();
    kI = sb_climberTab.add("kI", Constants.kClimber.kI).getEntry();
    kD = sb_climberTab.add("kD", Constants.kClimber.kD).getEntry();
    position = sb_climberTab.add("position", 0).getEntry();
    // limitSwitchValue = sb_climberTab.add("limitSwitch", 0).getEntry();
    irSwitchValue = sb_climberTab.add("irSwitch", 0).getEntry();

    m_main.burnFlash();
    m_follower.burnFlash();
  }

  // public void configPID() {
  // m_controller.setP(Constants.kClimber.kP);
  // m_controller.setI(Constants.kClimber.kI);
  // m_controller.setD(Constants.kClimber.kD);
  // }

  public void zeroEncoder() {
    s_encoder.setPosition(0);
  }

  // public void setpoint(double setpoint) {
  // m_controller.setReference(setpoint, ControlType.kPosition);
  // }

  public void manualExtend(double voltage) {
    m_main.setVoltage(voltage);
  }

  // public void zeroEncoderLimit() {
  // if (limitSwitch.get()) {
  // s_encoder.setPosition(0);
  // }
  // }

  public void zeroEncoderIR() {
    if (irSwitch.get()) {
      s_encoder.setPosition(0);
    }
  }

  // public Command setpoint(double setpoint) {
  //   return Commands.runOnce(() -> this.setpoint(setpoint));
  // }

  @Override
  public void periodic() {
    position.setDouble(s_encoder.getPosition());
    // limitSwitchValue.setBoolean(limitSwitch.get());
    irSwitchValue.setBoolean(irSwitch.get());
    // This method will be called once per scheduler run
  }
}
