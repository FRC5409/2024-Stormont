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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Deployment extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final SparkPIDController m_controller;
  private final RelativeEncoder s_encoder;
  // private DigitalInput limitSwitch;
  private DigitalInput irSwitch;

  private final ShuffleboardTab sb_DeploymentTab;
  private final GenericEntry kP, kI, kD, position, limitSwitchValue;

  /** Creates a new Deployment. */
  public Deployment() {
    m_motor = new CANSparkMax(Constants.kDeployment.id_motor, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(Constants.kDeployment.currentLimit);

    m_controller = m_motor.getPIDController();
    configPID();
    // m_controller.setOutputRange(-1, 1);

    s_encoder = m_motor.getEncoder();

    // limitSwitch = new DigitalInput(Constants.kDeployment.digitalInputPort);
    irSwitch = new DigitalInput(Constants.kDeployment.port_irSwitch);

    sb_DeploymentTab = Shuffleboard.getTab("Deployment");
    kP = sb_DeploymentTab.add("kP", Constants.kDeployment.kP).getEntry();
    kI = sb_DeploymentTab.add("kI", Constants.kDeployment.kI).getEntry();
    kD = sb_DeploymentTab.add("kD", Constants.kDeployment.kD).getEntry();
    position = sb_DeploymentTab.add("position", 0).getEntry();
    limitSwitchValue = sb_DeploymentTab.add("limitSwitch", 0).getEntry();

    m_motor.burnFlash();
  }

  public void configPID() {
    m_controller.setP(Constants.kDeployment.kP);
    m_controller.setI(Constants.kDeployment.kI);
    m_controller.setD(Constants.kDeployment.kD);
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

  // public void fixEncoder() {
  // if (limitSwitch.get()) {
  // s_encoder.setPosition(0);
  // }
  // }

  @Override
  public void periodic() {
    position.setDouble(s_encoder.getPosition());
    // limitSwitchValue.setBoolean(limitSwitch.get());
    // fixEncoder();
    // This method will be called once per scheduler run
  }
}
