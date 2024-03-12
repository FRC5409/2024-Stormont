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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Deployment extends SubsystemBase {
  // motor
  private final CANSparkMax m_motor;
  // controller
  private final SparkPIDController m_controller;
  // encoder
  private final RelativeEncoder s_encoder;

  private static Deployment instance = null;
  // sensor
  // private DigitalInput limitSwitch;
  // private DigitalInput irSwitch;

  // shuffleboard/generic entry
  private final ShuffleboardTab sb_DeploymentTab;
  private final GenericEntry kP, kI, kD;

  /**
   * Creates a new Deployment.
   * One CANSparkMax
   */
  public Deployment() {
    // initialize motor
    m_motor = new CANSparkMax(Constants.kDeployment.id_motor, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(Constants.kDeployment.currentLimit);

    // initialize controller
    m_controller = m_motor.getPIDController();
    configPID();

    // initialize encoder
    s_encoder = m_motor.getEncoder();
    zeroEncoder();

    // initiaize snesor
    // limitSwitch = new DigitalInput(Constants.kDeployment.digitalInputPort);
    // irSwitch = new DigitalInput(Constants.kDeployment.port_irSwitch);

    /*
     * set up shuffleboard tabs for deployment
     * PID
     * position of motor
     * IR switch value for distance of the elevator
     */
    sb_DeploymentTab = Shuffleboard.getTab("Deployment");
    kP = sb_DeploymentTab.add("kP", Constants.kDeployment.kP).getEntry();
    kI = sb_DeploymentTab.add("kI", Constants.kDeployment.kI).getEntry();
    kD = sb_DeploymentTab.add("kD", Constants.kDeployment.kD).getEntry();
    sb_DeploymentTab.addDouble("position", () -> getPosition());
    // sb_DeploymentTab.addBoolean("irSwitch", () -> !irSwitch.get());
    // sb_DeploymentTab.addBoolean("limitSwitch", () -> !limitSwitch.get());

    m_motor.burnFlash();
  }

  public static Deployment getInstance() {
    if (instance == null)
      instance = new Deployment();

    return instance;
  }

  /**
   * Sets PID value for motor controller
   * 
   * @param PID value
   */

  public void configPID() {
    m_controller.setP(Constants.kDeployment.kP);
    m_controller.setI(Constants.kDeployment.kI);
    m_controller.setD(Constants.kDeployment.kD);
    m_controller.setOutputRange(-0.5, 0.5);
  }

  public double getPosition() {
    return s_encoder.getPosition();
  }

  public void stopMot() {
    m_motor.stopMotor();
  }

  /**
   * Zero encoder with encoder
   * 
   * @param position
   */

  public void zeroEncoder() {
    s_encoder.setPosition(0);
  }

  public boolean atSetpoint(double setpoint) {
    return MathUtil.isNear(setpoint, getPosition(), Constants.kDeployment.kTolerance);
  }

  /**
   * Zero encoder with IR censor
   * 
   * @return distance from irswitch
   */

  // public void zeroEncoderIR() {
  // if (!irSwitch.get()) {
  // s_encoder.setPosition(0);
  // }
  // }

  /**
   * Motor go to setpoint from constants
   * 
   * @param setpoint    value
   * @param outputVolts value 0-12
   */

  public void setpoint(double setpoint) {
    m_controller.setReference(setpoint, ControlType.kPosition);
  }

  /**
   * Sets voltage for manual extension
   * 
   * @param voltage
   */

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
    // zeroEncoderIR();

    // fixEncoder();
    // This method will be called once per scheduler run
  }

}
