// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kClimber;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_main;
  private final CANSparkMax m_follower;

  private final SparkPIDController m_controller;
  private final RelativeEncoder s_encoder;
  // private DigitalInput limitSwitch;
  // private DigitalInput irSwitch;

  // private final ShuffleboardTab sb_climberTab;

  /** Creates a new Climber. */
  public Climber() {

    // Configurate motor 1
    m_main = new CANSparkMax(kCANID.CLIMBER_MAIN_ID, MotorType.kBrushless);
    m_main.restoreFactoryDefaults();
    m_main.setIdleMode(IdleMode.kBrake);
    m_main.setSmartCurrentLimit(kClimber.CURRENT_LIMIT);

    // Configurate motor 2
    m_follower = new CANSparkMax(kCANID.CLIMBER_FOLLOWER_ID, MotorType.kBrushless);
    m_follower.restoreFactoryDefaults();
    m_follower.follow(m_main, true);
    m_follower.setIdleMode(IdleMode.kBrake);
    m_follower.setSmartCurrentLimit(kClimber.CURRENT_LIMIT);

    // PID controller
    m_controller = m_main.getPIDController();
    configPID();

    // Encoder
    s_encoder = m_main.getEncoder();
    s_encoder.setPositionConversionFactor(kClimber.CONVERSION_FACTOR);
    zeroEncoder();
    // s_encoder.setInverted(true);

    // Limit Switches
    // limitSwitch = new DigitalInput(kClimber.port_limitSwitch);
    // irSwitch = new DigitalInput(kClimber.port_irSwitch);

    // Shuffleboard
    // sb_climberTab = Shuffleboard.getTab("Climber");
    // sb_climberTab.addDouble("kP", () -> m_controller.getP());
    // sb_climberTab.addDouble("kI", () -> m_controller.getI());
    // sb_climberTab.addDouble("kD", () -> m_controller.getD());
    // sb_climberTab.addDouble("position", () -> s_encoder.getPosition());
    // sb_climberTab.addBoolean("irSwitch", () -> !irSwitch.get());
    // sb_climberTab.addBoolean("limitSwitch", () -> limitSwitch.get());

    m_main.burnFlash();
    m_follower.burnFlash();
  }

  // Configurate PID
  public void configPID() {
    m_controller.setP(kClimber.KP_0, 0);
    m_controller.setI(kClimber.KI_0, 0);
    m_controller.setD(kClimber.KD_0, 0);
    m_controller.setOutputRange(-1.0, 1.0, 0);

    m_controller.setP(kClimber.KP_0, 1);
    m_controller.setI(kClimber.KI_0, 1);
    m_controller.setD(kClimber.KD_0, 1);
    m_controller.setOutputRange(-0.4, 0.4, 1);
  }

  // Zero encoder
  public void zeroEncoder() {
    s_encoder.setPosition(0);
  }

  /**
   * Gets the position of the climber
   * 
   * @return The encoders position
   */
  public double getPosition() {
    return s_encoder.getPosition();
  }

  /**
   * Sets the position of the climber
   * 
   * @param setpoint value
   */
  public void setPosition(double setpoint, int slot) {
    m_controller.setReference(setpoint, ControlType.kPosition, slot);
  }

  /**
   * Sets voltage for manual extension
   * 
   * @param voltage value
   */
  public void setVoltage(double voltage) {
    m_main.setVoltage(voltage);
  }

  // Zeros encoder using limitswtich
  // public void zeroEncoderLimit() {
  // if (!limitSwitch.get()) {
  // s_encoder.setPosition(0);
  //
  // }

  // Zeros encoder using IR sensor
  // public void zeroEncoderIR() {
  // if (!irSwitch.get()) {
  // s_encoder.setPosition(0);
  // }
  // }

  // public Command setpoint(double setpoint) {
  // return Commands.runOnce(() -> this.setpoint(setpoint));
  // }

  @Override
  public void periodic() {
    // zeroEncoderIR();
  }
}

// max limit is -50.7
// min limit is -0.5

// max height is 13.82 inches, 35.1 cm
