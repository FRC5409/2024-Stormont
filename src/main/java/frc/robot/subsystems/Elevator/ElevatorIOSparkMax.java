package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController pid;
    private double kP = 0.0, kI = 0.0, kD = 0.0, kff = 0;
    private final ShuffleboardTab sb_spark;
    private final GenericEntry sb_kp;
    private final GenericEntry sb_ki;
    private final GenericEntry sb_kd;
    private final GenericEntry sb_kff;

    public ElevatorIOSparkMax(int ID) {
        elevatorMotor = new CANSparkMax(ID, MotorType.kBrushless);
        
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(true);

        this.elevatorEncoder = elevatorMotor.getEncoder();

        pid = elevatorMotor.getPIDController();
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setFF(kff);
        
        sb_spark = Shuffleboard.getTab("Spark-Climber");

        sb_spark.addDouble("Position-Neo", () -> elevatorEncoder.getPosition());
        
        //Change Values Through Shuffleboard
        sb_kp = sb_spark.add("kP", kP).getEntry();
        sb_ki = sb_spark.add("kI", kI).getEntry();
        sb_kd = sb_spark.add("kD", kD).getEntry();
        sb_kff = sb_spark.add("kFF", kff).getEntry();

        elevatorMotor.burnFlash();
    }

    // Get subsystem
    @Override
    public void setVoltage(double volts){
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void setPosition(double value, int slot){
        pid.setReference(value,ControlType.kPosition, slot);
        System.out.println(value);
    }

    public void updateInputs(ElevatorInputs inputs){
        inputs.motorConnectedNEO = !(elevatorMotor.getFault(FaultID.kMotorFault) || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANTX));

        inputs.motorVoltsNEO = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrentNEO = elevatorMotor.getOutputCurrent();
        inputs.motorTempNEO = elevatorMotor.getMotorTemperature();

        inputs.motorPositionNEO = elevatorEncoder.getPosition();
    }

    @Override
    public void resetPosition(double resetValue){
        if(0<=resetValue || resetValue<=1.1){
            elevatorEncoder.setPosition(resetValue);
        } else if(resetValue<0){
            resetValue = 0;
            elevatorEncoder.setPosition(resetValue);
        } else if(resetValue>=1.2){
            resetValue = 1.1;
            elevatorEncoder.setPosition(resetValue);
        }
    }

    @Override
    public void debugPID(){
        if(pid.getP() != sb_kp.getDouble(0.0)){
            pid.setP(sb_kp.getDouble(0.0));
        }
        if(pid.getI() != sb_ki.getDouble(0.0)){
            pid.setI(sb_ki.getDouble(0.0));
        }
        if(pid.getD() != sb_kd.getDouble(0.0)){
            pid.setD(sb_kd.getDouble(0.0));
        }
        if(pid.getFF() != sb_kff.getDouble(0.0)){
            pid.setFF(sb_kff.getDouble(0.0));
        }
    }

    @Override
    public String getName(){
        return "Neo";
    }
    
}
