package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ElevatorIOSparkMax implements ElevatorIO {

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController pid;

    private final ShuffleboardTab sb_Spark;
    private final GenericEntry sb_kp;
    private final GenericEntry sb_ki;
    private final GenericEntry sb_kd;
    private final GenericEntry sb_kff;

    public ElevatorIOSparkMax(int ID) {
        elevatorMotor = new CANSparkMax(ID, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);
        pid = elevatorMotor.getPIDController();
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
        pid.setFF(0);
        elevatorMotor.burnFlash();
        elevatorEncoder = elevatorMotor.getEncoder();

        sb_Spark = Shuffleboard.getTab("Elevator -SparkMax");

        sb_Spark.addDouble("Position", () -> elevatorEncoder.getPosition());

        sb_kp = sb_Spark.add("kP", pid.getP()).getEntry();
        sb_ki = sb_Spark.add("kI", pid.getI()).getEntry();
        sb_kd = sb_Spark.add("kD", pid.getD()).getEntry();
        sb_kff = sb_Spark.add("kFF", pid.getFF()).getEntry();
    }

    @Override
    public void setPosition(double value, int slot) {
        pid.setReference(value, ControlType.kPosition, slot);
    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void resetPosition(double position) {
        elevatorEncoder.setPosition(position);
    }

    @Override
    public void updateInputs(ElevatorInput inputs) {
        inputs.elevatorConnection = !(elevatorMotor.getFault(FaultID.kMotorFault)
                || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANTX));
        inputs.elevatorVolts = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.elevatorCurrent = elevatorMotor.getOutputCurrent();
        inputs.elevatorTemp = elevatorMotor.getMotorTemperature();
        inputs.elevatorPosition = elevatorEncoder.getPosition();
    }

    @Override
    public String getName() {
        return "NEO";
    }

    @Override
    public void debugPID() {
        if (pid.getP() != sb_kp.getDouble(0.0)) {
            pid.setP(sb_kp.getDouble(0.0));
            pid.setI(sb_ki.getDouble(0.0));
            pid.setD(sb_kd.getDouble(0.0));
            pid.setFF(sb_kff.getDouble(0.0));
        }
    }
}
