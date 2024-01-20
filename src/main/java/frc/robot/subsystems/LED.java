package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kOperator;
import frc.robot.Utils.Color;

// 5409: The Chargers
// http://github.com/FRC5409

public class LED extends SubsystemBase {

    public enum LEDState {
        kSolid(0), kBlink(1), kSinWave(2), kSinFlow(3), kRainbowCycle(4), kRainbowBlink(5);
        
        LEDState(int value) {
            state = value;
        }
        public final int state;
    }

    private static LED instance = null;

    private final ShuffleboardTab communicationTab;
    private final GenericEntry sb_error;

    private LEDState state;
    private Color primaryColor;
    private Color secondaryColor;


    private LED() {
        state = LEDState.kRainbowCycle;
        primaryColor = Color.kRed;
        secondaryColor = Color.kBlue;

        communicationTab = Shuffleboard.getTab("LED_COMMUNICATION_TAB");
        sb_error = communicationTab.add("Error Code:", "Don't stress").getEntry();

        communicationTab.addInteger("STATE",                () -> state.state);
        communicationTab.addIntegerArray("PRIMARY_COLOR",   () -> Arrays.stream(primaryColor.getColor()).mapToLong(i -> i).toArray());
        communicationTab.addIntegerArray("SECONDARY_COLOR", () -> Arrays.stream(secondaryColor.getColor()).mapToLong(i -> i).toArray());
    }

    // Get subsystem
    public static LED getInstance() {
        if (instance == null) instance = new LED();

        return instance;
    }

    /**
     * Set a error message to the robot
     * @param errorCode
     */
    public void error(String errorCode) {
        sb_error.setString(errorCode);
        if (DriverStation.isDisabled()) {
            state = LEDState.kBlink;
            primaryColor = Color.kRed;
            secondaryColor = Color.kBlack;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (DriverStation.isDisabled()) {
            // Error codes
            if (!DriverStation.isJoystickConnected(kOperator.primaryControllerPort)) {
                error("Joystick: " + kOperator.primaryControllerPort);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}