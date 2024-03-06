package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.kDeployment;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kCartridge;

import frc.robot.subsystems.Deployment;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Cartridge;

/**
 * @author
 */
public class IntakeToCartridge extends Command {

    private final Intake sys_Intake;
    private final Indexer sys_Indexer;
    private final Cartridge sys_Cartridge;

    public IntakeToCartridge() {
        sys_Intake = Intake.getInstance();
        sys_Indexer = Indexer.getInstance();
        sys_Cartridge = Cartridge.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_Intake, sys_Indexer, sys_Cartridge);

    }

    @Override
    public void initialize() {
        sys_Intake.setVoltage(kIntake.VOLTAGE);
        sys_Indexer.setVoltage(kIndexer.VOLTAGE);
        sys_Cartridge.roll(-kCartridge.voltage);
    }

    @Override
    public void end(boolean interrupted) {
        sys_Indexer.setVoltage(0);
        sys_Intake.setVoltage(0);
        sys_Cartridge.roll(0);
    }

    @Override
    public boolean isFinished() {
        return sys_Cartridge.checkir();
    }

}