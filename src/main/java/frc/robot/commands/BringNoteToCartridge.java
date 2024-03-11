package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kCartridge;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Cartridge;

/**
 * @author
 */
public class BringNoteToCartridge extends Command {

    private final Indexer sys_Indexer;
    private final Cartridge sys_Cartridge;

    public BringNoteToCartridge(Cartridge cartridge, Indexer indexer) {
        sys_Indexer = Indexer.getInstance();
        sys_Cartridge = Cartridge.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_Indexer, sys_Cartridge);
    }

    @Override
    public void initialize() {
        sys_Indexer.setVoltage(kIndexer.VOLTAGE);
        sys_Cartridge.roll(-kCartridge.voltage);

    }

    @Override
    public void end(boolean interrupted) {
        sys_Indexer.setVoltage(0);
        sys_Cartridge.roll(0);
    }

    @Override
    public boolean isFinished() {
        return sys_Cartridge.checkir();
    }

}