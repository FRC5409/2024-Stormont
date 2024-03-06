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
public class IntakeToCartridge extends ParallelCommandGroup {

    private final Deployment sys_Deployment;
    private final Intake sys_Intake;
    private final Indexer sys_Indexer;
    private final Cartridge sys_Cartridge;

    public IntakeToCartridge() {
        sys_Deployment = new Deployment();
        sys_Intake = Intake.getInstance();
        sys_Indexer = Indexer.getInstance();
        sys_Cartridge = new Cartridge();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_Deployment, sys_Intake, sys_Indexer, sys_Cartridge);

    }

}