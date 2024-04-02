// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kCartridge;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kIntake;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class BringNoteToCartridge extends SequentialCommandGroup {
    /** Creates a new BringNoteToCartridge. */
    public BringNoteToCartridge(
            Intake sys_intake,
            Indexer sys_indexer,
            Cartridge sys_cartridge) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            Commands.runOnce(() -> {
                sys_intake.setVoltage(kIntake.VOLTAGE);
                sys_indexer.setVoltage(kIndexer.VOLTAGE);
                sys_cartridge.setVoltage(-kCartridge.VOLTAGE);
            }, sys_intake, sys_indexer, sys_cartridge),
            Commands.waitUntil(() -> sys_cartridge.checkir()),
            Commands.runOnce(() -> {
                sys_indexer.setVoltage(0.0);
                sys_cartridge.setVoltage(-kCartridge.VOLTAGE / 2.0);
            }, sys_indexer, sys_cartridge),
            Commands.waitSeconds(0.25),
            Commands.waitUntil(() -> sys_cartridge.checkir()),
            Commands.runOnce(() -> sys_cartridge.setVoltage(0), sys_cartridge)
        );
    }

    /** Creates a new BringNoteToCartridge. */
    public BringNoteToCartridge(
            Indexer sys_indexer,
            Cartridge sys_cartridge) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            Commands.runOnce(() -> {
                sys_indexer.setVoltage(kIndexer.VOLTAGE);
                sys_cartridge.setVoltage(-kCartridge.VOLTAGE);
            }, sys_indexer, sys_cartridge),
            Commands.waitUntil(() -> sys_cartridge.checkir()),
            Commands.runOnce(() -> {
                sys_indexer.setVoltage(0);
                sys_cartridge.setVoltage(-kCartridge.VOLTAGE / 2.0);
            }, sys_indexer, sys_cartridge),
            Commands.waitSeconds(0.25),
            Commands.waitUntil(() -> sys_cartridge.checkir()),
            Commands.runOnce(() -> sys_cartridge.setVoltage(0), sys_cartridge)
        );
    }


}
