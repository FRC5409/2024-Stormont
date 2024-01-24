package frc.robot.shuffleboard;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardManager {
    private final ShuffleboardTab tab;
	private final HashMap<String, GenericEntry>	entries;

    public ShuffleboardManager(String tabName) {
        tab = Shuffleboard.getTab(tabName);
        entries = new HashMap<>();
    }

    public void addEntry(String entry, Supplier<?> value) {
        entries.put(entry, tab.add(entry, value).getEntry());
    }
}
