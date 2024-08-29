package frc.robot.AutoCreator;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Filesystem;

public class Builder {
    public Builder() {

    }

    public HashMap<String, PathPlannerTrajectory> loadPaths() {
        HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();

        List<String> pathNames = getAllPathNames();
        List<PathPlannerTrajectory> pathTrajectories = loadTrajectories(pathNames);

        return trajectories;
    }

    public void buildAuto(String auto) {

    }

    /**
     * Get a list of all path names in the project
     *
     * @return List of all path names
     */
    public static List<String> getAllPathNames() {
        File[] autoFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/path").listFiles();

        if (autoFiles == null) {
            return new ArrayList<>();
        }

        return Stream.of(autoFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".path"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
    }

    public static List<PathPlannerTrajectory> loadTrajectories(List<String> pathNames) {
        return null;
    }
}
