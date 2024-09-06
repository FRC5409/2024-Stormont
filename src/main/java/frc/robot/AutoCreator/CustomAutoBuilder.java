package frc.robot.AutoCreator;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CustomAutoBuilder {

    public static Consumer<Pose2d> m_resetPose;
    public static BooleanSupplier m_shouldFlipPath;

    public static void configureHolonomic(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> robotRelativeOutput,
      HolonomicPathFollowerConfig config,
      BooleanSupplier shouldFlipPath,
      Subsystem driveSubsystem) {
        AutoBuilder.configureHolonomic(poseSupplier, resetPose, robotRelativeSpeedsSupplier, robotRelativeOutput, config, shouldFlipPath, driveSubsystem);
        m_resetPose = resetPose;
        m_shouldFlipPath = shouldFlipPath;
      }

    public static SendableChooser<Command> buildChooser() {
        SendableChooser<Command> sc_chooser = new SendableChooser<>();

        for (String name : AutoBuilder.getAllAutoNames()) {
            sc_chooser.addOption(name, AutoBuilder.buildAuto(name));
        }

        for (String name : getCustomAutoNames()) {
            sc_chooser.addOption(name, buildAuto(name));
        }

        sc_chooser.setDefaultOption("None", Commands.none());

        return sc_chooser;
    }

    /**
     * TAKEN FROM PATH PLANNER
     */
    public static List<String> getCustomAutoNames() {
        File[] autoFiles = new File(Filesystem.getDeployDirectory(), "autoplanner/autos").listFiles();

        if (autoFiles == null) {
          return new ArrayList<>();
        }

        return Stream.of(autoFiles)
            .filter(file -> !file.isDirectory())
            .map(File::getName)
            .filter(name -> name.endsWith(".auto"))
            .map(name -> name.substring(0, name.lastIndexOf(".")))
            .collect(Collectors.toList());
    }

    /**
     * TAKEN FROM PATH PLANNER
     */
    public static JSONObject loadJSON(String autoName) {
        try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "autoplanner/autos/" + autoName + ".auto")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
              fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            return json;
        } catch (Exception e) {
            throw new RuntimeException(String.format("Error building auto: %s", autoName), e);
        }
    }

    public static Command buildAuto(String autoName) {
        JSONObject JSONAuto = (JSONObject) loadJSON(autoName);

        JSONObject startingPos = (JSONObject) JSONAuto.get("startingPos");

        System.out.println(startingPos.toJSONString());

        Pose2d startingPose = new Pose2d((double) startingPos.get("x"), (double) startingPos.get("y"), Rotation2d.fromDegrees((double) startingPos.get("rot")));

        Command resetCommand = Commands.runOnce(() -> {
            if (m_shouldFlipPath.getAsBoolean()) {
                m_resetPose.accept(GeometryUtil.flipFieldPose(startingPose));
            } else {
                m_resetPose.accept(startingPose);
            }
        });

        return resetCommand.andThen(getCommandFromJSON((JSONObject) JSONAuto.get("command")));
    }

    public static Command[] getCommandsFromJSONArray(JSONArray arr) {
        Command[] commands = new Command[arr.size()];
        for (int i = 0; i < arr.size(); i++) {
            JSONObject commandData = (JSONObject) arr.get(i);
            commands[i] = getCommandFromJSON(commandData);
        }

        return commands;
    }

    public static Command getCommandFromJSON(JSONObject data) {
        String type = (String) data.get("type");

        switch (type) {
            case "wait":
                return new WaitCommand((double) data.get("waitTime"));
            case "named":
                return NamedCommands.getCommand((String) data.get("name"));
            case "path":
                return AutoBuilder.followPath(PathPlannerPath.fromPathFile((String) data.get("name")));
            case "sequential":
                return new SequentialCommandGroup(getCommandsFromJSONArray((JSONArray) data.get("commands")));
            case "parallel":
                return new ParallelCommandGroup(getCommandsFromJSONArray((JSONArray) data.get("commands")));
            case "race":
                return new ParallelRaceGroup(getCommandsFromJSONArray((JSONArray) data.get("commands")));
            case "deadline":
                Command[] commands = getCommandsFromJSONArray((JSONArray) data.get("commands"));

                // First command is the deadline command, then the rest of the commands
                return new ParallelDeadlineGroup(commands[0], Stream.of(commands).skip(1).toArray(Command[]::new));
            case "conditional":
                JSONArray JSONCommands = (JSONArray) data.get("commands");

                return new ConditionalCommand(getCommandFromJSON((JSONObject) JSONCommands.get(0)), getCommandFromJSON((JSONObject) JSONCommands.get(1)), NamedConditions.getCondition((String) data.get("condition")));
          }

        return Commands.none();
    }
}
