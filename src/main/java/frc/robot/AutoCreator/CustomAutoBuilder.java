package frc.robot.AutoCreator;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
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

    private static HashMap<String, Pose2d> m_startingPos;

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

        m_startingPos =  new HashMap<>();
      }

    public static SendableChooser<Command> buildChooser() {
        SendableChooser<Command> sc_chooser = new SendableChooser<>();

        for (String name : AutoBuilder.getAllAutoNames()) {
            sc_chooser.addOption(name, buildAuto(name));
        }

        sc_chooser.setDefaultOption("None", Commands.none());

        sc_chooser.onChange(cmd -> resetPosition(cmd));

        return sc_chooser;
    }

    private static void resetPosition(Command autoCommand) {
        String autoName = autoCommand.getName();

        Pose2d startingPose = m_startingPos.get(autoName);

        if (startingPose == null) 
            return;

        if (m_shouldFlipPath.getAsBoolean())
            m_resetPose.accept(GeometryUtil.flipFieldPose(startingPose));
        else
            m_resetPose.accept(startingPose);
    }

    /**
     * TAKEN FROM PATH PLANNER
     */
    public static JSONObject loadJSON(String autoName) {
        try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto")))) {
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

        JSONObject startingPose = (JSONObject) JSONAuto.get("startingPose");

        JSONObject position = (JSONObject) startingPose.get("position");

        Pose2d pose = 
            new Pose2d(
                (double) position.get("x"),
                (double) position.get("y"),
                Rotation2d.fromDegrees(((Long) startingPose.get("rotation")).doubleValue())
            );

        m_startingPos.put(autoName, pose);

        return getCommandFromJSON((JSONObject) JSONAuto.get("command")).withName(autoName);
    }

    public static Command[] getCommandsFromJSONArray(JSONArray arr) {
        Command[] commands = new Command[arr.size()];
        for (int i = 0; i < arr.size(); i++) {
            JSONObject commandData = (JSONObject) arr.get(i);
            commands[i] = getCommandFromJSON(commandData);
        }

        return commands;
    }

    public static Command buildPathCommand(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    }

    public static Command getCommandFromJSON(JSONObject jsonData) {
        String type = (String) jsonData.get("type");
        JSONObject data = (JSONObject) jsonData.get("data");

        switch (type) {
            case "wait":
                return new WaitCommand((double) data.get("waitTime"));
            case "named":
                return NamedCommands.getCommand((String) data.get("name"));
            case "path":
                return buildPathCommand((String) data.get("pathName"));
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
            case "until":
                return getCommandFromJSON((JSONObject) data.get("command")).until(NamedConditions.getCondition((String) data.get("condition")));
        }

        return Commands.none();
    }

    /**
     * Builds a auto command that will follow different paths based of conditions
     * @param condtions The NamedConditions that the condtion will follow
     * @param pathNames The Path Names associated with the condtions
     * @param otherwise If None of the condtions are true, follow this path
     * @return New Command 
     */
    public static Command buildCaseCommand(String[] condtions, String[] pathNames, String otherwise) {
        return buildCaseCommand(condtions, pathNames, buildPathCommand(otherwise));
    }

    /**
     * Builds a auto command that will follow different paths based of conditions
     * @param condtions The NamedConditions that the condtion will follow
     * @param pathNames The Path Names associated with the condtions
     * @param otherwise If None of the conditions are true, run this command
     * @return New Command 
     */
    public static Command buildCaseCommand(String[] condtions, String[] pathNames, Command otherwise) {
        Command[] commands = new Command[pathNames.length];
        for (int i = 0; i < pathNames.length; i++) {
            commands[i] = buildPathCommand(pathNames[i]);
        }
        return buildCaseCommand(condtions, commands, otherwise, 0);
    }

    /**
     * Builds a auto command that will follow different paths based of conditions
     * @param condtions NamedCondtions to follow
     * @param pathCommands The commands to run when the following conditon is true
     * @param otherwise If none of the conditions are true
     * @return New Command
     */
    public static Command buildCaseCommand(String[] condtions, Command[] pathCommands, Command otherwise) {
        return buildCaseCommand(condtions, pathCommands, otherwise, 0);
    }

    private static Command buildCaseCommand(String[] condtions, Command[] pathCommands, Command otherwise, int i) {
        if (condtions.length != pathCommands.length) {
            throw new IllegalArgumentException("Condtions and paths are not the same length!");
        }

        if (i == condtions.length) {
            return otherwise;
        }

        return new ConditionalCommand(
            pathCommands[i],
            buildCaseCommand(condtions, pathCommands, otherwise, i + 1),
            NamedConditions.getCondition(condtions[i])
        );
    }
}
