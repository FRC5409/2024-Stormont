package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.Supplier;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kDeployment;

public class NoteVisualizer {

    private static final double shotSpeed = 7.0 / 20.0;

    private static Supplier<Pose2d> m_robotPose;
    private static Supplier<Double> m_shooterAngle;
    private static Translation2d m_robotSize;
    private static Supplier<Transform3d> m_noteOffset;

    private static boolean isConfigured = false;

    private static int carryingNote = -1;

    private static ArrayList<Note> notes;

    private static final Transform3d AMP_NOTE_OFFSET = new Transform3d(new Translation3d(-0.05, 0.0, 0.15), new Rotation3d());

    private static final Pose2d[] NOTE_FIELD_LOCATIONS = {
        // Blue Close notes
        new Pose2d(new Translation2d(2.89, 7.00), Rotation2d.fromDegrees(0.0)),
        new Pose2d(new Translation2d(2.89, 5.54), Rotation2d.fromDegrees(0.0)),
        new Pose2d(new Translation2d(2.89, 4.10), Rotation2d.fromDegrees(0.0)),

        // Mid notes
        new Pose2d(new Translation2d(8.29, 7.44), Rotation2d.fromDegrees(0.0)),
        new Pose2d(new Translation2d(8.29, 5.78), Rotation2d.fromDegrees(0.0)),
        new Pose2d(new Translation2d(8.29, 4.12), Rotation2d.fromDegrees(0.0)),
        new Pose2d(new Translation2d(8.29, 2.46), Rotation2d.fromDegrees(0.0)),
        new Pose2d(new Translation2d(8.29, 0.80), Rotation2d.fromDegrees(0.0))
    };

    public static void configureNoteVisualizer(Supplier<Pose2d> robotPose, Supplier<Double> shooterAngle, Translation2d robotSize, Supplier<Transform3d> noteOffset) {
        m_robotPose = robotPose;
        m_shooterAngle = shooterAngle;
        m_robotSize = robotSize;
        m_noteOffset = noteOffset;

        notes = new ArrayList<>();

        isConfigured = true;
    }

    public static void createNotes() {
        if (!isConfigured) {
            throw new IllegalArgumentException("Note Visualizer is not configured!");
        }

        for (int i = 0; i < NOTE_FIELD_LOCATIONS.length; i++) {
            int noteNum = i;
            if (i >= 3)
                noteNum -= 3;

            noteNum++;
            if (i >= 3) 
                notes.add(new Note(NOTE_FIELD_LOCATIONS[i], "MNOTE" + noteNum));
            else {
                notes.add(new Note(NOTE_FIELD_LOCATIONS[i], "CNOTE" + noteNum + "_B"));
                notes.add(new Note(GeometryUtil.flipFieldPose(NOTE_FIELD_LOCATIONS[i]), "CNOTE" + noteNum + "_R"));
            }
        }
    }

    public static boolean updateNotes() {
        if (!isConfigured) {
            throw new IllegalArgumentException("Note Visualizer is not configured!");
        }

        for (int i = 0; i < notes.size(); i++) {
            Note note = notes.get(i);

            if (carryingNote == i) {
                Pose2d robotPose = m_robotPose.get();
                Pose3d notePose = new Pose3d(robotPose.getX(), robotPose.getY(), 0.0, new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));

                if (m_noteOffset.get().getZ() >= kDeployment.MIN_HEIGHT * 1.5)
                    notePose = notePose.plus(AMP_NOTE_OFFSET); // Stupid fix for the amp...

                notePose = notePose.plus(m_noteOffset.get());

                note.setPose(notePose);
            } else {
                note.update();

                if (note.getPose().getTranslation().toTranslation2d().getDistance(NOTE_FIELD_LOCATIONS[5].getTranslation()) > 16.5) {
                    note.resetPose();
                    note.setSpeed(new Transform3d());
                }
            }
        }

        if (carryingNote == -1) 
            return attachTouchingNotes();

        return false;
    }

    private static boolean attachTouchingNotes() {
        for (int i = 0; i < notes.size(); i++) {
            if (Math.abs(notes.get(i).getSpeed().getX()) >= 0.001)
                continue;

            Pose2d notePose = notes.get(i).getPose().toPose2d();

            if (checkIfPoseContactPose(m_robotPose.get(), m_robotSize, notePose, Note.noteSize.toTranslation2d())) {
                carryingNote = i;
                return true;
            }
        }

        return false;
    }

    private static boolean checkIfPoseContactPose(Pose2d pose1, Translation2d size1, Pose2d pose2, Translation2d size2) {
        Translation2d center1 = pose1.getTranslation();
        Translation2d center2 = pose2.getTranslation();

        double radius1 = Math.sqrt(Math.pow(size1.getX(), 2) + Math.pow(size1.getY(), 2)) / 2.0;
        double radius2 = Math.sqrt(Math.pow(size2.getX(), 2) + Math.pow(size2.getY(), 2)) / 2.0;

        double distance = center1.getDistance(center2);

        return distance <= (radius1 + radius2);
    }

    public static Command generateNoteShootingVisualizationCommand() {
        if (!isConfigured) {
            throw new IllegalArgumentException("Note Visualizer is not configured!");
        }

        return Commands.runOnce(() -> {
            if (carryingNote == -1) 
                return;

            Note note = notes.get(carryingNote);
            carryingNote = -1;

            double shooterAngle = m_shooterAngle.get();
            Rotation2d robotRotation = m_robotPose.get().getRotation();

            note.setAngle(new Rotation3d(0.0, shooterAngle, robotRotation.getRadians()));

            note.setSpeed(
                new Transform3d(
                    new Translation3d(
                        -shotSpeed,
                        0.0,
                        0.0
                    ),
                    new Rotation3d()
                )
            );
        });
    }

    public static Command generateNoteAmpingVisualizationCommand() {
        if (!isConfigured) {
            throw new IllegalArgumentException("Note Visualizer is not configured!");
        }

        return Commands.runOnce(() -> {
            if (carryingNote == -1) 
                return;

            Note note = notes.get(carryingNote);
            carryingNote = -1;

            double shooterAngle = Math.toRadians(-75);
            Rotation2d robotRotation = m_robotPose.get().getRotation();

            note.setAngle(new Rotation3d(0.0, shooterAngle, robotRotation.getRadians()));

            note.setSpeed(
                new Transform3d(
                    new Translation3d(
                        -0.08,
                        0.0,
                        0.01
                    ),
                    new Rotation3d()
                )
            );
        });
    }
}
