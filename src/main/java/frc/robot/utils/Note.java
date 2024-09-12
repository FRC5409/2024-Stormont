package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class Note {

    public static Translation3d noteSize = new Translation3d(0.36, 0.36, 0.05);

    private StructPublisher<Pose3d> publisher;
    private Pose3d notePose;
    private Transform3d speed;
    private String name;

    private Pose3d startingPose;
    
    public Note(Pose2d fieldPos, String noteName) {
        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Note: " + noteName, Pose3d.struct).publish();

        startingPose = new Pose3d(new Translation3d(fieldPos.getX(), fieldPos.getY(), noteSize.getZ() / 2.0), new Rotation3d());
        resetPose();

        speed = new Transform3d();

        name = noteName;
    }

    public void setPose(Pose3d pose3d) {
        notePose = pose3d;

        publisher.accept(notePose);
    }

    public void setPose(Pose2d pose2d) {
        setPose(new Pose3d(pose2d.getX(), pose2d.getY(), noteSize.getZ() / 2.0, new Rotation3d()));
    }

    public void resetPose() {
        setPose(startingPose);
    }

    public void setAngle(Rotation3d rotation) {
        setPose(new Pose3d(notePose.getTranslation(), rotation));
    }

    public Pose3d getPose() {
        return notePose;
    }

    public Translation3d getSize() {
        return noteSize;
    }

    public void setSpeed(Transform3d noteSpeed) {
        speed = noteSpeed;
    }

    public Transform3d getSpeed() {
        return speed;
    }

    public void update() {
        if (Math.abs(speed.getX()) + Math.abs(speed.getY()) >= 0.01)
            speed = speed.plus(new Transform3d(new Translation3d(0, 0, -9.8 / 2500.0), new Rotation3d()));
            
        notePose = notePose.plus(speed);
        setPose(notePose);
    }

    @Override
    public String toString() {
        return "NOTE: {Name :" + name + ", POSE: " + notePose.toString() + "}";
    }
}
