
public class DriveToRealativePoint extends SequentialCommandGroup {
    DriveTrain driveTrain;


    public DriveToRealativePoint(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
              new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
              new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
              new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
              bezierPoints,
              new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
              new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;

        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            target,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        addCommands(
            pathfindingCommand);
    }
}
