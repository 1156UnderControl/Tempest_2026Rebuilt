package frc.robot.subsystems.shooter;

public record ShooterParameters(
    double turretTargetAngle,
    double hoodTargetAngle,
    double flyWheelsTargetVelocity,
    double distanceToTarget) {}