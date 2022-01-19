// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCircle extends CommandBase {
  private final Drivetrain driveTrain;
  private final boolean turnLeft;
  private final double speed;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveCircle(double speed, boolean turnLeft, Drivetrain drive) {
    this.speed = speed;
    this.turnLeft = turnLeft;
    this.driveTrain = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.arcadeDrive(0, 0);
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(speed, turnLeft?-0.25:0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    double inchPerDegree = Math.PI * 5.551 / 360;
    // Compare distance travelled from start to distance based on degree turn
    return getAverageTurningDistance() >= (inchPerDegree * 360);
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(driveTrain.getLeftDistanceInch());
    double rightDistance = Math.abs(driveTrain.getRightDistanceInch());
    return (leftDistance + rightDistance) / 2.0;
  }
}
