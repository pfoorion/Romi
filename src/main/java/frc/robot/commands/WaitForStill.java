// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class WaitForStill extends CommandBase {
  private final Drivetrain drivetrain;

  private double previousX = 0.0;
  private double previousY = 0.0;
  private double previousZ = 0.0;
  private int counter = 0;

  public WaitForStill(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    previousX = drivetrain.getAccelX();
    previousY = drivetrain.getAccelY();
    previousZ = drivetrain.getAccelZ();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  private boolean isAccelInTolerance(double reading, double offset) {
      return Math.abs(reading-offset) <= Constants.ACCELEROMOTER_STILL_TOLERANCE;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean result = false;
    if (counter > 10) {
      double newX = drivetrain.getAccelX();
      double newY = drivetrain.getAccelY();
      double newZ = drivetrain.getAccelZ();
  
      SmartDashboard.putNumber("New X Accel Reading", newX);
      SmartDashboard.putNumber("Previous X Accel Reading", previousX);
      SmartDashboard.putNumber("New Y Accel Reading", newY);
      SmartDashboard.putNumber("Previous Y Accel Reading", previousY);
      SmartDashboard.putNumber("New Z Accel Reading", newZ);
      SmartDashboard.putNumber("Previous Z Accel Reading", previousZ);
  
      result = isAccelInTolerance(newX, previousX) &&
      isAccelInTolerance(newY, previousY) &&
      isAccelInTolerance(newZ, previousZ);
      if (!result) {
        previousX = newX;
        previousY = newY;
        previousZ = newZ;
      }  
      counter = 0;
    } else {
      counter++;
    }
    return result;
  }
}
