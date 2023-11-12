// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive extends CommandBase {
  DriveTrain dt;
  double setpoint;
  /** Creates a new AutoDrive. */
  public AutoDrive(DriveTrain dt, double setpoint) {
    this.dt = dt;
    this.setpoint = setpoint;
    addRequirements(dt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    dt.tankDrive (0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.tankDrive (0.3, 0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.resetEncoders();
    dt.tankDrive (0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (dt.ticksToMeter() >= setpoint) {
      return true;
    }
    else {
      return false;
    }
  }
}
