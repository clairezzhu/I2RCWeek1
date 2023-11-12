// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// importing drive train and double angle

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.USBOrder;
import frc.robot.subsystems.DriveTrain;

public class PIDTurn extends CommandBase {
  /** Creates a new PIDTurn. */
  DriveTrain dt;
  Double angle;

  //kP = max power/max error --> 0.3/90 = 0.00333333
  //max error is the setpoint from the 0
  //kD = 
  PIDController pid = new PIDController(USBOrder.kP, 0, 0);
  Double output;
  Double motorSign;

  public PIDTurn(DriveTrain dt, Double angle) {
    this.dt = dt;
    this.angle = angle;
    addRequirements(dt);
    pid.setTolerance(1.0);
    if(angle >=0){

      motorSign = 1.0;
    } else {
      motorSign = -1.0;
    }
    }


    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavx();
    dt.tankDrive(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //(left, right) - turns CCW or in the positive direction according to the unit circle
    output = pid.calculate (dt.getAngle(), angle);
    dt.tankDrive (-motorSign * output, motorSign * output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
