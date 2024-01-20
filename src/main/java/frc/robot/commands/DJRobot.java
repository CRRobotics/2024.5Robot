// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


// package frc.robot.commands;

// import java.util.ArrayList;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.music.Orchestra;

// import org.team639.RobotContainer;


// import edu.wpi.first.wpilibj2.command.Command;

// public class DJRobot extends Command {
//   private frc.robot.subsystems.DriveTrain driveTrain;
//   private Orchestra orchestra;
//   private double time;

//   ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

//   /**
//    * Constructs a new DJRobot
//    * @param driveTrain DriveTrain to be used
//    * @param time Time in seconds to play song
//    */
//   public DJRobot(frc.robot.subsystems.DriveTrain driveTrain, double time) {
//     this.driveTrain = driveTrain;
//     this.time = time;
//     addRequirements(driveTrain);
    
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // instruments.add(driveTrain.rightMain);
//     // instruments.add(driveTrain.rightFollower);
//     // instruments.add(driveTrain.leftMain);
//     // instruments.add(driveTrain.leftFollower);
    
//     orchestra = new Orchestra(instruments);
//     // orchestra.loadMusic(RobotContainer.songChooser());
//     System.out.println("DJRobot Initializing");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     orchestra.play();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("DJRobot dropping the mic");
//     orchestra.stop();
//     orchestra.clearInstruments();

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(orchestra.getCurrentTime() / 1000 > time)
//       return true;
//     return false;
//   }
// }
