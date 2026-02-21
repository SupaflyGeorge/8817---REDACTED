package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

@SuppressWarnings("unused")
public class RobotContainer {
  private final double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Vision vision = new Vision();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();

  // Teleop field-centric request
  private final SwerveRequest.FieldCentric teleopFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Robot-relative speeds request (perfect for tag strafe/approach)
  private final SwerveRequest.ApplyRobotSpeeds robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  
  private String autoName = "New Auto"; 

  public RobotContainer() {
    drivetrain.configurePathPlanner();
   
    configureBindings();
  }

  private void configureBindings() {
    // Default teleop
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          double vx = -joystick.getLeftY() * MaxSpeed;
          double vy = -joystick.getLeftX() * MaxSpeed;
          double omega = -joystick.getRightX() * MaxAngularRate;

          return teleopFieldCentric
              .withVelocityX(vx)
              .withVelocityY(vy)
              .withRotationalRate(omega);
        })
    );


    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

  
    // Reset field-centric heading
    joystick.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
  
    joystick.leftBumper().whileTrue(
        drivetrain.applyRequest(() -> {
          double vx = -joystick.getLeftY() * MaxSpeed;
          double vy = -joystick.getLeftX() * MaxSpeed;

          double omegaCmd = vision.calcAimOmegaRadPerSec();
          // clamp
          omegaCmd = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omegaCmd));

          return teleopFieldCentric
              .withVelocityX(vx)
              .withVelocityY(vy)
              .withRotationalRate(omegaCmd);
        })
    );

    
      // Hold = prepare shot (spin + aim hood), release = idle (hood returns to 0)
    joystick.rightBumper()
      .whileTrue(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT)))
      .whileFalse(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE)));

      // Hold = shoot (feeder enabled when ready), release = prepare (or idle, your choice)
    joystick.rightTrigger()
      .whileTrue(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.SHOOTING)))
      .whileFalse(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE)));


      
      // Nudge hood UP
    joystick.povUp()
     .whileTrue(Commands.run(() -> shooter.adjustHoodManual(+0.01)));

      // Nudge hood DOWN
    joystick.povDown()
      .whileTrue(Commands.run(() -> shooter.adjustHoodManual(-0.01)));      

      // Intake roller: hold left trigger to intake, release = idle
    joystick.leftTrigger()
      .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE)))
      .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

      // Pivot: A = deploy, B = stow
    joystick.a().whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOYED)));
    joystick.b().whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.STOWED)));

  
   


    RobotModeTriggers.autonomous().onTrue(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero))
    );
  }


  public void periodic() {
    vision.updateVisionPose(drivetrain);
  }

 public Command getAutonomousCommand() {
  return new PathPlannerAuto("New Auto");
}
}