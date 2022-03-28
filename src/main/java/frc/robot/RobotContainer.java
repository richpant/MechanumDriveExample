// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutonomousOne;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;


import edu.wpi.first.wpilibj.Timer;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final Intake intake = new Intake();
  private final Shooter shooter;
  private final Transfer transfer;
  public final Timer timer = new Timer();

  
  SendableChooser<Command> chooser = new SendableChooser<>();
  // The driver's controller
  XboxController m_driverController0 = new XboxController(OIConstants.kDriverControllerPort0);
  XboxController m_driverController1 = new XboxController(OIConstants.kDriverControllerPort1);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Shooter shooter0, Transfer transfer0, DriveSubsystem m_robotDrive0) {
    shooter = shooter0;
    transfer = transfer0;
    m_robotDrive = m_robotDrive0;
    AutonomousOne Auto1Sec;
    Auto1Sec = new AutonomousOne(m_robotDrive, shooter);
    chooser.setDefaultOption("Auto 1", Auto1Sec);
    SmartDashboard.putData("Auto 1", chooser);
    // Configure the button bindings
    configureButtonBindings();
    // ------------------INTAKE--------------------
    JoystickButton runIntake = new JoystickButton(m_driverController1, XboxController.Button.kStart.value);
    runIntake.whileHeld(() -> intake.intakeBall(Constants.INTAKE_SPEED)).whenReleased(() -> intake.intakeBall(0));
    JoystickButton runIntakeReverse = new JoystickButton(m_driverController1, XboxController.Button.kLeftStick.value);
    runIntakeReverse.whileHeld(() -> intake.intakeBall(-Constants.INTAKE_SPEED))
        .whenReleased(() -> intake.intakeBall(0));
    // -----------------TRANSFER--------------------
    JoystickButton transferButton = new JoystickButton(m_driverController1, XboxController.Button.kBack.value);
    transferButton.whileHeld(() -> transfer.transferBall(Constants.TRANSFER_SPEED))
        .whenReleased(() -> transfer.transferBall(0));
    JoystickButton transferButtonReverse = new JoystickButton(m_driverController1,
        XboxController.Button.kRightBumper.value);
    transferButtonReverse.whileHeld(() -> transfer.transferBall(-.3 * Constants.TRANSFER_SPEED))
        .whenReleased(() -> transfer.transferBall(0));
    // ------------------SHOOTER-------------------
    JoystickButton shootButton = new JoystickButton(m_driverController1, XboxController.Button.kLeftBumper.value);
    shootButton.whileHeld(() -> shooter.shootBall(Constants.SHOOTER_SPEED)).whenReleased(() -> shooter.shootBall(0));
    JoystickButton shootButton2 = new JoystickButton(m_driverController1, XboxController.Button.kA.value);
    shootButton2.whileHeld(() -> shooter.shootBall(Constants.SHOOTER_SPEED)).whenReleased(() -> shooter.shootBall(0));
    JoystickButton shootButtonReverse = new JoystickButton(m_driverController1, XboxController.Button.kY.value);
    shootButtonReverse.whileHeld(() -> shooter.shootBall(-.5 * Constants.SHOOTER_SPEED))
        .whenReleased(() -> shooter.shootBall(0));
    // ----------------INTAKEBALL-------------------
    JoystickButton IntakeBall = new JoystickButton(m_driverController2, XboxController.Button.kRightBumper.value);
    IntakeBall.whileHeld(() -> intake.intakeBall(Constants.INTAKE_SPEED)).whenReleased(() -> intake.intakeBall(0));
    IntakeBall.whileHeld(() -> transfer.transferBall(Constants.TRANSFER_SPEED))
        .whenReleased(() -> transfer.transferBall(0));
    IntakeBall.whileHeld(() -> shooter.shootBall(-.5 * Constants.SHOOTER_SPEED))
        .whenReleased(() -> shooter.shootBall(0));

    m_robotDrive.setMaxOutput(.67);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () -> m_robotDrive.drive(
                -m_driverController0.getRightX(), // rightX
                -m_driverController0.getLeftY(), // leftY
                m_driverController0.getLeftX(), // leftX
                false), // doesn't change anything if true

            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController0, Button.kX.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5));
    // .whenReleased(() -> m_robotDrive.setMaxOutput(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-3, 0, new Rotation2d(0)),
        config);

    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kFeedforward,
        DriveConstants.kDriveKinematics,

        // Position contollers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

        // Needed for normalizing wheel speeds
        AutoConstants.kMaxSpeedMetersPerSecond,

        // Velocity PID's
        new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
        new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
        new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
        new PIDController(DriveConstants.kPRearRightVel, 0, 0),
        m_robotDrive::getCurrentWheelSpeeds,
        m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
       
        
  }

 
}
