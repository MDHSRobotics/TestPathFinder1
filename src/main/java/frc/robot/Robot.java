// This is a test program to autonomously drive a robot along a pre-defined path.
// It is based on the 2019 documentation for "PATHWEAVER AND ROBOT PATH PLANNING" 
// which can be found at:
//    http://wpilib.screenstepslive.com/s/currentCS/m/84338
//
// Another useful reference is "Pathfinder for FRC Java" which can for found at:
//    https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

// Make sure you install the Pathfinder vendor library from:
//  http://dev.imjac.in/maven/jaci/pathfinder/PathfinderOLD-latest.json
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // number of encoder counts per wheel revolution
  private static final int k_ticks_per_rev = 1024;
  // diameter of the wheels
  private static final double k_wheel_diameter = 4.0 / 12.0;
  // maximum velocity of the robot
  private static final double k_max_velocity = 10;
  // port numbers for the left and right speed controllers
  private static final int k_left_channel = 0;
  private static final int k_right_channel = 1;
  // port numbers for the encoders connected to the left and right side of the drivetrain
  private static final int k_left_encoder_port_a = 0;
  private static final int k_left_encoder_port_b = 1;
  private static final int k_right_encoder_port_a = 2;
  private static final int k_right_encoder_port_b = 3;
  // analog input for the gyro (other gyros might be connected differently)
  private static final int k_gyro_port = 0;
  // name of this path
  private static final String k_path_name = "example";

  private SpeedController m_left_motor;
  private SpeedController m_right_motor;
  private Encoder m_left_encoder;
  private Encoder m_right_encoder;
  private AnalogGyro m_gyro;
  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  private Notifier m_follower_notifier;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_left_motor = new Spark(k_left_channel);
    m_right_motor = new Spark(k_right_channel);
    m_left_encoder = new Encoder(k_left_encoder_port_a, k_left_encoder_port_b);
    m_right_encoder = new Encoder(k_right_encoder_port_a, k_right_encoder_port_b);
    m_gyro = new AnalogGyro(k_gyro_port);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    /*
      1.  Create the trajectories for the left and right sides of the drivetrain. This will look for paths in the
          /home/lvuser/deploy/paths folder on the roboRIO. If you choose the output directory in PathWeaver 
          (as per the instructions in the PathWeaver documentation), PathWeaver will automatically place the
          paths in the proper folder. The full filenames for the paths are: 
            /home/lvuser/deploy/paths/PathName.left.pf1.csv
            /home/lvuser/deploy/paths/PathName.right.pf1.csv 
          for the left and right paths.
    */
    Trajectory left_trajectory = null;
    Trajectory right_trajectory = null;
    try {
      left_trajectory  = PathfinderFRC.getTrajectory(k_path_name + ".left");
    }
    catch (IOException e) { 
      System.out.println("Cannot find left trajectory");
    }
    try {
     right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
    }
    catch (IOException e) { 
      System.out.println("Cannot find right trajectory");
    }

    /*
      2.  Create encoder followers from the left and right trajectories. The encoder followers compute
          the motor values based on where the robot is in the path.
    */
    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    /*
      3.  Configure the encoders used by the followers with the number of counts per wheel revolution
          and diameter and PID constants to tune how fast the follower reacts to changes in velocity.
    */
    m_left_follower.configureEncoder(m_left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
    m_right_follower.configureEncoder(m_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    /*
      4.  Create the notifier that will regularly call the followPath() method that computes the motor
          speeds and send them to the motors.
    */
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
  }

  /*
    Notififier method that actually drives the motors
    Each delta time (value programmed into the notifier in the previous code segment) gets the current
    wheel speeds for the left and the right side. Use the predicted heading at each point and the actual
    robot heading from the gyro sensor. The difference between the actual and predicted heading is
    the heading error that is factored into the motor speed setting to help ensure the robot tracks the
    path direction.
  */
  private void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(m_left_encoder.get());
      double right_speed = m_right_follower.calculate(m_right_encoder.get());
      double heading = m_gyro.getAngle();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn = 0.8 * (-1.0/80.0) * heading_difference;
      m_left_motor.set(left_speed + turn);
      m_right_motor.set(right_speed - turn);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /*
    Stop the motors at the start of the Teleop period
    After the autonomous period ends and the teleop period begins, be sure to stop the notifier from
    running the followPath() method (above) and stop the motors in case they were still running.
  */
  @Override
  public void teleopInit() {
    m_follower_notifier.stop();
    m_left_motor.set(0);
    m_right_motor.set(0);
    }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
