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

import edu.wpi.first.wpilibj.Notifier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import com.analog.adis16448.frc.ADIS16448_IMU;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Robot extends TimedRobot {

  // -----------------CONSTANTS ------------------
  // number of encoder counts per wheel revolution
  private static final int TICKS_PER_REV = 4096;
  // diameter of the wheels
  private static final double WHEEL_DIAMETER = 4.0 / 12.0;
  // maximum velocity of the robot
  private static final double MAX_VELOCITY = 10;
  // port numbers for the left and right speed controllers
  private static final int LEFT_TALON_ID = 0;
  private static final int RIGHT_TALON_ID = 1;
  
  // Default Talon and encoder constants:
  private static final int PID_LOOP_IDX = 0;
  public static final int SLOT_IDX = 0;
  public static final int TIMEOUT_MS = 30;
  public static boolean SENSOR_PHASE = false;
  public static boolean MOTOR_INVERT = false;

  // ------------- Class Members -------------

  // Speed controllers for left and right side
  private TalonSRX m_leftTalon;
  private TalonSRX m_rightTalon;
  // Gyro
  private ADIS16448_IMU m_gryo;
  // Encoder followers for left and right side
  private EncoderFollower m_leftFollower;
  private EncoderFollower m_rightFollower;
  // Nofifier - this is what keeps the robot on the path
  private Notifier m_followerNotifier;

  // Trajectory files to choose from
  private static final String TRAJECTORY_1 = "Straight 2 feet";
  private static final String TRAJECTORY_2 = "Simple Curve";
  private String m_trajectorySelected;
  private final SendableChooser<String> m_trajectoryChooser = new SendableChooser<>();

  @Override
  public void robotInit() {

    m_trajectoryChooser.setDefaultOption(TRAJECTORY_1, TRAJECTORY_1);
    m_trajectoryChooser.addOption(TRAJECTORY_2, TRAJECTORY_2);
    SmartDashboard.putData("Trajectory choices", m_trajectoryChooser);

    // Set up the speed controllers
    m_leftTalon = new TalonSRX(LEFT_TALON_ID);
    configureTalon(m_leftTalon);
    m_rightTalon = new TalonSRX(RIGHT_TALON_ID);
    configureTalon(m_rightTalon);

    // Set up the gyro
    m_gryo = new ADIS16448_IMU();
    m_gryo.reset();
    m_gryo.calibrate();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

    // Get the trajectory filename from SmartDashboard
    m_trajectorySelected = m_trajectoryChooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Trajectory selected: " + m_trajectorySelected);

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
      // Note: PathWeaver V2019.3.1 has a known issue. The left and right paths are being swapped. 
      //TODO Change the following line to "left" once the PathWeaver bug is fixed
      left_trajectory  = PathfinderFRC.getTrajectory(m_trajectorySelected + ".right");
    }
    catch (IOException e) { 
      System.out.println("Cannot find left trajectory for trajectory: " + m_trajectorySelected);
      System.exit(1);
    }
    try {
      // Note: PathWeaver V2019.3.1 has a known issue. The left and right paths are being swapped. 
      //TODO Change the following line to "right" once the PathWeaver bug is fixed
      right_trajectory = PathfinderFRC.getTrajectory(m_trajectorySelected + ".left");
    }
    catch (IOException e) { 
      System.out.println("Cannot find right trajectory for trajectory: " + m_trajectorySelected);
      System.exit(1);
    }

    /*
      2.  Create encoder followers from the left and right trajectories. The encoder followers compute
          the motor values based on where the robot is in the path.
    */
    m_leftFollower = new EncoderFollower(left_trajectory);
    m_rightFollower = new EncoderFollower(right_trajectory);

    /*
      3.  Configure the encoders used by the followers with the number of counts per wheel revolution
          and diameter and PID constants to tune how fast the follower reacts to changes in velocity.
    */
    m_leftFollower.configureEncoder(getEncoderPosition(m_leftTalon), TICKS_PER_REV, WHEEL_DIAMETER);
    // You must tune the PID values (kp, ki, kd, kv) on the following line!
    m_leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY, 0);
    
    m_rightFollower.configureEncoder(getEncoderPosition(m_rightTalon), TICKS_PER_REV, WHEEL_DIAMETER);
    // You must tune the PID values (kp, ki, kd, kv) on the following line!
    m_rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY, 0);

    /*
      4.  Create the notifier that will regularly call the followPath() method that computes the motor
          speeds and send them to the motors.
    */
    m_followerNotifier = new Notifier(this::followPath);
    m_followerNotifier.startPeriodic(left_trajectory.get(0).dt);
  }

  /*
    Notifier method that actually drives the motors
    Each delta time (value programmed into the notifier in the previous code segment) gets the current
    wheel speeds for the left and the right side. Use the predicted heading at each point and the actual
    robot heading from the gyro sensor. The difference between the actual and predicted heading is
    the heading error that is factored into the motor speed setting to help ensure the robot tracks the
    path direction.
  */
  private void followPath() {
    if (m_leftFollower.isFinished() || m_rightFollower.isFinished()) {
      m_followerNotifier.stop();
    } else {
      int leftEncoderPosition = getEncoderPosition(m_leftTalon);
      int rightEncoderPosition = getEncoderPosition(m_rightTalon);

      double left_speed = m_leftFollower.calculate(leftEncoderPosition);
      double right_speed = m_rightFollower.calculate(rightEncoderPosition);

      //TODO Make sure that this is the proper angle to retrieve from the IMU gyro
      double heading = m_gryo.getAngleZ();
      double desired_heading = Pathfinder.r2d(m_leftFollower.getHeading());
      // Note: PathWeaver V2019.3.1 has a known issue. The left and right paths are being swapped. 
      // This bug potentially affects the desired_heading depending on the orientation of the gyro.
      // If so, you may also need to invert the desired heading with the following fix:
      //     double desired_heading = -Pathfinder.r2d(m_left_follower.getHeading());

      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn = 0.8 * (-1.0/80.0) * heading_difference;

      m_leftTalon.set(ControlMode.PercentOutput, left_speed + turn);
      m_rightTalon.set(ControlMode.PercentOutput, right_speed - turn);
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  /*
    Stop the motors at the start of the Teleop period
    After the autonomous period ends and the teleop period begins, be sure to stop the notifier from
    running the followPath() method (above) and stop the motors in case they were still running.
  */
  @Override
  public void teleopInit() {
    m_followerNotifier.stop();
    m_leftTalon.set(ControlMode.PercentOutput, 0);
    m_rightTalon.set(ControlMode.PercentOutput, 0);
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

  // Configure the talons and associated encoders
  private void configureTalon(TalonSRX talon){
		// Choose the sensor and sensor direction
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_LOOP_IDX, TIMEOUT_MS);

		// Set appropriately to ensure sensor is positive when output is positive
		talon.setSensorPhase(SENSOR_PHASE);

		// Set based on what direction you want forward/positive to be.
		// This does not affect sensor phase.
		talon.setInverted(MOTOR_INVERT);

		// Set the peak and nominal outputs, 12V means full
		talon.configNominalOutputForward(0, TIMEOUT_MS);
		talon.configNominalOutputReverse(0, TIMEOUT_MS);
		talon.configPeakOutputForward(1, TIMEOUT_MS);
		talon.configPeakOutputReverse(-1, TIMEOUT_MS);
		
		// Lets grab the 360 degree position of the MagEncoder's absolute
		// position, and initially set the relative sensor to match.
		int absolutePosition = talon.getSensorCollection().getPulseWidthPosition();
		// Mask out overflows, keep bottom 12 bits
		absolutePosition &= 0xFFF;
		if (SENSOR_PHASE)	absolutePosition *= -1;
		if (MOTOR_INVERT)	absolutePosition *= -1;
		
		// Set the quadrature (relative) sensor to match absolute
		talon.setSelectedSensorPosition(absolutePosition, PID_LOOP_IDX, TIMEOUT_MS);
  }

  // Get current position of the encoder on the associated talon
  private int getEncoderPosition(TalonSRX talon) {
    //TODO Make sure that this is the proper method to get the current encoder position in ticks
    int ticks = talon.getSensorCollection().getQuadraturePosition();
    return ticks;
  }
}