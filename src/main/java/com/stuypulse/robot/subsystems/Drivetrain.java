/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Angle;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Drivetrain.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*-
 * Moves the robot around
 *
 * Contains:
 *      - 3 motors on left side
 *      - 3 motors on right side
 *      - Encoders for both sides
 *      - Gear shifting solonoid
 *      - NavX / Gyroscope
 */
public class Drivetrain extends SubsystemBase {

    // Enum used to store the state of the gear
    public static enum Gear {
        HIGH(Value.kReverse),
        LOW(Value.kForward);

        private final Value value;

        private Gear(Value value) {
            this.value = value;
        }
    }

    // An array of motors on the left and right side of the drive train
    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

    // DifferentialDrive and Gear Information
    private Gear gear;
    private final DoubleSolenoid gearShift;
    private final DifferentialDrive drivetrain;

    // An encoder for each side of the drive train
    private final Encoder leftGrayhill;
    private final Encoder rightGrayhill;

    // NAVX for Gyro
    private final AHRS navx;

    // Odometry
    private final DifferentialDriveOdometry odometry;
    private final Field2d field;

    public Drivetrain() {
        // Add Motors to list
        leftMotors =
                new CANSparkMax[] {
                    new CANSparkMax(Ports.Drivetrain.LEFT_TOP, MotorType.kBrushless),
                    new CANSparkMax(Ports.Drivetrain.LEFT_MIDDLE, MotorType.kBrushless),
                    new CANSparkMax(Ports.Drivetrain.LEFT_BOTTOM, MotorType.kBrushless)
                };

        rightMotors =
                new CANSparkMax[] {
                    new CANSparkMax(Ports.Drivetrain.RIGHT_TOP, MotorType.kBrushless),
                    new CANSparkMax(Ports.Drivetrain.RIGHT_MIDDLE, MotorType.kBrushless),
                    new CANSparkMax(Ports.Drivetrain.RIGHT_BOTTOM, MotorType.kBrushless)
                };

        // Make differential drive object
        drivetrain =
                new DifferentialDrive(
                        new MotorControllerGroup(leftMotors),
                        new MotorControllerGroup(rightMotors));

        // Add Gear Shifter
        gearShift =
                new DoubleSolenoid(
                        PneumaticsModuleType.CTREPCM,
                        Ports.Drivetrain.GEAR_SHIFT_FORWARD,
                        Ports.Drivetrain.GEAR_SHIFT_REVERSE);

        // Create Encoders
        leftGrayhill = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B);
        rightGrayhill = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);
        setGrayhillDistancePerPulse(Encoders.GRAYHILL_DISTANCE_PER_PULSE);

        // Initialize NAVX
        navx = new AHRS(SPI.Port.kMXP);

        // Initialize Odometry
        odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDistance(), getRightDistance());
        field = new Field2d();
        reset();

        // Configure Motors and Other Things
        setMotorConfig();
        setHighGear();
    }

    /***********************
     * MOTOR CONFIGURATION *
     ***********************/

    private void setMotorConfig() {
        leftGrayhill.setReverseDirection(
                Settings.Drivetrain.Encoders.GRAYHILL_INVERTED ^ Motors.Drivetrain.LEFT.INVERTED);
        for (CANSparkMax motor : leftMotors) {
            Motors.Drivetrain.LEFT.configure(motor);
        }

        rightGrayhill.setReverseDirection(
                Settings.Drivetrain.Encoders.GRAYHILL_INVERTED ^ Motors.Drivetrain.RIGHT.INVERTED);
        for (CANSparkMax motor : rightMotors) {
            Motors.Drivetrain.RIGHT.configure(motor);
        }
    }

    private void setGrayhillDistancePerPulse(double distance) {
        rightGrayhill.setDistancePerPulse(distance);
        rightGrayhill.reset();

        leftGrayhill.setDistancePerPulse(distance);
        leftGrayhill.reset();
    }

    /*****************
     * Gear Shifting *
     *****************/

    // Gets the current gear the robot is in
    public Gear getGear() {
        return gear;
    }

    // Sets the current gear the robot is in
    public void setGear(Gear gear) {
        gearShift.set(gear.value);
        this.gear = gear;
    }

    // Sets robot into low gear
    public void setLowGear() {
        setGear(Gear.LOW);
    }

    // Sets robot into high gear
    public void setHighGear() {
        setGear(Gear.HIGH);
    }

    /*********************
     * ENCODER FUNCTIONS *
     *********************/

    // Distance
    public double getLeftDistance() {
        return leftGrayhill.getDistance();
    }

    public double getRightDistance() {
        return rightGrayhill.getDistance();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    // Velocity
    public double getLeftVelocity() {
        return leftGrayhill.getRate();
    }

    public double getRightVelocity() {
        return rightGrayhill.getRate();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /***************
     * ROBOT ANGLE *
     ***************/

    // Gets current Angle of the Robot as a double (contiuous / not +-180)
    public double getRawGyroAngle() {
        return navx.getAngle();
    }

    // Gets current Angle of the Robot
    public Angle getGyroAngle() {
        return Angle.fromDegrees(getRawGyroAngle());
    }

    public Angle getAngle() {
        return getGyroAngle();
    }

    public Angle getRoll() {
        return Angle.fromDegrees(navx.getRoll());
    }

    /**********************
     * ODOMETRY FUNCTIONS *
     **********************/

    private void updateOdometry() {
        odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
    }

    public Rotation2d getRotation2d() {
        // TODO: check if this needs to be negative
        return getAngle().negative().getRotation2d();
    }

    public Pose2d getPose() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    public Field2d getField() {
        return field;
    }

    /************************
     * OVERALL SENSOR RESET *
     ************************/

    public void reset(Pose2d location) {
        navx.reset();
        leftGrayhill.reset();
        rightGrayhill.reset();

        odometry.resetPosition(getRotation2d(), getLeftDistance(), getRightDistance(), location);
    }

    public void reset() {
        reset(getPose());
    }

    /*********************
     * VOLTAGE FUNCTIONS *
     *********************/

    public double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public double getLeftVoltage() {
        return leftMotors[0].get() * getBatteryVoltage();
    }

    public double getRightVoltage() {
        return rightMotors[0].get() * getBatteryVoltage();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        for (MotorController motor : leftMotors) {
            motor.setVoltage(leftVolts);
        }

        for (MotorController motor : rightMotors) {
            motor.setVoltage(rightVolts);
        }

        drivetrain.feed();
    }

    /*******************
     * STALL DETECTION *
     *******************/

    // public double getLeftCurrentAmps() {
    //     double amps = 0.0;

    //     for (CANSparkMax motor : leftMotors) {
    //         amps += Math.abs(motor.getOutputCurrent());
    //     }

    //     return amps / leftMotors.length;
    // }

    // public double getRightCurrentAmps() {
    //     double amps = 0.0;

    //     for (CANSparkMax motor : rightMotors) {
    //         amps += Math.abs(motor.getOutputCurrent());
    //     }

    //     return amps / rightMotors.length;
    // }

    // public double getCurrentAmps() {
    //     return (getLeftCurrentAmps() + getRightCurrentAmps()) / 2.0;
    // }

    // public boolean isLeftStalling() {
    //     boolean highGear = getGear() == Gear.HIGH;
    //     boolean current = getLeftCurrentAmps() > Stalling.CURRENT_THRESHOLD;
    //     boolean output = Math.abs(leftMotors[0].get()) > Stalling.DUTY_CYCLE_THRESHOLD;
    //     boolean velocity = Math.abs(getLeftVelocity()) < Stalling.SCIBORGS_THRESHOLD;
    //     return highGear && (current || output) && velocity;
    // }

    // public boolean isRightStalling() {
    //     boolean highGear = getGear() == Gear.HIGH;
    //     boolean current = getRightCurrentAmps() > Stalling.CURRENT_THRESHOLD;
    //     boolean output = Math.abs(rightMotors[0].get()) > Stalling.DUTY_CYCLE_THRESHOLD;
    //     boolean velocity = Math.abs(getRightVelocity()) < Stalling.SCIBORGS_THRESHOLD;
    //     return highGear && (current || output) && velocity;
    // }

    // public boolean isStalling() {
    //     return isLeftStalling() || isRightStalling();
    // }

    /********************
     * DRIVING COMMANDS *
     ********************/

    // Stops drivetrain from moving
    public void stop() {
        drivetrain.stopMotor();
    }

    // Drives using tank drive
    public void tankDrive(double left, double right) {
        drivetrain.tankDrive(left, right, false);
    }

    // Drives using arcade drive
    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation, false);
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());

        SmartDashboard.putData("Field2d", field);

        SmartDashboard.putBoolean("Drivetrain/High Gear", getGear().equals(Gear.HIGH));
        SmartDashboard.putNumber("Drivetrain/Odometry X Pose (m)", getPose().getX());
        SmartDashboard.putNumber("Drivetrain/Odometry Y Pose (m)", getPose().getY());
        SmartDashboard.putNumber("Drivetrain/Odometry Rotation (deg)", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Drivetrain/Left Voltage (V)", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/Right Voltage (V)", getRightVoltage());

        SmartDashboard.putNumber("Drivetrain/Distance Travelled (m)", getDistance());
        SmartDashboard.putNumber("Drivetrain/Distance Travelled Left (m)", getLeftDistance());
        SmartDashboard.putNumber("Drivetrain/Distance Travelled Right (m)", getRightDistance());

        SmartDashboard.putNumber("Drivetrain/Velocity (m per s)", getVelocity());
        SmartDashboard.putNumber("Drivetrain/Velocity Left (m per s)", getLeftVelocity());
        SmartDashboard.putNumber("Drivetrain/Velocity Right (m per s)", getRightVelocity());
    }
}
