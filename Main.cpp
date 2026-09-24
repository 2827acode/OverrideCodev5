#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <math.h>

// SETS UP MOVEMENT AND MOTOR/MOTORGROUPS, PNEUMATICS, AND AUTON MOVEMENT
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motor Groups & Motors
pros::MotorGroup left_mg({18, -14, -19}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({-15, 17, 16}, pros::MotorGearset::blue);
pros::MotorGroup IntakeAndOuttake({-1}, pros::MotorGearset::red);
pros::MotorGroup BottomGoalOutake({1}, pros::MotorGearset::red);
pros::MotorGroup DoubleBar({10}, pros::MotorGearset::green);
pros::Motor Cascade(16, pros::MotorGearset::red);
pros::Motor ClawArm(9, pros::MotorGearset::green);

// Sensors & Pneumatics
pros::Imu imu(19);

bool claw = false;
bool macro_started = false;

pros::ADIDigitalOut air('H', claw);

lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.4, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu// inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              127 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}


// Start of functions
// Start of functions
// Start of functions


// ---------------- Claw functions ----------------

// Motor degrees per mechanism degree. TUNE THESE to your real gearing.
constexpr double CLAW_TICKS_PER_DEG      = 1.35 * 2.5;
constexpr double DOUBLEBAR_TICKS_PER_DEG = 1.35 * 2.5;

void moveClawToAngle(double angle, int velocity = 200) {
    ClawArm.move_absolute(angle * CLAW_TICKS_PER_DEG, velocity);
}

bool clawAtAngle(double angle, double gearRatio = 1.0, double tolerance = 2.0) {
    return std::fabs(ClawArm.get_position() - (angle * gearRatio)) <= tolerance * gearRatio;
}

// Claw Stall Code
// State flags for the stall macro
volatile bool clawStallActive = false;
volatile bool clawStallCancel = false;

/*
 * Runs the claw until it stalls, then brakes it.
 * speed, currentlimit, velocityLimit, and timeout are all optional parameters with default values.
*/
void moveClawUntilStalled(int speed = 100, int currentLimit = 1500,
                          int velocityLimit = 10, int timeout = 3000) {
    if (clawStallActive) return; // already running, ignore duplicate calls

    clawStallActive = true;
    clawStallCancel = false;

    pros::Task([=]() {
        const uint32_t graceMs = 200;        // ignores first friction spike
        const uint32_t stallConfirmMs = 50; // prevents instant stall detection from friction spikes
        uint32_t start = pros::millis();
        uint32_t stallStart = 0;

        ClawArm.move_velocity(speed);

        while (!clawStallCancel && (pros::millis() - start) < (uint32_t)timeout) {
            uint32_t now = pros::millis();
            bool stalled = ClawArm.get_current_draw() > currentLimit && std::fabs(ClawArm.get_actual_velocity()) < velocityLimit;

            if ((now - start) > graceMs && stalled) {
                if (stallStart == 0) stallStart = now;
                else if (now - stallStart >= stallConfirmMs) break; // confirmed stall
            } else {
                stallStart = 0; // no longer stalled, reset the timer
            }
            pros::delay(10);
        }

        ClawArm.brake(); // brakes using the HOLD brake mode set in initialize()
        clawStallActive = false;
        ClawArm.tare_position();
    });
}


// ---------------- DoubleBar functions ----------------

volatile bool doubleBarStallActive = false;
volatile bool doubleBarStallCancel = false;

void moveDoubleBartoangle(double angle, double gearRatio = 5, double anglefix = 2.5, int velocity = 100) {
    DoubleBar.move_absolute(angle * gearRatio * anglefix, velocity);
}

void moveDoubleBarUntilStalled(int speed = 100, int currentLimit = 2000,
                          int velocityLimit = 10, int timeout = 3000) {
    if (doubleBarStallActive) return; // fixes button mashing issue

    doubleBarStallActive = true;
    doubleBarStallCancel = false;

    pros::Task([=]() {
        const uint32_t graceMs = 200;        // ignores first friction spike
        const uint32_t stallConfirmMs = 100; // prevents instant stall detection from friction spikes
        uint32_t start = pros::millis();
        uint32_t stallStart = 0;

        DoubleBar.move_velocity(speed);

        while (!doubleBarStallCancel && (pros::millis() - start) < (uint32_t)timeout) {
            uint32_t now = pros::millis();
            bool stalled = DoubleBar.get_current_draw() > currentLimit &&
            std::fabs(DoubleBar.get_actual_velocity()) < velocityLimit;

            if ((now - start) > graceMs && stalled) {
                if (stallStart == 0) stallStart = now;
                else if (now - stallStart >= stallConfirmMs) break; // confirmed stall
            } else {
                stallStart = 0; // no longer stalled, reset the timer
            }
            pros::delay(10);
        }

        DoubleBar.brake(); // brakes using the HOLD brake mode set in initialize()
        doubleBarStallActive = false;
    });
}


//VFB

// TUNING KNOBS
// adjust the magnitude to fine-tune.
double VFB_RATIO = -.2;
// Position correction strength. Raise if the claw lags, lower if it buzzes.
double VFB_KP = 1.2;
// Feedforward scale. 1.0 = claw copies the arm's speed times the ratio.
double VFB_KFF = 1;
// How close (claw motor degrees) before it stops correcting after you release.
double VFB_TOLERANCE = 2.0;

volatile bool virtualFourBarActive = false;
volatile bool vfbSettling = false;          
volatile bool vfbHasReference = false;
double vfbClawOffset = 0.0;

void engageVirtualFourBar() {
    if (clawStallActive) return;
    if (!vfbHasReference) {
        // Set ONCE: claw position that matches the current arm position.
        vfbClawOffset = ClawArm.get_position() - DoubleBar.get_position() * VFB_RATIO;
        vfbHasReference = true;
    }
    virtualFourBarActive = true;
    vfbSettling = false;
}

void disengageVirtualFourBar() {
    if (!virtualFourBarActive) return;
    virtualFourBarActive = false;
    vfbSettling = true; // let the loop finish the move, then it brakes
}

// Call after anything else moves the claw or double bar (like the X macro),
// so the next engage captures a fresh reference.
void resetVirtualFourBarReference() {
    virtualFourBarActive = false;
    vfbSettling = false;
    vfbHasReference = false;
}

void virtualFourBarLoop() {
    const double maxVel = 200.0; // green cartridge max
    while (true) {
        if ((virtualFourBarActive || vfbSettling) && vfbHasReference && !clawStallActive) {
            double target = vfbClawOffset + DoubleBar.get_position() * VFB_RATIO;
            double error  = target - ClawArm.get_position();

            if (!virtualFourBarActive && std::fabs(error) < VFB_TOLERANCE) {
                ClawArm.brake();
                vfbSettling = false;
            } else {
                double feedforward = DoubleBar.get_actual_velocity() * VFB_RATIO * VFB_KFF;
                double cmd = feedforward + error * VFB_KP;
                ClawArm.move_velocity(std::clamp(cmd, -maxVel, maxVel));
            }
        }
        pros::delay(10);
    }
}
// End of functions
// End of functions
// End of functions

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    Cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DoubleBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ClawArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::Task vfbTask(virtualFourBarLoop);

    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y );
            pros::lcd::print(2, "Theta: %f",chassis.getPose().theta);
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}

void autonomous() {

}

void opcontrol() {
    chassis.cancelAllMotions();
    chassis.setPose(0, 0, 0);

    doubleBarStallActive = false;
    doubleBarStallCancel = false;
    clawStallActive = false;
    clawStallCancel = false;

    while (true) {
        // Driver Control
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        right_mg.move(dir + turn);
        left_mg.move(dir - turn);

        if (master.get_digital_new_press(DIGITAL_X)) {
            resetVirtualFourBarReference();
            pros::Task([]() {
                moveClawUntilStalled(75);
                moveDoubleBarUntilStalled(-200);
                while (clawStallActive) {
                    pros::delay(20);
                }
                moveClawToAngle(-90);
            });
        }

        if (master.get_digital(DIGITAL_Y)) {
            doubleBarStallCancel = true;
            doubleBarStallActive = false;
            engageVirtualFourBar();
            DoubleBar.move(35);
        }
        else if (master.get_digital(DIGITAL_RIGHT)) {
            doubleBarStallCancel = true;
            doubleBarStallActive = false;
            engageVirtualFourBar();
            DoubleBar.move(-35);
        }

        else if (master.get_digital(DIGITAL_L1)) {
            Cascade.move(-127);
        }

        else if (master.get_digital(DIGITAL_R1)) {
            Cascade.move(127);
        }

        else {
            disengageVirtualFourBar();
            if (!doubleBarStallActive) {
                DoubleBar.brake();
            }
            Cascade.brake();
        }

        pros::delay(20);
    }
}
