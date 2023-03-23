/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHUtility.h>
#include <LCDColors.h>
#include <sstream>
#include "FEHServo.h"

// allow us to access private members of FEHSD and initialize it
// ourselves, so that we can clear the screen after it forcefully
// writes to it. This is really janky and I wish that this did not
// have to happen.
#define private public
#include <FEHSD.h>
#undef private

#include <cmath>
#include <memory>
#include <vector>

#include "util.h"

Servo s1(FEHServo::Servo0, 500, 2500);

/// Translates the robot for a given distance at a given heading
class TranslateStep : public Step {
   public:
    /// Translate (move) the robot for a given distance at a given
    /// heading
    /// @param name The name of the step
    /// @param distance The distance of the translation in inches
    /// @param heading The heading (angle) to translate towards in
    /// radians
    TranslateStep(std::string name,
                  double distance,
                  double heading,
                  double power);

    /// Translate (move) the robot for a given distance at a given
    /// heading. The name of the step will be automatically generated.
    /// @param distance The distance of the translation in inches
    /// @param heading The heading (angle) to translate towards in
    /// radians
    /// @param power The power to drive the motors at
    TranslateStep(double distance, double heading, double power);

    /// Execute the translation step
    bool execute(double t) override;

   private:
    double heading;
    double distance;
    double power;

    double x;
    double y;
    double m1_ratio;
    double m2_ratio;
    double m3_ratio;
};

TranslateStep::TranslateStep(std::string name,
                             double distance,
                             double heading,
                             double power)
    : Step(name), distance(distance), heading(heading), power(power) {
    x = std::cos(heading);
    y = std::sin(heading);
    m1_ratio = (2.0 / 3.0) * x;
    m2_ratio = -(1.0 / 3.0) * x - (1.0 / std::sqrt(3.0)) * y;
    m3_ratio = -(1.0 / 3.0) * x + (1.0 / std::sqrt(3.0)) * y;
}

TranslateStep::TranslateStep(double distance,
                             double heading,
                             double power)
    : TranslateStep("Translate", distance, heading, power) {
    std::stringstream ss;
    ss.precision(2);
    ss << "T " << distance << "in " << heading << "rad "
       << (power * 100.) << "%";
    this->name = ss.str();
}

bool TranslateStep::execute(double t) {
    const auto momentum_accounting = power;
    if (std::abs(m1.get_distance() * m1_ratio) +
            std::abs(m2.get_distance() * m2_ratio) +
            std::abs(m3.get_distance() * m3_ratio) >=
        distance - momentum_accounting) {
        LOG_INFO("moved " << distance << "in " << heading << "rad");
        return true;
    }

    m1.drive(power * m1_ratio);
    m2.drive(power * m2_ratio);
    m3.drive(power * m3_ratio);

    return false;
}

/// Translates the robot for a given duration at a given heading
class TranslateTimeStep : public Step {
   public:
    /// Translate (move) the robot for a given duration at a given
    /// heading
    /// @param name The name of the step
    /// @param duration The duration (time) of the translation in
    /// seconds
    /// @param heading The heading (angle) to translate towards in
    /// radians
    TranslateTimeStep(std::string name,
                      double duration,
                      double heading,
                      double power);

    /// Execute the translation step
    bool execute(double t) override;

   private:
    double heading;
    double duration;
    double power;

    double x;
    double y;
    double m1_ratio;
    double m2_ratio;
    double m3_ratio;
};

TranslateTimeStep::TranslateTimeStep(std::string name,
                                     double duration,
                                     double heading,
                                     double power)
    : Step(name), duration(duration), heading(heading), power(power) {
    x = std::cos(heading);
    y = std::sin(heading);
    m1_ratio = (2.0 / 3.0) * x;
    m2_ratio = -(1.0 / 3.0) * x - (1.0 / std::sqrt(3.0)) * y;
    m3_ratio = -(1.0 / 3.0) * x + (1.0 / std::sqrt(3.0)) * y;
}

bool TranslateTimeStep::execute(double t) {
    if (t >= t_start + duration) {
        LOG_INFO("moved " << duration << "s " << heading << "rad");
        return true;
    }

    m1.drive(power * m1_ratio);
    m2.drive(power * m2_ratio);
    m3.drive(power * m3_ratio);

    return false;
}

/// Ends the program
class EndStep : public Step {
   public:
    /// Constructor to make a step that ends the program
    EndStep();

    /// Execute the end step
    bool execute(double t) override;
};

EndStep::EndStep() : Step("End") {}

bool EndStep::execute(double t) {
    LOG_INFO("end");

    return true;
}

/// Waits for the cds to be below a certain value
class CDSWaitStep : public Step {
   public:
    /// Constructor to make a step that waits for the cds to be below
    /// a certain
    /// @param name The name of the step
    CDSWaitStep(std::string name);

    /// Execute the cds wait step
    bool execute(double t) override;
};

CDSWaitStep::CDSWaitStep(std::string name) : Step(name) {}

bool CDSWaitStep::execute(double t) {
    constexpr auto RED_VALUE = 0.3;

    const auto val = cds.Value();
    if (val < RED_VALUE) {
        LOG_INFO("cds end " << val << " < " << RED_VALUE);
        return true;
    }

    return false;
}

/// Rotate the robot for a given duration by a given angle
class RotateStep : public Step {
   public:
    /// Rotate the robot for a given duration by a given angle
    /// @param name The name of the step
    /// @param theta The angle to rotate by
    RotateStep(std::string name, double theta, double power);

    /// Execute the rotation step
    bool execute(double t) override;

   private:
    double theta;
    double power;

    double distance;
};

RotateStep::RotateStep(std::string name, double theta, double power)
    : Step(name), theta(theta), power(power) {
    distance = ROBOT_CENTER_TO_WHEEL_DISTANCE * theta;
}

bool RotateStep::execute(double t) {
    if (m1.get_distance() + m2.get_distance() + m3.get_distance() >=
        distance * 3 * (std::sqrt(3.0) / 2.0)) {
        LOG_INFO("rotated " << theta << "rad");
        return true;
    }

    m1.drive(power);
    m2.drive(power);
    m3.drive(power);

    return false;
}

/// Sleeps for a given duration
class SleepStep : public Step {
   public:
    /// Sleeps for a given duration
    /// @param name The name of the step
    /// @param duration The duration of the rotation
    SleepStep(std::string name, double duration);

    /// Execute the rotation step
    bool execute(double t) override;

   private:
    double duration;
};

SleepStep::SleepStep(std::string name, double duration)
    : Step(name), duration(duration) {}

bool SleepStep::execute(double t) {
    if (t >= t_start + duration) {
        LOG_INFO("slept " << duration << "s");
    }

    return t >= t_start + duration;
}

class TicketKioskStep : public Step {
   public:
    TicketKioskStep(std::string name);

    bool execute(double t) override;
};

TicketKioskStep::TicketKioskStep(std::string name) : Step(name) {}

bool TicketKioskStep::execute(double t) {
    constexpr auto RED_VALUE = 0.5;
    constexpr auto BLUE_VALUE = 1.2;

    const auto val = cds.Value();

    if (val < RED_VALUE) {
        timeline->add_ephemeral_steps(
            TranslateStep("red right", 11, deg_to_rad(0), 0.60),
            RotateStep("red rotate", deg_to_rad(180), 0.30),
            TranslateTimeStep(
                "red forward", 1.5, deg_to_rad(90), -0.30),
            TranslateStep("red back", 3, deg_to_rad(90), 0.30),
            RotateStep("red rotate", deg_to_rad(180), -0.30),
            TranslateStep("red left", 5, deg_to_rad(180), 0.60));
        LOG_INFO("detected red " << val);
        return true;
    }

    if (val > RED_VALUE && val < BLUE_VALUE) {
        timeline->add_ephemeral_steps(
            TranslateStep("blue right", 5, deg_to_rad(0), 0.60),
            RotateStep("blue rotate", deg_to_rad(180), 0.30),
            TranslateTimeStep(
                "blue forward", 1.5, deg_to_rad(90), -0.30),
            TranslateStep("blue back", 3, deg_to_rad(90), 0.30),
            RotateStep("blue rotate", deg_to_rad(180), -0.30));
        LOG_INFO("detected blue " << val);
        return true;
    }

    return false;
}

class ServoStep : public Step {
   public:
    ServoStep(std::string name, double theta);

    void start() override;

   private:
    double theta;
};

ServoStep::ServoStep(std::string name, double theta)
    : Step(name), theta(theta) {}

void ServoStep::start() { s1.set_angle(theta); }

class FuelLeverStep : public Step {
   public:
    FuelLeverStep(std::string name);

    bool execute(double t) override;
};

FuelLeverStep::FuelLeverStep(std::string name) : Step(name) {}

constexpr auto LEVER_LEFT = 0;
constexpr auto LEVER_MIDDLE = 1;
constexpr auto LEVER_RIGHT = 2;

bool FuelLeverStep::execute(double t) {
    const auto lever = RPS.GetCorrectLever();
    LOG_INFO("got rps lever " << lever);

    if (lever == LEVER_LEFT) {
        timeline->add_ephemeral_steps(
            //
            RotateStep("rotate", deg_to_rad(190), 0.30),
            ServoStep("down", deg_to_rad(180)),
            SleepStep("sleep", 1),
            TranslateStep(3, deg_to_rad(270), 0.60),
            TranslateStep(3, deg_to_rad(90), 0.60),
            SleepStep("sleep", 3),
            ServoStep("up", deg_to_rad(90)),
            TranslateStep(3, deg_to_rad(270), 0.60)
            //
        );
    }

    if (lever == LEVER_MIDDLE) {
        timeline->add_ephemeral_steps(
            //
            TranslateStep(3.5, deg_to_rad(180), 0.60),
            RotateStep("rotate", deg_to_rad(190), 0.30),
            ServoStep("down", deg_to_rad(180)),
            SleepStep("sleep", 1),
            TranslateStep(3, deg_to_rad(270), 0.60),
            TranslateStep(3, deg_to_rad(90), 0.60),
            SleepStep("sleep", 3),
            ServoStep("up", deg_to_rad(90)),
            TranslateStep(3, deg_to_rad(270), 0.60)
            //
        );
    }

    if (lever == LEVER_RIGHT) {
        timeline->add_ephemeral_steps(
            //
            TranslateStep(3.5, deg_to_rad(180), 0.60),
            RotateStep("rotate", deg_to_rad(225), 0.30),
            ServoStep("down", deg_to_rad(180)),
            SleepStep("sleep", 1),
            TranslateStep(3, deg_to_rad(270), 0.60),
            TranslateStep(3, deg_to_rad(90), 0.60),
            SleepStep("sleep", 3),
            ServoStep("up", deg_to_rad(90)),
            TranslateStep(3, deg_to_rad(270), 0.60)
            //
        );
    }

    return true;
}

/// Main function which is the entrypoint for the entire program
int main() {
    s1.set_angle(deg_to_rad(90));
    SD.Initialize();
    LOG_INFO("starting");
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    timeline = std::make_shared<Timeline>(
        // begin
        CDSWaitStep("Wait for light"),

        TranslateStep(11, deg_to_rad(90), 0.60),
        TranslateStep(19, deg_to_rad(180), 0.60),
        FuelLeverStep("Fuel lever"),

        EndStep()
        // end
    );

    Navbar navbar;
    Rect working_area = Rect(0,
                             navbar.bounding_box.height + 1,
                             LCD_WIDTH,
                             LCD_HEIGHT - navbar.bounding_box.height);
    TimelineUI timeline_ui(working_area, navbar);
    LogUI log_ui(working_area, navbar);
    MiscUI misc_ui(working_area, navbar);
    navbar.add_button("Timeline", [&]() { timeline_ui.render(); });
    navbar.add_button("Logs", [&]() { log_ui.render(); });
    navbar.add_button("Misc", [&]() { misc_ui.render(); });

    // Main loop
    auto t = 0.0;
    auto current_time = TimeNow();

    while (true) {
        auto new_time = TimeNow();
        auto dt = new_time - t;
        current_time = new_time;
        t += dt;

        touch_pressed = LCD.Touch(&touch_x, &touch_y);

        // advance the timeline
        timeline->timestep(dt);

        // update all UI elements
        navbar.update();
        timeline_ui.update();
        log_ui.update();
        misc_ui.update();
    }
}
