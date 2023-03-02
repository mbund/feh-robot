/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <LCDColors.h>

#include <cmath>

#include "util.h"

/// Translates the robot for a given duration at a given heading
class TranslateStep : public Step {
   public:
    /// Translate (move) the robot for a given duration at a given heading
    /// @param name The name of the step
    /// @param distance The distance of the translation in inches
    /// @param heading The heading (angle) to translate towards in radians
    TranslateStep(std::string name,
                  double distance,
                  double heading,
                  double power);

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

bool TranslateStep::execute(double t) {
    if (std::abs(m1.get_distance() * m1_ratio) +
            std::abs(m2.get_distance() * m2_ratio) +
            std::abs(m3.get_distance() * m3_ratio) >=
        distance) {
        LOG_INFO("moved " << distance << "in " << heading << "rad");
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

class CDSWaitStep : public Step {
   public:
    CDSWaitStep(std::string name);

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

double deg_to_rad(double deg) { return deg * TAU / 360.0; }

/// Main function which is the entrypoint for the entire program
int main() {
    LOG_INFO("starting");
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    Timeline timeline{
        CDSWaitStep("Wait for light"),
        RotateStep("rotate 360deg 40%", deg_to_rad(360), 0.40),
        TranslateStep("up 24in 40%", 24, deg_to_rad(90), 0.40),
        EndStep(),
    };

    Navbar navbar;
    Rect working_area = Rect(0,
                             navbar.bounding_box.height + 1,
                             LCD_WIDTH,
                             LCD_HEIGHT - navbar.bounding_box.height);
    TimelineUI timeline_ui(working_area, timeline, navbar);
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
        timeline.timestep(dt);

        // update all UI elements
        navbar.update();
        timeline_ui.update();
        log_ui.update();
        misc_ui.update();
    }
}
