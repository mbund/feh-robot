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
    /// @param heading The heading (angle) to translate towards
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

    double m1_dist = 0;
    double m2_dist = 0;
    double m3_dist = 0;
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
    if (m1_dist >= distance &&
        /* m2_dist >= distance && */ m3_dist >= distance) {
        LOG_INFO("translate step done");
        return true;
    }

    m1_dist += m1.get_distance();
    m2_dist += m2.get_distance();
    m3_dist += m3.get_distance();

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
    return cds.Value() < RED_VALUE;
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

    double m1_dist = 0;
    double m2_dist = 0;
    double m3_dist = 0;
};

RotateStep::RotateStep(std::string name, double theta, double power)
    : Step(name), theta(theta), power(power) {
    distance = ROBOT_CENTER_TO_WHEEL_DISTANCE * theta;
    LOG_INFO("Rotating " << distance << " in");
}

bool RotateStep::execute(double t) {
    if (m1_dist + m2_dist + m3_dist >= distance) {
        LOG_INFO("rotate step done");
        return true;
    }

    if ((int)std::round(t) % 2 == 0) {
        LOG_INFO("dist " << distance);
        LOG_INFO("m1 " << m1_dist);
        LOG_INFO("m2 " << m2_dist);
        LOG_INFO("m3 " << m3_dist);
    }

    m1.drive(power);
    m1_dist += m1.get_distance();

    m2.drive(power);
    m2_dist += m2.get_distance();

    m3.drive(power);
    m3_dist += m3.get_distance();

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
        LOG_INFO("sleep step done");
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
        RotateStep("Rotate", deg_to_rad(90), 0.2),
        CDSWaitStep("Wait for light"),
        TranslateStep("Initial move", 12, deg_to_rad(0), 0.5),
        SleepStep("Sleep", 1),
        TranslateStep("Move back", 4, deg_to_rad(180), 0.5),
        TranslateStep("0", 4, deg_to_rad(0), 0.5),
        TranslateStep("45", 4, deg_to_rad(45), 0.5),
        TranslateStep("90", 4, deg_to_rad(90), 0.5),
        TranslateStep("135", 4, deg_to_rad(135), 0.5),
        TranslateStep("180", 4, deg_to_rad(180), 0.5),
        TranslateStep("225", 4, deg_to_rad(225), 0.5),
        TranslateStep("270", 4, deg_to_rad(270), 0.5),
        TranslateStep("315", 4, deg_to_rad(315), 0.5),
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

        // flush all motors
        m1.flush();
        m2.flush();
        m3.flush();
    }
}
