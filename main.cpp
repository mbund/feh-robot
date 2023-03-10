/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <LCDColors.h>

// allow us to access private members of FEHSD and initialize it ourselves, so
// that we can clear the screen after it forcefully writes to it. This is really
// janky and I wish that this did not have to happen.
#define private public
#include <FEHSD.h>
#undef private

#include <cmath>
#include <memory>
#include <vector>

#include "util.h"

/// Converts degrees to radians
/// @param deg The degrees to convert
/// @return The radian value of the degrees
double deg_to_rad(double deg) { return deg * TAU / 360.0; }

/// Translates the robot for a given distance at a given heading
class TranslateStep : public Step {
   public:
    /// Translate (move) the robot for a given distance at a given heading
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

/// Translates the robot for a given duration at a given heading
class TranslateTimeStep : public Step {
   public:
    /// Translate (move) the robot for a given duration at a given heading
    /// @param name The name of the step
    /// @param duration The duration (time) of the translation in seconds
    /// @param heading The heading (angle) to translate towards in radians
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
    /// Constructor to make a step that waits for the cds to be below a certain
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
    constexpr auto RED_VALUE = 0.8;
    constexpr auto BLUE_VALUE = 1.3;

    const auto val = cds.Value();

    if (val < RED_VALUE) {
        timeline->add_ephemeral_steps(
            TranslateStep("red right", 11, deg_to_rad(0), 0.60),
            RotateStep("red rotate", deg_to_rad(180), 0.30),
            TranslateTimeStep("red forward", 1.5, deg_to_rad(90), -0.30),
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
            TranslateTimeStep("blue forward", 1.5, deg_to_rad(90), -0.30),
            TranslateStep("blue back", 3, deg_to_rad(90), 0.30),
            RotateStep("blue rotate", deg_to_rad(180), -0.30));
        LOG_INFO("detected blue " << val);
        return true;
    }

    return false;
}

/// Main function which is the entrypoint for the entire program
int main() {
    SD.Initialize();
    LOG_INFO("starting");
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    timeline = std::make_shared<Timeline>(
        CDSWaitStep("Wait for light"),

        TranslateStep("t  9  90deg  60%", 9, deg_to_rad(90), 0.60),
        TranslateStep("t 20 175deg  60%", 20, deg_to_rad(175), 0.60),
        TranslateStep("t 30  90deg 100%", 30, deg_to_rad(90), 1.00),
        TranslateStep("t  8   0deg  60%", 8, deg_to_rad(0), 0.60),
        TranslateTimeStep("t 2.5s 270deg  90%", 2.5, deg_to_rad(270), 0.90),
        TranslateStep("t 19  90deg  60%", 19, deg_to_rad(90), 0.60),
        AnyStep("Kiosk",
                TicketKioskStep("Ticket Kiosk"),
                TranslateStep("Strafe", 4, deg_to_rad(180), 0.40)),
        TranslateStep("t 12 270deg  60%", 12, deg_to_rad(270), 0.60),
        TranslateStep("t 9 180deg  60%", 9, deg_to_rad(180), 0.60),
        TranslateStep("t 24 270deg  60%", 24, deg_to_rad(270), 0.60),

        EndStep()  // end
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
