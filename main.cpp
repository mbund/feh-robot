/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHUtility.h>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <vector>

const auto LCD_WIDTH = 320;
const auto LCD_HEIGHT = 240;
int touchX, touchY;
bool touchPressed;

/// Wrapper for the FEHMotor class
class Motor {
   public:
    /// Constructor for the Motor class
    Motor(FEHMotor::FEHMotorPort port, double correction_factor);

    /// Sets the power of the motor
    /// @param power The power to set the motor to between -1 and 1
    auto drive(double power) const -> void;

   private:
    FEHMotor motor;
    double correction_factor;
};

/// Base class for all steps
class Step {
   public:
    /// Constructor for the Step class
    Step();

    /// Executes the step
    /// @param t The current time
    /// @return Whether the step is done
    virtual auto execute(double t) -> bool;

    /// The time at which the step starts
    double t_start;
};

Step::Step() : t_start(-1) {}

auto Step::execute(double t) -> bool {
    if (t_start < 0)
        t_start = t;

    return true;
}

/// Translates the robot for a given duration at a given heading
class TranslateStep : public Step {
   public:
    /// Translate (move) the robot for a given duration at a given heading
    TranslateStep(double duration, double heading);

    /// Execute the translation step
    auto execute(double t) -> bool override;

   private:
    double heading;
    double duration;
};

TranslateStep::TranslateStep(double duration, double heading)
    : Step(), duration(duration), heading(heading) {}

auto TranslateStep::execute(double t) -> bool {
    Step::execute(t);
    LCD.WriteAt("Translate step", 0, 0);

    return t >= t_start + duration;
}

/// Ends the program
class EndStep : public Step {
   public:
    /// Constructor to make a step that ends the program
    EndStep();

    /// Execute the end step
    auto execute(double t) -> bool override;
};

EndStep::EndStep() : Step() {}

auto EndStep::execute(double t) -> bool {
    LCD.WriteAt("End step", 0, 0);

    return true;
}

/// Rotate the robot for a given duration by a given angle
class RotateStep : public Step {
   public:
    /// Rotate the robot for a given duration by a given angle
    /// @param duration The duration of the rotation
    /// @param theta The angle to rotate by
    RotateStep(double duration, double theta);

    /// Execute the rotation step
    auto execute(double t) -> bool override;

   private:
    double theta;
    double duration;
};

RotateStep::RotateStep(double duration, double theta)
    : Step(), duration(duration), theta(theta) {}

auto RotateStep::execute(double t) -> bool {
    Step::execute(t);
    LCD.WriteAt("Rotate step", 0, 0);

    return t >= t_start + duration;
}

template <typename T, typename... Ts>
std::shared_ptr<T> make_shared(Ts&&... ts) {
    return std::shared_ptr<T>(new T{std::forward<Ts>(ts)...});
}

template <typename Base, typename... Ts>
std::vector<std::shared_ptr<Base>> make_vector_of_shared(Ts&&... ts) {
    std::shared_ptr<Base> init[] = {make_shared<Ts>(std::forward<Ts>(ts))...};
    return std::vector<std::shared_ptr<Base>>{
        std::make_move_iterator(std::begin(init)),
        std::make_move_iterator(std::end(init))};
}

/// Execute a set of steps in parallel
class UnionStep : public Step {
   public:
    /// Constructor for a union step
    /// @param ts The steps to execute in parallel
    template <typename... Ts>
    UnionStep(Ts&&... ts);

    /// Execute the union step
    auto execute(double t) -> bool override;

   private:
    std::vector<std::shared_ptr<Step>> steps;
};

template <typename... Ts>
UnionStep::UnionStep(Ts&&... ts)
    : Step(), steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

auto UnionStep::execute(double t) -> bool {
    Step::execute(t);

    // iterate through steps and remove them once they are done
    steps.erase(
        std::remove_if(steps.begin(),
                       steps.end(),
                       [&t](const auto& step) { return step->execute(t); }),
        steps.end());

    return steps.size() == 0;
}

/// Sequence of steps to execute
class Timeline {
   public:
    /// Constructor for the Timeline
    /// @param ts The steps to execute in order
    template <typename... Ts>
    Timeline(Ts&&... ts);

    /// Execute the timeline
    /// @param t The current time
    auto timestep(double t) -> bool;

   private:
    const std::vector<std::shared_ptr<Step>> steps;
    size_t current_step_index = 0;
};

template <typename... Ts>
Timeline::Timeline(Ts&&... ts)
    : steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

auto Timeline::timestep(double t) -> bool {
    if (current_step_index >= steps.size())
        return false;

    auto& step = steps[current_step_index];
    if (step->execute(t))
        current_step_index++;

    return true;
}

/// Main function which is the entrypoint for the entire program
auto main() -> int {
    Timeline timeline{
        TranslateStep(1, 90),
        UnionStep(RotateStep(1, 90), TranslateStep(3, 90)),
        RotateStep(1, 90),
        EndStep(),
    };

    // Main loop

    auto t = 0.0;
    auto start_time = TimeNow();

    auto running = true;
    while (running) {
        t = TimeNow() - start_time;
        touchPressed = LCD.Touch(&touchX, &touchY);
        running = timeline.timestep(t);
    }
}
