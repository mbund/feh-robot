/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHUtility.h>
#include <cstdlib>
#include <memory>
#include <vector>

const auto LCD_WIDTH = 320;
const auto LCD_HEIGHT = 240;
int touchX, touchY;
bool touchPressed;

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

struct Step {
    /// Constructor for the Step class
    /// @param t_start The time at which the step starts
    /// @param t_end The time at which the step ends
    Step(double t_start, double t_end);

    /// Executes the step
    virtual auto action() const -> void;

    /// The time at which the step starts
    double t_start;

    /// The time at which the step ends
    double t_end;
};

Step::Step(double t_start, double t_end) : t_start(t_start), t_end(t_end) {}
auto Step::action() const -> void {}

struct TranslateStep : public Step {
   public:
    TranslateStep(double t_start, double t_end, double heading);
    auto action() const -> void override;

   private:
    double heading;
};

TranslateStep::TranslateStep(double t_start, double t_end, double heading)
    : Step(t_start, t_end), heading(heading) {}

auto TranslateStep::action() const -> void {}

struct EndStep : public Step {
   public:
    EndStep(double t);
    auto action() const -> void override;
};
EndStep::EndStep(double t) : Step(t, t + 1) {}
auto EndStep::action() const -> void { exit(1); }

struct RotateStep : public Step {
   public:
    RotateStep(double t_start, double t_end, double theta);
    auto action() const -> void override;

   private:
    double theta;
};

RotateStep::RotateStep(double t_start, double t_end, double theta)
    : Step(t_start, t_end), theta(theta) {}

auto RotateStep::action() const -> void {}

template <typename T, typename... Ts>
std::unique_ptr<T> make_unique(Ts&&... args) {
    return std::unique_ptr<T>(new T{std::forward<Ts>(args)...});
}

template <typename Base, typename... Ts>
std::vector<std::unique_ptr<Base>> make_vector_of_unique(Ts&&... ts) {
    std::unique_ptr<Base> init[] = {make_unique<Ts>(std::forward<Ts>(ts))...};
    return std::vector<std::unique_ptr<Base>>{
        std::make_move_iterator(std::begin(init)),
        std::make_move_iterator(std::end(init))};
}

class Timeline {
   public:
    template <typename... Ts>
    Timeline(Ts&&... t);

    void execute(double t);

   private:
    const std::vector<std::unique_ptr<Step>> steps;
};

template <typename... Ts>
Timeline::Timeline(Ts&&... t)
    : steps(make_vector_of_unique<Step>(std::forward<Ts>(t)...)) {}

auto Timeline::execute(double t) -> void {
    for (const auto& step : steps) {
        if (t >= step->t_start && t <= step->t_end) {
            step->action();
        }
    }
}

/// Main function which is the entrypoint for the entire program
auto main() -> int {
    Timeline timeline{
        TranslateStep(0, 1, 90),
        RotateStep(1, 2, 90),
        TranslateStep(2, 3, 90),
        RotateStep(3, 4, 90),
        EndStep(4),
    };

    // Main loop

    auto t = 0.0;
    auto dt = 0.01;

    auto start_time = TimeNow();

    while (true) {
        t = TimeNow() - start_time;

        touchPressed = LCD.Touch(&touchX, &touchY);
        timeline.execute(t);
    }
}
