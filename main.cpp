/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHUtility.h>
#include <LCDColors.h>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

const auto LCD_WIDTH = 320;
const auto LCD_HEIGHT = 240;
const auto FONT_WIDTH = 12;
const auto FONT_HEIGHT = 17;
float touchX, touchY;
bool touchPressed;

/// Helper function to clamp a value between a lower and upper bound
/// @param n The value to clamp
/// @param lower The lower bound
/// @param upper The upper bound
template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}

/// Wrapper for the FEHMotor class
class Motor {
   public:
    /// Constructor for the Motor class
    /// @param port The port to use for the motor
    /// @param correction_factor The correction factor to use for the motor
    Motor(FEHMotor::FEHMotorPort port, double correction_factor);

    /// Sets the power of the motor
    /// @param power The power to set the motor to between -1 and 1
    void drive(double power);

   private:
    FEHMotor motor;
    double correction_factor;
};

Motor::Motor(FEHMotor::FEHMotorPort port, double correction_factor)
    : motor(port, 9.0), correction_factor(correction_factor) {}

void Motor::drive(double power) {
    auto percent = clamp(power * correction_factor, -1.0, 1.0);
    motor.SetPercent(percent * 100.0);
}

/// Base class for all steps
class Step {
   public:
    /// Constructor for the Step class
    /// @param name The name of the step (for debugging)
    Step(std::string name);

    /// Executes the step
    /// @param t The current time
    /// @return Whether the step is done
    virtual bool execute(double t);

    /// The name of the step
    std::string name;

    /// The time at which the step starts
    double t_start;
};

Step::Step(std::string name) : t_start(-1), name(name) {}

bool Step::execute(double t) {
    if (t_start < 0)
        t_start = t;

    return true;
}

/// Translates the robot for a given duration at a given heading
class TranslateStep : public Step {
   public:
    /// Translate (move) the robot for a given duration at a given heading
    /// @param name The name of the step
    /// @param duration The duration of the translation
    /// @param heading The heading (angle) to translate towards
    TranslateStep(std::string name, double duration, double heading);

    /// Execute the translation step
    bool execute(double t) override;

   private:
    double heading;
    double duration;
};

TranslateStep::TranslateStep(std::string name, double duration, double heading)
    : Step(name), duration(duration), heading(heading) {}

bool TranslateStep::execute(double t) {
    Step::execute(t);
    LCD.SetFontColor(WHITE);
    LCD.WriteAt("Translate step", 0, 0);

    return t >= t_start + duration;
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
    LCD.SetFontColor(WHITE);
    LCD.WriteAt("End step", 0, 0);

    return true;
}

/// Rotate the robot for a given duration by a given angle
class RotateStep : public Step {
   public:
    /// Rotate the robot for a given duration by a given angle
    /// @param name The name of the step
    /// @param duration The duration of the rotation
    /// @param theta The angle to rotate by
    RotateStep(std::string name, double duration, double theta);

    /// Execute the rotation step
    bool execute(double t) override;

   private:
    double theta;
    double duration;
};

RotateStep::RotateStep(std::string name, double duration, double theta)
    : Step(name), duration(duration), theta(theta) {}

bool RotateStep::execute(double t) {
    Step::execute(t);
    LCD.SetFontColor(WHITE);
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
    /// @param name The name of the step
    /// @param ts The steps to execute in parallel
    template <typename... Ts>
    UnionStep(std::string name, Ts&&... ts);

    /// Execute the union step
    bool execute(double t) override;

   private:
    std::vector<std::shared_ptr<Step>> steps;
};

template <typename... Ts>
UnionStep::UnionStep(std::string name, Ts&&... ts)
    : Step(name), steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

bool UnionStep::execute(double t) {
    Step::execute(t);

    // iterate through steps and remove them once they are done
    steps.erase(
        std::remove_if(steps.begin(),
                       steps.end(),
                       [&t](const auto& step) { return step->execute(t); }),
        steps.end());

    return steps.size() == 0;
}

/// Represents a rectangle
struct Rect {
    /// Constructor for a rectangle
    /// @param x The x coordinate of the top left corner
    /// @param y The y coordinate of the top left corner
    /// @param width The width of the rectangle
    /// @param height The height of the rectangle
    Rect(unsigned int x,
         unsigned int y,
         unsigned int width,
         unsigned int height)
        : x(x), y(y), width(width), height(height) {}

    /// The x coordinate of the top left corner
    unsigned int x;

    /// The y coordinate of the top left corner
    unsigned int y;

    /// The width of the rectangle
    unsigned int width;

    /// The height of the rectangle
    unsigned int height;
};

class TouchableRegion {
   public:
    /// Constructor for a touchable region
    /// @param rect The rectangle that represents the region
    /// @param on_button_enter The function to call when the button is pressed
    /// @param on_button_exit The function to call when the button is released
    TouchableRegion(
        Rect rect,
        std::function<void()> on_button_enter = []() {},
        std::function<void()> on_button_exit = []() {});

    /// Checks the touch sensor and updates the state of the button
    void update();

    /// The rectangle that represents the region
    Rect rect;

   private:
    /// Type to represent the state of the button
    typedef enum {
        InBounds,
        OutOfBounds,
    } ButtonState;

    /// The current state of the button
    ButtonState state;

    /// The function to call when the button is pressed
    std::function<void()> on_button_enter;

    /// The function to call when the button is released
    std::function<void()> on_button_exit;
};

TouchableRegion::TouchableRegion(Rect rect,
                                 std::function<void()> on_button_enter,
                                 std::function<void()> on_button_exit)
    : rect(rect),
      state(ButtonState::OutOfBounds),
      on_button_enter(on_button_enter),
      on_button_exit(on_button_exit) {}

void TouchableRegion::update() {
    float y_offset = -4;  // our Proteus innacurately reports touch location, so
                          // we must account for it

    bool in_bounds = touchX >= rect.x && touchX <= rect.x + rect.width &&
                     touchY + y_offset >= rect.y &&
                     touchY + y_offset <= rect.y + rect.height;

    if (in_bounds && touchPressed && state == ButtonState::OutOfBounds) {
        state = ButtonState::InBounds;
        on_button_enter();
    } else if (!in_bounds && state != ButtonState::OutOfBounds) {
        state = ButtonState::OutOfBounds;
        on_button_exit();
    }
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
    /// @return True if the timeline is finished executing
    bool timestep(double t);

    /// Update the timeline UI
    void update();

   private:
    /// The steps to execute
    const std::vector<std::shared_ptr<Step>> steps;

    /// The UI regions which can be touched
    std::vector<TouchableRegion> regions;

    /// The index of the current step
    size_t current_step_index = 0;
};

template <typename... Ts>
Timeline::Timeline(Ts&&... ts)
    : steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {
    Rect rect(10, 30, 250, FONT_HEIGHT + 2);
    regions.reserve(steps.size());

    for (const auto& step : steps) {
        LCD.SetFontColor(BLUE);
        LCD.DrawRectangle(rect.x, rect.y, rect.width, rect.height);
        LCD.WriteAt(step->name.c_str(), rect.x + 1, rect.y + 2);
        regions.push_back(TouchableRegion(
            rect,
            [=]() {
                LCD.SetFontColor(GREEN);
                LCD.DrawRectangle(rect.x, rect.y, rect.width, rect.height);
            },
            [=]() {
                LCD.SetFontColor(BLUE);
                LCD.DrawRectangle(rect.x, rect.y, rect.width, rect.height);
            }));

        // draw button controls
        const auto MEASURE = rect.height;
        LCD.DrawRectangle(
            rect.x + rect.width - MEASURE * 1, rect.y, MEASURE, MEASURE);
        LCD.DrawRectangle(
            rect.x + rect.width - MEASURE * 2, rect.y, MEASURE, MEASURE);
        LCD.DrawRectangle(
            rect.x + rect.width - MEASURE * 3, rect.y, MEASURE, MEASURE);

        rect.y += rect.height + 3;
    }
}

bool Timeline::timestep(double t) {
    if (current_step_index >= steps.size())
        return false;

    auto& step = steps[current_step_index];
    if (step->execute(t))
        current_step_index++;

    return true;
}

void Timeline::update() {
    for (auto& region : regions)
        region.update();
}

/// Main function which is the entrypoint for the entire program
int main() {
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    Timeline timeline{
        TranslateStep("Initial move", 1, 90),
        UnionStep("Union",
                  RotateStep("Second rotate", 1, 90),
                  TranslateStep("Second move", 3, 90)),
        RotateStep("Last rotate", 1, 90),
        EndStep(),
    };

    // Main loop
    auto t = 0.0;
    auto start_time = TimeNow();

    auto running = true;
    while (running) {
        t = TimeNow() - start_time;
        touchPressed = LCD.Touch(&touchX, &touchY);

        // running = timeline.timestep(t);
        timeline.timestep(t);

        timeline.update();
    }
}
