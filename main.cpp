/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHLCD.h>
// #include <FEHMotor.h>
#include <FEHUtility.h>
#include <LCDColors.h>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

const auto TOUCH_OFFSET_Y = -4;  // our Proteus innacurately reports touch
                                 // location, so we must account for it

const auto LCD_WIDTH = 320;
const auto LCD_HEIGHT = 240;
const auto FONT_WIDTH = 12;
const auto FONT_HEIGHT = 17;
float touchX, touchY;
bool touchPressed;

#define LOG_INFO(x)                                                  \
    do {                                                             \
        std::stringstream ss;                                        \
        ss << "[i:" << __FUNCTION__ << ":" << __LINE__ << "] " << x; \
        logger->info(ss.str());                                      \
    } while (0)

/// Helper function to clamp a value between a lower and upper bound
/// @param n The value to clamp
/// @param lower The lower bound
/// @param upper The upper bound
template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}

class Log {
   public:
    Log();
    Log(const Log& obj) = delete;

    void info(std::string message);

   private:
    std::vector<std::string> messages;
};

Log::Log() {}
void Log::info(std::string message) {
    // std::cout << message << "\n";
    messages.push_back(message);
}

auto logger = std::make_shared<Log>();

// /// Wrapper for the FEHMotor class
// class Motor {
//   public:
//     /// Constructor for the Motor class
//     /// @param port The port to use for the motor
//     /// @param correction_factor The correction factor to use for the motor
//     Motor(FEHMotor::FEHMotorPort port, double correction_factor);

//     /// Sets the power of the motor
//     /// @param power The power to set the motor to between -1 and 1
//     void drive(double power);

//   private:
//     FEHMotor motor;
//     double correction_factor;
// };

// Motor::Motor(FEHMotor::FEHMotorPort port, double correction_factor)
//     : motor(port, 9.0), correction_factor(correction_factor) {
// }

// void Motor::drive(double power) {
//     auto percent = clamp(power * correction_factor, -1.0, 1.0);
//     motor.SetPercent(percent * 100.0);
// }

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

    /// The time at which the step started
    double t_start;

    /// The time at which the step ended
    double t_end;
};

Step::Step(std::string name) : t_start(0), t_end(0), name(name) {}

bool Step::execute(double t) { return true; }

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
    if (t >= t_start + duration) {
        LOG_INFO("translate step done");
    }

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
    LOG_INFO("end");

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
    if (t >= t_start + duration) {
        LOG_INFO("rotate step done");
    }

    return t >= t_start + duration;
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
    for (auto& step : steps)
        step->t_start = t_start;

    return std::all_of(steps.begin(), steps.end(), [t](const auto& step) {
        return step->execute(t);
    });
}

/// Represents a rectangle
struct Rect {
    /// Default constructor for a rectangle
    Rect() : x(0), y(0), width(0), height(0) {}

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
    bool in_bounds = touchX >= rect.x && touchX <= rect.x + rect.width &&
                     touchY + TOUCH_OFFSET_Y >= rect.y &&
                     touchY + TOUCH_OFFSET_Y <= rect.y + rect.height;

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

    void draw(Rect working_area);

    /// Update the timeline UI
    void update(Rect working_area, double t);

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
    : steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

void Timeline::draw(Rect working_area) {
    Rect block =
        Rect(1, working_area.y, working_area.width - 1, FONT_HEIGHT * 2 + 2);
    for (const auto& step : steps) {
        LCD.SetFontColor(WHITE);
        LCD.SetBackgroundColor(BLACK);
        LCD.DrawRectangle(block.x, block.y, block.width, block.height);
        LCD.WriteAt(step->name.c_str(), block.x + 1, block.y + 2);
        LCD.WriteAt("t=", block.x + 1, block.y + 2 + FONT_HEIGHT);

        block.y += block.height;
    }
}

void Timeline::update(Rect working_area, double t) {
    for (auto& region : regions)
        region.update();

    if (current_step_index >= steps.size())
        return;

    Rect block =
        Rect(1,
             working_area.y + (FONT_HEIGHT * 2 + 2) * current_step_index,
             working_area.width - 1,
             FONT_HEIGHT * 2 + 2);
    auto& step = steps[current_step_index];
    // LCD.SetFontColor(BLACK);
    // LCD.FillRectangle(block.x + 1 + (FONT_WIDTH * 2), block.y + 2 +
    // FONT_HEIGHT,
    //                   FONT_WIDTH * 5, FONT_HEIGHT - 1);

    LCD.SetFontColor(WHITE);
    LCD.SetBackgroundColor(BLACK);
    LCD.WriteAt(t - step->t_start,
                block.x + 1 + (FONT_WIDTH * 2),
                block.y + 2 + FONT_HEIGHT);
}

bool Timeline::timestep(double t) {
    if (current_step_index >= steps.size())
        return false;

    auto& step = steps[current_step_index];
    if (step->execute(t)) {
        step->t_end = t;
        current_step_index++;

        if (current_step_index < steps.size()) {
            auto& next_step = steps[current_step_index];
            next_step->t_start = t;
        } else {
            return false;
        }
    }

    return true;
}

struct TopBarButton {
    TopBarButton(Rect bounding_box,
                 std::string text,
                 std::function<void()> on_button_down);

    Rect bounding_box;
    std::string text;
    std::function<void()> on_button_down;
    std::unique_ptr<TouchableRegion> region;

    void render(bool is_pressed);
};

class TopBar {
   public:
    TopBar();

    void add_button(const std::string& text,
                    std::function<void()> on_button_down);

    void update();

    void draw();

    Rect bounding_box;

   private:
    const unsigned int OUTER_PADDING = 4;
    const unsigned int INNER_PADDING = 2;

    std::vector<TopBarButton> buttons;
    unsigned int x;
    unsigned int current_selected;
};

TopBarButton::TopBarButton(Rect bounding_box,
                           std::string text,
                           std::function<void()> on_button_down)
    : bounding_box(bounding_box), text(text), on_button_down(on_button_down) {
    region = std::make_unique<TouchableRegion>(
        bounding_box, on_button_down, []() {});
}

void TopBarButton::render(bool is_pressed) {
    LCD.SetFontColor(is_pressed ? GRAY : BLACK);
    LCD.FillRectangle(bounding_box.x + 1,
                      bounding_box.y + 1,
                      bounding_box.width - 1,
                      bounding_box.height - 1);
    LCD.SetFontColor(WHITE);
    LCD.SetBackgroundColor(is_pressed ? GRAY : BLACK);
    LCD.DrawRectangle(bounding_box.x,
                      bounding_box.y,
                      bounding_box.width,
                      bounding_box.height);
    LCD.WriteAt(text.c_str(), bounding_box.x + 1, bounding_box.y + 3);
}

TopBar::TopBar() : x(0), current_selected(0) {
    const auto y = OUTER_PADDING + INNER_PADDING + FONT_HEIGHT + INNER_PADDING;
    bounding_box = Rect(0, 0, LCD_WIDTH, y);
    LCD.DrawHorizontalLine(y, 0, LCD_WIDTH);
}
void TopBar::add_button(const std::string& text,
                        std::function<void()> on_button_down) {
    const auto TEXT_WIDTH = text.length() * FONT_WIDTH;
    const Rect button_rect(OUTER_PADDING + x,
                           OUTER_PADDING,
                           INNER_PADDING + TEXT_WIDTH + INNER_PADDING,
                           INNER_PADDING + FONT_HEIGHT + INNER_PADDING);

    auto index = buttons.size();
    buttons.push_back(TopBarButton(
        button_rect,
        text,
        [this, index, on_button_down]() {  // future segfault, `this` could move
            if (this->current_selected != index) {
                buttons[this->current_selected].render(false);
                buttons[index].render(true);
                this->current_selected = index;

                on_button_down();
            }
        }));

    buttons.back().render(false);

    // the first insertion should be selected and render its own ui
    if (index == 0) {
        buttons[this->current_selected].render(true);
        on_button_down();
    }

    x += button_rect.width + OUTER_PADDING;
}

void TopBar::update() {
    for (auto& button : buttons)
        button.region->update();
}

void TopBar::draw() {}

/// Main function which is the entrypoint for the entire program
int main() {
    LOG_INFO("starting");
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    Timeline timeline{
        TranslateStep("Initial move", 3, 90),
        SleepStep("Sleep", 1),
        UnionStep("Union",
                  RotateStep("Second rotate", 2, 90),
                  TranslateStep("Second move", 3, 90)),
        RotateStep("Last rotate", 1, 90),
        EndStep(),
    };

    TopBar top_bar;
    Rect working_area = Rect(0,
                             top_bar.bounding_box.height,
                             LCD_WIDTH,
                             LCD_HEIGHT - top_bar.bounding_box.height);
    top_bar.add_button("Timeline", [&]() { timeline.draw(working_area); });
    top_bar.add_button("Logs", []() {});
    top_bar.add_button("Stats", []() {});

    LCD.SetFontColor(PINK);
    LCD.DrawRectangle(1, 0, LCD_WIDTH - 2, LCD_HEIGHT - 2);

    // Main loop
    auto t = 0.0;
    auto start_time = TimeNow();

    auto running = true;
    while (running) {
        t = TimeNow() - start_time;
        touchPressed = LCD.Touch(&touchX, &touchY);

        // running = timeline.timestep(t);
        timeline.timestep(t);

        top_bar.update();
        timeline.update(working_area, t);
    }
}
