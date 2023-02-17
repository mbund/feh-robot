/// @file main.cpp
/// @author Mark Bundschuh
/// @brief Contains the entrypoint for the program

#include <FEHLCD.h>
#include <FEHUtility.h>
#include <LCDColors.h>
// #include <FEHMotor.h>
// #include <FEHBattery.h>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// forward declarations
class Timeline;
class TimelineUI;
class UIWindow;
class Navbar;
struct Rect;

const auto TOUCH_OFFSET_Y = -4;  // our Proteus innacurately reports touch
                                 // location, so we must account for it

const auto LCD_WIDTH = 320;
const auto LCD_HEIGHT = 240;
const auto FONT_WIDTH = 12;
const auto FONT_HEIGHT = 17;
float touchX, touchY;
bool touchPressed;

inline std::string method_name(const std::string& pretty_function) {
    size_t colons = pretty_function.find("::");
    size_t begin = pretty_function.substr(0, colons).rfind(" ") + 1;
    size_t end = pretty_function.rfind("(") - begin;

    return pretty_function.substr(begin, end);
}

#define __METHOD_NAME__ method_name(__PRETTY_FUNCTION__)

#define LOG_INFO(x)                         \
    do {                                    \
        std::stringstream ss;               \
        ss << "i" << __LINE__ << "| " << x; \
        logger->info(ss.str());             \
    } while (0)

#define LOG_INFO_LONG(x)                                                \
    do {                                                                \
        std::stringstream ss;                                           \
        ss << "[i:" << __METHOD_NAME__ << ":" << __LINE__ << "] " << x; \
        logger->info(ss.str());                                         \
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

    friend class LogUI;

   private:
    std::vector<std::string> messages;
};

Log::Log() {}

void Log::info(std::string message) {
    // std::cout << message << "\n";
    messages.push_back(message);
}

auto logger = std::make_shared<Log>();

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
    // if (t >= t_start + duration) {
    //     LOG_INFO("rotate step done");
    // }

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

class UIWindow {
   public:
    /// Constructor for the UIWindow class
    UIWindow(Rect bounds);

    /// Initial bulk render of the window (full re-render)
    virtual void render() = 0;

    /// Inject updates to the UI
    virtual void update(double t) = 0;

   protected:
    Rect bounds;
};

UIWindow::UIWindow(Rect bounds) : bounds(bounds) {}

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

struct NavbarButton {
    NavbarButton(Rect bounding_box,
                 std::string text,
                 std::function<void()> on_button_down);

    Rect bounding_box;
    std::string text;
    std::function<void()> on_button_down;
    std::unique_ptr<TouchableRegion> region;

    void render(bool is_pressed);
};

class Navbar {
   public:
    Navbar();

    void add_button(const std::string& text,
                    std::function<void()> on_button_down);

    void update();

    Rect bounding_box;

    friend class LogUI;
    friend class TimelineUI;
    friend class StatsUI;

   private:
    const unsigned int OUTER_PADDING = 4;
    const unsigned int INNER_PADDING = 2;

    std::vector<NavbarButton> buttons;
    unsigned int x;
    unsigned int current_selected;
};

NavbarButton::NavbarButton(Rect bounding_box,
                           std::string text,
                           std::function<void()> on_button_down)
    : bounding_box(bounding_box), text(text), on_button_down(on_button_down) {
    region = std::make_unique<TouchableRegion>(
        bounding_box, on_button_down, []() {});
}

void NavbarButton::render(bool is_pressed) {
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

Navbar::Navbar() : x(0), current_selected(0) {
    const auto y = OUTER_PADDING + INNER_PADDING + FONT_HEIGHT + INNER_PADDING;
    bounding_box = Rect(0, 0, LCD_WIDTH, y);
    LCD.DrawHorizontalLine(y, 0, LCD_WIDTH);
}
void Navbar::add_button(const std::string& text,
                        std::function<void()> on_button_down) {
    const auto TEXT_WIDTH = text.length() * FONT_WIDTH;
    const Rect button_rect(OUTER_PADDING + x,
                           OUTER_PADDING,
                           INNER_PADDING + TEXT_WIDTH + INNER_PADDING,
                           INNER_PADDING + FONT_HEIGHT + INNER_PADDING);

    auto index = buttons.size();
    buttons.push_back(NavbarButton(
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

void Navbar::update() {
    for (auto& button : buttons)
        button.region->update();
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

    friend class TimelineUI;

   private:
    /// The steps to execute
    const std::vector<std::shared_ptr<Step>> steps;

    /// The index of the current step
    size_t current_step_index = 0;
};

template <typename... Ts>
Timeline::Timeline(Ts&&... ts)
    : steps(make_vector_of_shared<Step>(std::forward<Ts>(ts)...)) {}

bool Timeline::timestep(double t) {
    if (current_step_index >= steps.size())
        return false;

    auto& step = steps[current_step_index];
    step->t_end = t;
    if (step->execute(t)) {
        current_step_index++;
        LOG_INFO("maybe moving to next step");

        if (current_step_index < steps.size()) {
            auto& next_step = steps[current_step_index];
            next_step->t_start = t;
            LOG_INFO("moving to next step");
        } else {
            return false;
            LOG_INFO("timeline done");
        }
    }

    return true;
}

class TimelineUI : public UIWindow {
   public:
    /// Constructor for the TimelineUI class
    /// @param timeline The timeline to display
    TimelineUI(Rect bounds, Timeline& timeline, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update(double t) override;

   private:
    /// The timeline to display
    Timeline& timeline;

    Navbar& navbar;

    /// The UI regions which can be touched
    std::vector<TouchableRegion> regions;
};

TimelineUI::TimelineUI(Rect bounds, Timeline& timeline, Navbar& navbar)
    : UIWindow(bounds), timeline(timeline), navbar(navbar) {}

void TimelineUI::render() {
    LCD.SetFontColor(BLACK);
    LCD.FillRectangle(bounds.x, bounds.y, bounds.width, bounds.height);

    LCD.SetFontColor(WHITE);
    LCD.SetBackgroundColor(BLACK);

    Rect block = Rect(1, bounds.y, bounds.width - 1, FONT_HEIGHT * 2 + 2);
    for (const auto& step : timeline.steps) {
        LCD.DrawRectangle(block.x, block.y, block.width, block.height);
        LCD.WriteAt(step->name.c_str(), block.x + 1, block.y + 2);
        LCD.WriteAt("t=", block.x + 1, block.y + 2 + FONT_HEIGHT);
        LCD.WriteAt(step->t_end - step->t_start,
                    block.x + 1 + (FONT_WIDTH * 2),
                    block.y + 2 + FONT_HEIGHT);

        block.y += block.height;
    }
}

void TimelineUI::update(double t) {
    if (navbar.current_selected != 0)
        return;

    for (auto& region : regions)
        region.update();

    LCD.SetFontColor(WHITE);
    LCD.SetBackgroundColor(BLACK);

    Rect block = Rect(1, bounds.y, bounds.width - 1, FONT_HEIGHT * 2 + 2);
    for (const auto& step : timeline.steps) {
        // LCD.SetFontColor(BLACK);
        // LCD.FillRectangle(block.x + 1 + (FONT_WIDTH * 2),
        //                   block.y + 2 + FONT_HEIGHT, FONT_WIDTH * 7,
        //                   FONT_HEIGHT - 1);
        // LCD.SetFontColor(WHITE);

        LCD.WriteAt(step->t_end - step->t_start,
                    block.x + 1 + (FONT_WIDTH * 2),
                    block.y + 2 + FONT_HEIGHT);

        block.y += block.height;
    }
}

class LogUI : public UIWindow {
   public:
    LogUI(Rect bounds, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update(double t) override;

   private:
    /// The UI regions which can be touched
    std::vector<TouchableRegion> regions;

    Navbar& navbar;
};

LogUI::LogUI(Rect bounds, Navbar& navbar) : UIWindow(bounds), navbar(navbar) {}

void LogUI::render() {
    LCD.SetFontColor(BLACK);
    LCD.FillRectangle(bounds.x, bounds.y, bounds.width, bounds.height);

    update(0);
}

void LogUI::update(double t) {
    if (navbar.current_selected != 1)
        return;

    LCD.SetBackgroundColor(BLACK);
    LCD.SetFontColor(WHITE);

    const size_t max_lines = bounds.height / FONT_HEIGHT;
    size_t scroll_index = 0;
    if (logger->messages.size() > max_lines)
        scroll_index = logger->messages.size() - max_lines;
    for (size_t log_index = scroll_index, ui_index = 0;
         log_index < scroll_index + max_lines &&
         log_index < logger->messages.size();
         log_index++, ui_index++) {
        LCD.WriteAt(logger->messages[log_index].substr(0, 27).c_str(),
                    bounds.x,
                    bounds.y + (ui_index * FONT_HEIGHT));
    }
}

class StatsUI : public UIWindow {
   public:
    StatsUI(Rect bounds, Navbar& navbar);

    /// Initial bulk render of the window (full re-render)
    void render() override;

    /// Inject updates to the UI
    void update(double t) override;

   private:
    /// The UI regions which can be touched
    std::vector<TouchableRegion> regions;

    Navbar& navbar;
};

StatsUI::StatsUI(Rect bounds, Navbar& navbar)
    : UIWindow(bounds), navbar(navbar) {}

void StatsUI::render() {
    LCD.SetFontColor(BLACK);
    LCD.FillRectangle(bounds.x, bounds.y, bounds.width, bounds.height);

    update(0);
}

void StatsUI::update(double t) {
    if (navbar.current_selected != 2)
        return;

    LCD.SetBackgroundColor(BLACK);
    LCD.SetFontColor(WHITE);

    const std::string battery_text = "Battery (V): ";
    LCD.WriteAt(battery_text.c_str(), bounds.x, bounds.y);
    // LCD.WriteAt(Battery.Voltage(),
    //             bounds.x + (battery_text.length() * FONT_WIDTH), bounds.y);
}

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

    Navbar navbar;
    Rect working_area = Rect(0,
                             navbar.bounding_box.height + 1,
                             LCD_WIDTH,
                             LCD_HEIGHT - navbar.bounding_box.height);
    TimelineUI timeline_ui(working_area, timeline, navbar);
    LogUI log_ui(working_area, navbar);
    StatsUI stats_ui(working_area, navbar);
    navbar.add_button("Timeline", [&]() { timeline_ui.render(); });
    navbar.add_button("Logs", [&]() { log_ui.render(); });
    navbar.add_button("Stats", [&]() { stats_ui.render(); });

    // Main loop
    auto t = 0.0;
    auto start_time = TimeNow();

    auto running = true;
    while (running) {
        t = TimeNow() - start_time;
        touchPressed = LCD.Touch(&touchX, &touchY);

        // running = timeline.timestep(t);
        timeline.timestep(t);

        navbar.update();
        timeline_ui.update(t);
        log_ui.update(t);
        stats_ui.update(t);

        // drawable region of our proteus is less than defined LCD values?????
        // LCD.SetFontColor(PINK);
        // LCD.DrawRectangle(1, 0, LCD_WIDTH - 2, LCD_HEIGHT - 2);
    }
}
