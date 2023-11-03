// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
//
// This code is public domain
// (but note, once linked against the led-matrix library, this is
// covered by the GPL v2)
//
// Basic operations originally taken from
// https://github.com/hzeller/rpi-rgb-led-matrix
#include <assert.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include "graphics.h"
#include "led-matrix.h"
#include "pixel-mapper.h"

using std::cout;
using std::max;
using std::min;

#define TERM_ERR "\033[1;31m"
#define TERM_NORM "\033[0m"

#define PLASMA_PALETTE_SIZE 240
#define MAX_PLASMA_COLORS 6

using namespace rgb_matrix;

volatile bool interrupt_received = false;
volatile bool keystroke_exited = false;
volatile bool SIGUSR1_received = false;
volatile bool SIGUSR2_received = false;
static void InterruptHandler(int signo) { interrupt_received = true; }
static void SIGUSR1Handler(int signo) { SIGUSR1_received = true; }
static void SIGUSR2Handler(int signo) { SIGUSR2_received = true; }

// Pass pointer to Color, pointer to string. Color gets updated
// Returns bool true if parsing successful
// Ex: if (!parseColor(&color, optarg)) ...
static bool parseColor(Color *c, std::string str) {
  try {
    std::stringstream colorspec(str);
    std::vector<int> result;
    while (colorspec.good()) {
      std::string substr;
      getline(colorspec, substr, ',');
      result.push_back(std::stoi(substr));
    }
    c->r = result[0];
    c->g = result[1];
    c->b = result[2];
    return true;
  }

  catch (...) {
    return false;
  }
}

class DemoRunner {
 protected:
  DemoRunner(Canvas *canvas) : canvas_(canvas) {}
  inline Canvas *canvas() { return canvas_; }

 public:
  virtual ~DemoRunner() {}
  virtual void Run() = 0;

 private:
  Canvas *const canvas_;
};

void wait_for_keystroke(void) {
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  keystroke_exited = true;
}

// Particle Swarm
class ParticleSwarm : public DemoRunner {
 public:
  ParticleSwarm(Canvas *m, int model = 0) : DemoRunner(m), model_(model) {}

  const float max_swarm_rot_speed = 90.0;

  void Run() override {
    float rot_speed_x = 1.0;   // 0.1;
    float rot_speed_y = 1.0;   // 0.1;
    float rot_speed_z = -1.0;  // -0.1;

    float angle_x = 0, angle_y = 0, angle_z = 0;
    float x_tmp, y_tmp;

    float x_speed = 0.10, y_speed = 0.07;
    float max_abs_speed = 0.15;
    float center_y = 16, center_x = 32;

    // A Cube
    Eigen::Matrix<double, 20, 3> cube{
        {-6, -6, -6}, {-6, -6, 0}, {-6, -6, 6}, {-6, 0, -6}, {-6, 0, 6},
        {-6, 6, -6},  {-6, 6, 0},  {-6, 6, 6},  {6, -6, -6}, {6, -6, 0},
        {6, -6, 6},   {6, 0, -6},  {6, 0, 6},   {6, 6, -6},  {6, 6, 0},
        {6, 6, 6},    {0, -6, -6}, {0, -6, 6},  {0, 6, -6},  {0, 6, 6}};
    Eigen::Matrix<double, 18, 3> octahedron{
        {6, 0, 0},  {-6, 0, 0}, {0, 6, 0},  {0, -6, 0},  {0, 0, 6},
        {0, 0, -6}, {3, 3, 0},  {3, -3, 0}, {-3, 3, 0},  {-3, -3, 0},
        {3, 0, 3},  {3, 0, -3}, {-3, 0, 3}, {-3, 0, -3}, {0, 3, 3},
        {0, -3, 3}, {0, 3, -3}, {0, -3, -3}};

    std::vector<Eigen::RowVector3d> points;

    // Load points with the model
    switch (model_) {
      case 0:
        for (int i = 0; i < cube.rows(); ++i) points.push_back(cube.row(i));
        break;
      case 1:
        for (int i = 0; i < octahedron.rows(); ++i)
          points.push_back(octahedron.row(i));
        break;
    }

    while (!interrupt_received) {
      angle_x += rot_speed_x * M_PI / 180.0;
      angle_y += rot_speed_y * M_PI / 180.0;
      angle_z += rot_speed_z * M_PI / 180.0;

      Eigen::AngleAxisd pitch(angle_x, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd yaw(angle_y, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd roll(angle_z, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond quat = roll * yaw * pitch;

      Eigen::Matrix3d rotationMatrix = quat.matrix();

      // For determining if we're bumping against the walls
      float max_x = 0.0, min_x = 64.0, max_y = 0.0, min_y = 32.0;

      canvas()->Clear();

      Eigen::RowVector3d translation{center_x, center_y, 0};

      std::vector<Eigen::RowVector3d> transformed_points;

      for (int64_t i = 0; i < points.size(); ++i) {
        Eigen::RowVector3d point = points[i];
        transformed_points.push_back(point * rotationMatrix);
      }

      // Z-sort points, so we can draw them with correct brightness later
      std::sort(transformed_points.begin(), transformed_points.end(),
                [](Eigen::RowVector3d const &t1, Eigen::RowVector3d const &t2) {
                  return t1(2) < t2(2);
                });

      for (int64_t i = 0; i < points.size(); ++i) {
        Eigen::RowVector3d point = transformed_points[i] += translation;
        x_tmp = point(0);
        y_tmp = point(1);

        max_x = max(max_x, x_tmp);
        min_x = min(min_x, x_tmp);
        max_y = max(max_y, y_tmp);
        min_y = min(min_y, y_tmp);

        float px_value = 128.0 + (10 * point(2));

        // Four pixels, to reduce apparent jumpiness
        canvas()->SetPixel(x_tmp - 1, y_tmp - 1, px_value, px_value, px_value);
        canvas()->SetPixel(x_tmp, y_tmp - 1, px_value, px_value, px_value);
        canvas()->SetPixel(x_tmp - 1, y_tmp, px_value, px_value, px_value);
        canvas()->SetPixel(x_tmp, y_tmp, px_value, px_value, px_value);
      };

      // If we've hit an edge, bounce
      if ((max_x >= 63 && x_speed > 0) ||  // Right side
          (min_x <= 1 && x_speed < 0)) {   // Left side
        x_speed = -x_speed;
        y_speed += random_deflection();  // introduce some randomness
      }
      if ((max_y >= 31 && y_speed > 0) ||  // Bottom
          (min_y <= 1 && y_speed < 0)) {   // Top
        y_speed = -y_speed;
        x_speed += random_deflection();  // introduce some randomness
      }

      //  Keep speeds within bounds
      if (y_speed > 0) {
        y_speed = min(y_speed, max_abs_speed);
      } else {
        y_speed = max(y_speed, -max_abs_speed);
      }
      if (x_speed > 0) {
        x_speed = min(x_speed, max_abs_speed);
      } else {
        x_speed = max(x_speed, -max_abs_speed);
      }

      center_x += x_speed;
      center_y += y_speed;

      usleep(8 * 1000);
    }
  }

 private:
  float random_deflection(void) { return float((rand() % 3) - 2) / 100.0; }

  int model_;
};

// Plasma
// Translated from Arduino code
// TODO:
// - Add palette saving and loading
// - Add clock overlay
class Plasma : public DemoRunner {
 public:
  Plasma(Canvas *m, int plasma_color_count = 6, std::time_t end_time = 0,
         std::string palette_filename = "", int sleep_delay = 5)
      : DemoRunner(m),
        plasma_color_count_(plasma_color_count),
        end_time_(end_time),
        palette_filename_(palette_filename),
        sleep_delay_(sleep_delay) {}

  double sqrt_lut[5120];

  Color plasma_palette[PLASMA_PALETTE_SIZE];
  Color anchor_colors[MAX_PLASMA_COLORS];

  void Run() override {
    std::thread keystroke_to_exit(wait_for_keystroke);
    keystroke_to_exit.detach();

    bool on_timer = end_time_ > 0;

    plasma_color_count_ = min(plasma_color_count_, MAX_PLASMA_COLORS);

    double current_value = float(rand() % 10000);
    double step_value = 0.07;

    const double plasma_randomizing_param = 10 + ((rand() % 750) / 50.0);
    const int width = canvas()->width();
    const int height = canvas()->height();

    // Generate palette

    bool generate_random = true;

    if (palette_filename_ != "") {
      std::ifstream infile(palette_filename_);
      if (infile) {
        cout << "Loading palette from file: " << palette_filename_ << '\n';
        plasma_color_count_ = 0;
        std::string line;
        while (std::getline(infile, line)) {
          Color newColor;
          if (line[0] != '#') {  // Comments? I dunno
            if (parseColor(&newColor, line)) {
              anchor_colors[plasma_color_count_] = newColor;
              plasma_color_count_++;
              if (plasma_color_count_ == MAX_PLASMA_COLORS) {
                break;
              }
            }
          }
        }

        if (plasma_color_count_ > 1) {
          generate_random = false;
        } else {
          cout << "Insufficient palette entries found.\n";
          generate_random = true;
          plasma_color_count_ = 6;
        }
      } else {
        cout << "Could not find palette file " << palette_filename_ << '\n';
        generate_random = true;
      }
    }

    if (generate_random) {
      generate_random_palette_anchors();
    }

    dump_palette();

    smooth_palette();

    //////////

    // Generate LUTs
    for (int x = 0; x < 64; x++) {
      for (int y = 0; y < 32; y++) {
        int idx = (y * 64) + x;
        double dy = double(y) / 2.0;
        sqrt_lut[idx] = sqrt((x * x) + (dy * dy)) / 8.0;
      }
    }
    //////////

    double x_drift = 0.0;
    double y_drift = 0.0;
    const double x_drift_rate = drift_rate();
    const double y_drift_rate = drift_rate();

    while (!interrupt_received && !keystroke_exited &&
           !(on_timer && time(NULL) >= end_time_)) {
      if (SIGUSR1_received) {
        SIGUSR1_received = false;
        generate_random_palette_anchors();
        dump_palette();
        smooth_palette();
      }

      x_drift += x_drift_rate;
      y_drift += y_drift_rate;

      for (int x = 0; x < width; x++) {
        double dx = double(x) + x_drift;
        for (int y = 0; y < height; y++) {
          double dy = (double(y) + y_drift) / 1.5;

          double calculated_value =
              sinf((dx + current_value) / plasma_randomizing_param) +
              cosf((dy + (current_value / 2.0)) / 9.0) +
              sinf(((dx + current_value) + (dy - current_value)) / 25.0) +
              sinf(sqrt_lut[(y * 64) + x]);
          int scaled_value = int(abs(calculated_value) * 100);
          Color output_color =
              plasma_palette[scaled_value % PLASMA_PALETTE_SIZE];
          canvas()->SetPixel(x, y, output_color.r, output_color.g,
                             output_color.b);
        }
      }

      current_value += step_value;
      usleep(sleep_delay_ *
             1000);  // limits to ~60fps; each frame takes ~11ms to generate
    }
  }

 private:
  Color interpolate_palette(Color from, Color to, double factor) {
    int red = int(from.r + ((to.r - from.r) * factor));
    int green = int(from.g + ((to.g - from.g) * factor));
    int blue = int(from.b + ((to.b - from.b) * factor));
    return Color(red, green, blue);
  }

  int random_byte(void) { return rand() % 256; }

  double drift_rate(void) {
    bool negative = rand() % 2 == 0;
    double abs_drift = 0.05 + (double((rand() % 10) / 100.0));
    return abs_drift * (negative ? -1.0 : 1.0);
  }

  void generate_random_palette_anchors(void) {
    cout << "Generating a random palette\n";
    anchor_colors[0] = Color(0, 0, 0);
    for (int x = 1; x < plasma_color_count_; x++) {
      anchor_colors[x] = Color(random_byte(), random_byte(), random_byte());
    }
  }

  void dump_palette(void) {
    cout << "Palette anchors (" << std::to_string(plasma_color_count_)
         << " colors):\n";
    for (int x = 0; x < plasma_color_count_; x++) {
      Color color = anchor_colors[x];
      cout << +color.r << ',' << +color.g << ',' << +color.b << '\n';
    }
  }

  void smooth_palette(void) {
    int palette_steps = PLASMA_PALETTE_SIZE / plasma_color_count_;
    double deg_smoothing_step = 180.0 / float(palette_steps);

    for (int anchor_idx = 0; anchor_idx < plasma_color_count_; anchor_idx++) {
      int start_idx = anchor_idx * palette_steps;
      int stop_idx = start_idx + palette_steps;
      int to_idx = (anchor_idx + 1) % plasma_color_count_;

      for (int i = start_idx; i < stop_idx; i++) {
        int j = i % palette_steps;
        double degs = j * deg_smoothing_step;
        double rads = (degs * 71) / 4068.0;
        double f = 1.0 - ((cos(rads) + 1) / 2.0);

        plasma_palette[i] = interpolate_palette(anchor_colors[anchor_idx],
                                                anchor_colors[to_idx], f);
      }
    }
  }

  int plasma_color_count_;
  time_t end_time_;
  std::string palette_filename_;
  int sleep_delay_;
};

// Conway's game of life
// Contributed by: Vliedel
class GameLife : public DemoRunner {
 public:
  GameLife(Canvas *m, int delay_ms = 500, bool torus = true)
      : DemoRunner(m), delay_ms_(delay_ms), torus_(torus) {
    width_ = canvas()->width();
    height_ = canvas()->height();

    // Allocate memory
    values_ = new int *[width_];
    for (int x = 0; x < width_; ++x) {
      values_[x] = new int[height_];
    }
    newValues_ = new int *[width_];
    for (int x = 0; x < width_; ++x) {
      newValues_[x] = new int[height_];
    }

    // Init values randomly
    srand(time(NULL));
    for (int x = 0; x < width_; ++x) {
      for (int y = 0; y < height_; ++y) {
        values_[x][y] = rand() % 2;
      }
    }
    r_ = rand() % 255;
    g_ = rand() % 255;
    b_ = rand() % 255;

    if (r_ < 150 && g_ < 150 && b_ < 150) {
      int c = rand() % 3;
      switch (c) {
        case 0:
          r_ = 200;
          break;
        case 1:
          g_ = 200;
          break;
        case 2:
          b_ = 200;
          break;
      }
    }
  }

  ~GameLife() {
    for (int x = 0; x < width_; ++x) {
      delete[] values_[x];
    }
    delete[] values_;
    for (int x = 0; x < width_; ++x) {
      delete[] newValues_[x];
    }
    delete[] newValues_;
  }

  void Run() override {
    while (!interrupt_received) {
      updateValues();

      for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
          if (values_[x][y])
            canvas()->SetPixel(x, y, r_, g_, b_);
          else
            canvas()->SetPixel(x, y, 0, 0, 0);
        }
      }
      usleep(delay_ms_ * 1000);  // ms
    }
  }

 private:
  int numAliveNeighbours(int x, int y) {
    int num = 0;
    if (torus_) {
      // Edges are connected (torus)
      num += values_[(x - 1 + width_) % width_][(y - 1 + height_) % height_];
      num += values_[(x - 1 + width_) % width_][y];
      num += values_[(x - 1 + width_) % width_][(y + 1) % height_];
      num += values_[(x + 1) % width_][(y - 1 + height_) % height_];
      num += values_[(x + 1) % width_][y];
      num += values_[(x + 1) % width_][(y + 1) % height_];
      num += values_[x][(y - 1 + height_) % height_];
      num += values_[x][(y + 1) % height_];
    } else {
      // Edges are not connected (no torus)
      if (x > 0) {
        if (y > 0) num += values_[x - 1][y - 1];
        if (y < height_ - 1) num += values_[x - 1][y + 1];
        num += values_[x - 1][y];
      }
      if (x < width_ - 1) {
        if (y > 0) num += values_[x + 1][y - 1];
        if (y < 31) num += values_[x + 1][y + 1];
        num += values_[x + 1][y];
      }
      if (y > 0) num += values_[x][y - 1];
      if (y < height_ - 1) num += values_[x][y + 1];
    }
    return num;
  }

  void updateValues() {
    // Copy values to newValues
    for (int x = 0; x < width_; ++x) {
      for (int y = 0; y < height_; ++y) {
        newValues_[x][y] = values_[x][y];
      }
    }
    // update newValues based on values
    for (int x = 0; x < width_; ++x) {
      for (int y = 0; y < height_; ++y) {
        int num = numAliveNeighbours(x, y);
        if (values_[x][y]) {
          // cell is alive
          if (num < 2 || num > 3) newValues_[x][y] = 0;
        } else {
          // cell is dead
          if (num == 3) newValues_[x][y] = 1;
        }
      }
    }
    // copy newValues to values
    for (int x = 0; x < width_; ++x) {
      for (int y = 0; y < height_; ++y) {
        values_[x][y] = newValues_[x][y];
      }
    }
  }

  int **values_;
  int **newValues_;
  int delay_ms_;
  int r_;
  int g_;
  int b_;
  int width_;
  int height_;
  bool torus_;
};

static int usage(const char *progname) {
  fprintf(stderr, "usage: %s <options> -D <demo-nr> [optional parameter]\n",
          progname);
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "\t-D <demo-nr>              : Always needs to be set\n");

  rgb_matrix::PrintMatrixFlags(stderr);

  fprintf(stderr, "Demos, choosen with -D\n");
  fprintf(stderr,
          "\t1  - Plasma (-p <# of palette 'anchor colors'> -r <run time "
          "seconds> -d <inter-frame sleep microseconds>)\n"
          "\t2  - Conway's game of line (-m <time-step-ms>)\n"
          "\t3  - Particles\n");
  fprintf(stderr,
          "Example:\n\t%s -D 0\n"
          "Scrolls the runtext until Ctrl-C is pressed\n",
          progname);
  return 1;
}

int main(int argc, char *argv[]) {
  srand(time(NULL));

  int demo = -1;
  int scroll_ms = 30;
  int plasma_color_count = 6;
  long int run_time = 0;
  std::string filename = "";
  int particles_model = 0;
  int sleep_delay = 5;

  RGBMatrix::Options matrix_options;
  rgb_matrix::RuntimeOptions runtime_opt;

  // These are the defaults when no command-line flags are given.
  matrix_options.rows = 32;
  matrix_options.cols = 64;
  matrix_options.chain_length = 1;
  matrix_options.parallel = 1;

  // First things first: extract the command line flags that contain
  // relevant matrix options.
  if (!ParseOptionsFromFlags(&argc, &argv, &matrix_options, &runtime_opt)) {
    return usage(argv[0]);
  }

  int opt;
  while ((opt = getopt(argc, argv, "d:D:f:r:P:c:p:b:m:LR:")) != -1) {
    switch (opt) {
      case 'd':
        sleep_delay = atoi(optarg);
        break;

      case 'D':
        demo = atoi(optarg);
        break;

      case 'm':
        scroll_ms = atoi(optarg);
        particles_model = atoi(optarg);
        break;

      case 'p':
        plasma_color_count = atoi(optarg);
        break;

      case 'r':
        run_time = atoi(optarg);
        break;

      case 'f':
        filename = strdup(optarg);
        break;

      default: /* '?' */
        return usage(argv[0]);
    }
  }

  if (demo < 0) {
    fprintf(stderr, TERM_ERR "Expected required option -D <demo>\n" TERM_NORM);
    return usage(argv[0]);
  }

  RGBMatrix *matrix = RGBMatrix::CreateFromOptions(matrix_options, runtime_opt);
  if (matrix == NULL) return 1;

  printf("Size: %dx%d. Hardware gpio mapping: %s\n", matrix->width(),
         matrix->height(), matrix_options.hardware_mapping);

  Canvas *canvas = matrix;

  std::time_t end_time = (run_time == 0) ? 0 : time(NULL) + run_time;

  // The DemoRunner objects are filling
  // the matrix continuously.
  DemoRunner *demo_runner = NULL;
  switch (demo) {
    case 1:
      demo_runner = new Plasma(canvas, plasma_color_count, end_time, filename,
                               sleep_delay);
      break;

    case 2:
      demo_runner = new GameLife(canvas, scroll_ms);
      break;

    case 3:
      demo_runner = new ParticleSwarm(canvas, particles_model);
      break;
  }

  if (demo_runner == NULL) return usage(argv[0]);

  // Set up an interrupt handler to be able to stop animations while they go
  // on. Each demo tests for while (!interrupt_received) {},
  // so they exit as soon as they get a signal.
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);
  signal(SIGUSR1, SIGUSR1Handler);
  signal(SIGUSR2, SIGUSR2Handler);

  // Now, run our particular demo; it will exit when it sees interrupt_received.
  demo_runner->Run();

  delete demo_runner;
  delete canvas;

  printf("Exiting.\n");
  return 0;
}
