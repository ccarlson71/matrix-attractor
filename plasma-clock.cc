// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Example of a clock. This is very similar to the text-example,
// except that it shows the time :)
//
// This code is public domain
// (but note, that the led-matrix library this depends on is GPL v2)

#include <rpi-rgb-led-matrix/graphics.h>
#include <rpi-rgb-led-matrix/led-matrix.h>

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using std::cout;

using namespace rgb_matrix;

volatile bool interrupt_received = false;
static void InterruptHandler(int signo) { interrupt_received = true; }

double drift_rate(void) {
  bool negative = rand() % 2 == 0;
  double abs_drift = 0.05 + (double((rand() % 10) / 100.0));
  return abs_drift * (negative ? -1.0 : 1.0);
}

///////////////////////////////
// Plasma-related stuff
#define PLASMA_PALETTE_SIZE 240
#define MAX_PLASMA_COLORS 6
#define PLASMA_STEP_VALUE 0.07

double sqrt_lut[5120];
Color plasma_palette[PLASMA_PALETTE_SIZE];
std::string palette_filename = "";
int plasma_color_count = MAX_PLASMA_COLORS;
Color anchor_colors[MAX_PLASMA_COLORS];
const double plasma_randomizing_param = 10 + ((rand() % 750) / 50.0);
const int width = 64;
const int height = 32;
double x_drift = 0.0;
double y_drift = 0.0;
const double x_drift_rate = drift_rate();
const double y_drift_rate = drift_rate();

int sleep_ms = 40;
///////////////////////////////

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

int random_byte(void) { return rand() % 256; }

// Calculates an intermediate color, a given % distance between them
Color interpolate_palette(Color from, Color to, double factor) {
  int red = int(from.r + ((to.r - from.r) * factor));
  int green = int(from.g + ((to.g - from.g) * factor));
  int blue = int(from.b + ((to.b - from.b) * factor));
  return Color(red, green, blue);
}

// Display RGB colors comprising each palette anchor
void dump_palette() {
  cout << "Palette anchors (" << std::to_string(plasma_color_count)
       << " colors):\n";
  for (int x = 0; x < plasma_color_count; x++) {
    Color color = anchor_colors[x];
    cout << +color.r << ',' << +color.g << ',' << +color.b << '\n';
  }
}

// With anchor colors determined, smoothly blend the full palette between them
void smooth_palette(void) {
  int palette_steps = PLASMA_PALETTE_SIZE / plasma_color_count;
  double deg_smoothing_step = 180.0 / float(palette_steps);

  for (int anchor_idx = 0; anchor_idx < plasma_color_count; anchor_idx++) {
    int start_idx = anchor_idx * palette_steps;
    int stop_idx = start_idx + palette_steps;
    int to_idx = (anchor_idx + 1) % plasma_color_count;

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

void generate_palette() {
  bool generate_random = true;

  std::ifstream infile(palette_filename);
  if (infile) {
    cout << "Loading palette from file: " << palette_filename << '\n';

    plasma_color_count = 0;
    std::string line;
    while (std::getline(infile, line)) {
      Color newColor;
      if (line[0] != '#') { // Comments? I dunno
        if (parseColor(&newColor, line)) {
          anchor_colors[plasma_color_count] = newColor;
          plasma_color_count++;
          if (plasma_color_count == MAX_PLASMA_COLORS) {
            break;
          }
        }
      }
    }

    if (plasma_color_count > 1) {
      generate_random = false;
    } else {
      cout << "Insufficient palette entries found.\n";
      generate_random = true;
      plasma_color_count = MAX_PLASMA_COLORS;
    }
  }

  if (generate_random) {
    cout << "Generating a random palette\n";
    anchor_colors[0] = Color(0, 0, 0);
    for (int x = 1; x < plasma_color_count; x++) {
      anchor_colors[x] = Color(random_byte(), random_byte(), random_byte());
    }
  }

  dump_palette();
  smooth_palette();
}

int text_width(const char *text, rgb_matrix::Font &font) {
  int width = 0;

  while (*text)
    width += font.CharacterWidth(*text++);

  return width;
}

void draw_plasma(FrameCanvas *offscreen, double current_value) {
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
      Color output_color = plasma_palette[scaled_value % PLASMA_PALETTE_SIZE];
      offscreen->SetPixel(x, y, output_color.r, output_color.g, output_color.b);
    }
  }
}

static int usage(const char *progname) {
  fprintf(stderr, "usage: %s [options]\n", progname);
  fprintf(stderr, "Reads text from stdin and displays it. "
                  "Empty string: clear screen\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "\t-f <font-file>          : Use given font.\n"
                  "\t-p <palette-file>       : Palette filename.\n"
                  "\t-s <sleep-milliseconds> : Delay between frames in milliseconds (default 40).\n"
                  "\n");
  rgb_matrix::PrintMatrixFlags(stderr);
  return 1;
}

int main(int argc, char *argv[]) {
  double current_plasma_value = float(rand() % 10000);

  RGBMatrix::Options matrix_options;
  rgb_matrix::RuntimeOptions runtime_opt;
  if (!rgb_matrix::ParseOptionsFromFlags(&argc, &argv, &matrix_options,
                                         &runtime_opt)) {
    return usage(argv[0]);
  }

  std::string format_line = "%H:%M";
  Color clock_color(255, 255, 255);
  Color clock_outline_color(0, 0, 0);

  const char *bdf_font_file = NULL;

  int opt;
  while ((opt = getopt(argc, argv, "f:p:s:")) != -1) {
    switch (opt) {
    case 'f':
      bdf_font_file = strdup(optarg);
      break;
    case 'p':
      palette_filename = strdup(optarg);
      break;
    case 's':
      sleep_ms = atoi(optarg);
      break;
    default:
      return usage(argv[0]);
    }
  }

  generate_palette();

  // Generate LUTs
  for (int x = 0; x < 64; x++) {
    for (int y = 0; y < 32; y++) {
      int idx = (y * 64) + x;
      double dy = double(y) / 2.0;
      sqrt_lut[idx] = sqrt((x * x) + (dy * dy)) / 8.0;
    }
  }

  if (bdf_font_file == NULL) {
    fprintf(stderr, "Need to specify BDF font-file with -f\n");
    return usage(argv[0]);
  }

  /*
   * Load font. This needs to be a filename with a bdf bitmap font.
   */
  rgb_matrix::Font font;
  if (!font.LoadFont(bdf_font_file)) {
    fprintf(stderr, "Couldn't load font '%s'\n", bdf_font_file);
    return 1;
  }
  rgb_matrix::Font *outline_font = NULL;
  outline_font = font.CreateOutlineFont();

  RGBMatrix *matrix = RGBMatrix::CreateFromOptions(matrix_options, runtime_opt);
  if (matrix == NULL)
    return 1;

  int y = int((32 - font.height()) / 2);

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();

  char text_buffer[256];

  struct tm tm;
  time_t now;

  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);

  while (!interrupt_received) {
    draw_plasma(offscreen, current_plasma_value);
    current_plasma_value += PLASMA_STEP_VALUE;

    now = time(NULL);
    localtime_r(&now, &tm);

    strftime(text_buffer, sizeof(text_buffer), format_line.c_str(), &tm);

    // Dynamically determine the offset so text is centered
    int x = int((64 - text_width(text_buffer, font)) / 2);

    rgb_matrix::DrawText(offscreen, *outline_font, x - 1, y + font.baseline(),
                         clock_outline_color, NULL, text_buffer, -2);
    rgb_matrix::DrawText(offscreen, font, x, y + font.baseline(), clock_color,
                         NULL, text_buffer, 0);

    // Atomic swap with double buffer
    offscreen = matrix->SwapOnVSync(offscreen);
    usleep(1000 * sleep_ms);

    // next_time.tv_sec += 1;
  }

  // Finished. Shut down the RGB matrix.
  delete matrix;

  write(STDOUT_FILENO, "\n", 1); // Create a fresh new line after ^C on screen
  return 0;
}

