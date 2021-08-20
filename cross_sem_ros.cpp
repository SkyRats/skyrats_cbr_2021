#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
#include <chrono>
using namespace std::chrono;

#define ANGLE_THRESH 0.01
#define TESTE_VEL 0
#define PI 3.14159
#define KERNEL_THRESH_MAX 1.2
#define KERNEL_THRESH_MIN 0.9

const int max_value_H = 360 / 2;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

#define vp vector<Point>
#define vpf vector<Point2f>

// Set true for debugging purposes, showing internals of algorithm
#define DEBUG true

int NUM_GAUSS_BLUR = 0;

// Sorts points based on y coordinate
struct comparison
{
  bool operator()(Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y); }
} comparing;

class CrossDetector
{
private:
  vpf edge_pts = {Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)};
  void order_points();
  Mat four_points_transform(Mat image);
  float angle(Point2f v1, Point2f v2, Point2f relative = Point2f(0, 0));
  bool angle_check(vpf pts);

public:
  Mat warped;
  CrossDetector();
  Mat detect(Mat frame);
};

CrossDetector::CrossDetector() {}

/* Order points in edge_pts so that the first exit is the top-left, the second 
top-right, the third bottom-right, and the fourth bottom-left */
void CrossDetector::order_points()
{

  sort(this->edge_pts.begin(), this->edge_pts.end(), comparing);
  Point2f p1, p2;

  /* Switch the position of the first and second points
        if the second is to the right of the second */
  if (this->edge_pts[0].x > this->edge_pts[1].x)
  {
    p1 = this->edge_pts[1];
    this->edge_pts[1] = this->edge_pts[0];
    this->edge_pts[0] = p1;
  }

  /* Same as above for the third and fourth */
  if (this->edge_pts[2].x < this->edge_pts[3].x)
  {
    p2 = this->edge_pts[3];
    this->edge_pts[3] = this->edge_pts[2];
    this->edge_pts[2] = p2;
  }
}

/* Takes an image as argument and returns warped perspective, moving edge_pts to
the edge of the frame */
Mat CrossDetector::four_points_transform(Mat image)
{

  order_points();

  Point tl = this->edge_pts[0];
  Point tr = this->edge_pts[1];
  Point br = this->edge_pts[2];
  Point bl = this->edge_pts[3];

  float widthA = sqrt(abs(pow(br.x - bl.x, 2.0) - pow(br.y - bl.y, 2.0)));
  float widthB = sqrt(abs(pow(tr.x - tl.x, 2.0) - pow(tr.y - tl.y, 2.0)));
  // Requires explicit reference due to cv::max
  float maxWidth = std::max(widthA, widthB);

  float heightA = sqrt(abs(pow(br.y - tr.y, 2.0) - pow(br.x - tr.x, 2.0)));
  float heightB = sqrt(abs(pow(tl.y - bl.y, 2.0) - pow(tl.x - bl.x, 2.0)));

  float maxHeight = std::max(heightA, heightB);

  vpf dst = {
      Point2f(0.0, 0.0),
      Point2f(maxWidth, 0.0),
      Point2f(maxWidth, maxHeight),
      Point2f(0.0, maxHeight)};

  Mat M = getPerspectiveTransform(this->edge_pts, dst);

  warpPerspective(image, this->warped, M, Size(maxWidth, maxHeight));

  return M;
}

// Determines angle between vectors 'v1' and 'v2' using 'relative' as origin
float CrossDetector::angle(Point2f v1, Point2f v2, Point2f relative)
{

  float Vlenght1, Vlenght2; // Length of v1 and v2

  Vlenght1 = sqrt((v1.x - relative.x) * (v1.x - relative.x) + (v1.y - relative.y) * (v1.y - relative.y));
  Vlenght2 = sqrt((v2.x - relative.x) * (v2.x - relative.x) + (v2.y - relative.y) * (v2.y - relative.y));
  // Takes dot product and divides by vector lengths to get cos of the angle
  float a = ((v1.x - relative.x) * (v2.y - relative.y) - (v1.y - relative.y) * (v2.x - relative.x)) / (Vlenght1 * Vlenght2);

  return asin(a);
}
/* Checks if all sides of a 12 sided shape 'pts' are perpendicular and have the right orientation, 
    using ANGLE_THRESH */
bool CrossDetector::angle_check(vpf pts)
{

  int bitmasks[8] = {3510, 2925, 1755, 585, 1170, 2340};
  int current_bm = 0;

  float a = 0;
  for (int i = 0; i < 12; i++)
  {
    // General term
    a = angle(pts[(12 + i + 1) % 12], pts[(12 + i - 1) % 12], pts[(12 + i) % 12]);
    if (abs(abs(a) - PI / 2) < ANGLE_THRESH)
      return false;
    else
      current_bm = current_bm | (a > 0) << i;
  }

  /* cout << current_bm << endl; */
  for (int bm : bitmasks)
  {
    if (current_bm == bm)
    {
      if (!TESTE_VEL)
        cout << "Cruz detectada" << endl;
      return true;
    }
  }
  return false;
}

// Takes an image 'frame' and detects whether it contains the Cross
Mat CrossDetector::detect(Mat frame)
{
  auto start = high_resolution_clock::now();
  Mat frame2 = frame;
  cvtColor(frame, frame, COLOR_BGR2HSV);
  // Blur and threshold remove noise from image

  for (int test = 0; test < NUM_GAUSS_BLUR; test++)
  {
    GaussianBlur(frame, frame, Size(5, 5), 0);
  }
  inRange(frame, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame);
  cv::adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 9, 20.0);
  cv::adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 0.0);

  vector<vp> contour;
  findContours(frame, contour, RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  for (vp cnt : contour)
  {

    int peri = arcLength(cnt, true);
    if (peri >= 0)
    {
      vpf approx;
      approxPolyDP(cnt, approx, 0.02 * peri, true);

      if (approx.size() == 12)
      {

        Rect2f bounds = boundingRect(approx);

        float a1 = angle(approx[0] - approx[1], Point2f(0, 1));
        float a2 = angle(approx[1] - approx[2], Point2f(0, 1));

        /* If the sides of the H are very close to parallel to its bounds,
                    use the bounding rect vertices for warp */
        if (a1 < 0.1 || a2 < 0.1 || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1)
        {

          this->edge_pts = {
              Point2f(bounds.x, bounds.y),
              Point2f(bounds.x + bounds.width, bounds.y),
              Point2f(bounds.x, bounds.y + bounds.height),
              Point2f(bounds.x + bounds.width, bounds.y + bounds.height)};

          /* If they are far, use the vertices that are closest to the bounding
                    rect sides */
        }
        else
        {

          for (Point2f v : approx)
          {

            // Close on left side of bound
            if (abs(v.x - bounds.x) <= 1)
              this->edge_pts[0] = v;
            // On right side
            else if (abs(v.x - (bounds.x + bounds.width)) <= 1)
              this->edge_pts[1] = v;

            // On top
            else if (abs(v.y - bounds.y) <= 1)
              this->edge_pts[2] = v;
            //On bottom
            else if (abs(v.y - (bounds.y + bounds.height)) <= 1)
              this->edge_pts[3] = v;
          }
        }

        Mat perspective = four_points_transform(frame);
        vpf transformed;
        perspectiveTransform(approx, transformed, perspective);

        if (DEBUG)
        {
          imshow("warped", frame);
        }

        if (angle_check(approx))
        {

          if (!TESTE_VEL)
          {
            cout << "Cruz detectado" << endl;
            rectangle(frame2, bounds, (0, 255, 0));
            imshow("Lines", frame2);
          }
          auto stop = high_resolution_clock::now();
          auto duration = duration_cast<microseconds>(stop - start);
          if (!TESTE_VEL)
            cout << "Time from getting frame to detecting Cross: ";

          cout << duration.count() << endl;
        }
        else if (!TESTE_VEL)
          cout << endl;
      }
    }
  }
  return frame2;
}

static void on_low_H_thresh_trackbar(int, void *)
{
  low_H = min(high_H - 1, low_H);
  setTrackbarPos("Low H", "display", low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
  high_H = max(high_H, low_H + 1);
  setTrackbarPos("High H", "display", high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
  low_S = min(high_S - 1, low_S);
  setTrackbarPos("Low S", "display", low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
  high_S = max(high_S, low_S + 1);
  setTrackbarPos("High S", "display", high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
  low_V = min(high_V - 1, low_V);
  setTrackbarPos("Low V", "display", low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
  high_V = max(high_V, low_V + 1);
  setTrackbarPos("High V", "display", high_V);
}

// For testing
int main()
{
  NUM_GAUSS_BLUR = 1;
  if (TESTE_VEL + 1)
    cin >> NUM_GAUSS_BLUR;
  Mat frame;

  namedWindow("display");
  // Trackbars to set thresholds for HSV values
  createTrackbar("Low H", "display", &low_H, max_value_H, on_low_H_thresh_trackbar);
  createTrackbar("High H", "display", &high_H, max_value_H, on_high_H_thresh_trackbar);
  createTrackbar("Low S", "display", &low_S, max_value, on_low_S_thresh_trackbar);
  createTrackbar("High S", "display", &high_S, max_value, on_high_S_thresh_trackbar);
  createTrackbar("Low V", "display", &low_V, max_value, on_low_V_thresh_trackbar);
  createTrackbar("High V", "display", &high_V, max_value, on_high_V_thresh_trackbar);

  VideoCapture video(0);
  CrossDetector *detector = new CrossDetector();
  video >> frame;
  while (true)
  {
    imshow("display", detector->detect(frame));
    if (waitKey(30) == 27)
      break;
    video >> frame;
  }
}
