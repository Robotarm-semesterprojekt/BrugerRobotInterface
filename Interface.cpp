#include <iostream>
#include <vector>
#include <utility>
#include <cmath>

// Define a type for a point in our theoretical 2D space, using integer coordinates
typedef std::pair<int, int> Point;
// Define a type for a point that the robot can be feed, using double bacuse the robot takes the position in meters
typedef std::pair<double, double> PointR;

//Transform a point from the 2D space to the robot's coordinate system using an affine transformation
PointR transformPointAffine(const Point& p) {
    double x = p.first;
    double y = p.second;

    double xp = -0.0006017 * x + 0.000684 * y + 0.11;
    double yp =  0.001048  * x + 0.0003646 * y - 0.645;

    return {xp, yp};
}


//Function to aplly the tranfrom to each point in the vector of points
std::vector<PointR> transformPointsAffine(const std::vector<Point>& points) {
    std::vector<PointR> result;
    result.reserve(points.size());

    for (const auto& p : points) {
        result.push_back(transformPointAffine(p));
    }

    return result;
}


// Function to compute points on a line between two points with specified distance between points
std::vector<Point> get_line_points(int x1, int y1, int x2, int y2, double distance) {
    std::vector<Point> points;

    // Calculate the differences in x and y coordinates
    double dx = x2 - x1;
    double dy = y2 - y1;

    // Calculate the Euclidean distance (length) of the line
    double length = std::sqrt(dx * dx + dy * dy);

    // If the points are the same, return the single point
    if (length == 0) {
        points.push_back({x1, y1});
        return points;
    }

    // Calculate the number of steps needed based on the desired distance between points
    double steps = length / distance;

    // Calculate the step size in x and y directions
    double step_x = dx / steps;
    double step_y = dy / steps;

    // Start from the first point
    double x = x1;
    double y = y1;

    // Generate points along the line at the specified intervals
    for (int i = 0; i <= static_cast<int>(steps); ++i) {
        // Round to nearest integer coordinates and add to the list
        points.push_back({static_cast<int>(std::round(x)), static_cast<int>(std::round(y))});
        x += step_x;
        y += step_y;
    }

    return points;
}

int main() {
    // Define the dimensions of the 2D space
    const int WIDTH = 223;
    const int HEIGHT = 288;

    // Variable to determine the distance between consecutive points on the line
    double point_distance = 1.0; // Default distance; can be changed

    // Prompt user for the distance between points
    std::cout << "Enter the distance between points on the line (e.g., 1.0 for unit distance): ";
    std::cin >> point_distance;

    // Validate that distance is positive
    if (point_distance <= 0) {
        std::cout << "Error: Distance must be positive." << std::endl;
        return 1;
    }

    // Prompt for the first point
    int x1, y1;
    std::cout << "Enter first point (x y), where 0 <= x < " << WIDTH << ", 0 <= y < " << HEIGHT << ": ";
    std::cin >> x1 >> y1;

    // Prompt for the second point
    int x2, y2;
    std::cout << "Enter second point (x y), where 0 <= x < " << WIDTH << ", 0 <= y < " << HEIGHT << ": ";
    std::cin >> x2 >> y2;

    // Check that points are within bounds
    if (x1 < 0 || x1 >= WIDTH || y1 < 0 || y1 >= HEIGHT ||
        x2 < 0 || x2 >= WIDTH || y2 < 0 || y2 >= HEIGHT) {
        std::cout << "Error: Points are out of bounds." << std::endl;
        return 1;
    }

    // Compute the line points using the specified distance
    auto points = get_line_points(x1, y1, x2, y2, point_distance);
    auto transformed_points = transformPointsAffine(points);


    // Output the number of points and the list of points
    std::cout << "Line points (" << transformed_points.size() << " points):" << std::endl;
    for (std::size_t i = 0; i < transformed_points.size(); ++i) {
        std::cout << "(" << points[i].first << ", " << points[i].second << ") -> (" << transformed_points[i].first << ", " << transformed_points[i].second << ")"<<std::endl;
        if (i < points.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    return 0;
}