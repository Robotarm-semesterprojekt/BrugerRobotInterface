#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>


using namespace ur_rtde;

// Define a type for a point in our theoretical 2D space, using integer coordinates
typedef std::pair<int, int> Point;
// Define a type for a point that the robot can be feed, using double bacuse the robot takes the position in meters
typedef std::pair<double, double> PointR;


const double pi = 3.1416;

std::vector<PointR> loadPoints(const std::string& filename)
{
    std::vector<PointR> points;

    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Could not open file\n";
        return points;
    }

    std::string line;

    // Skip header
    std::getline(file, line);

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);

        std::string indexStr;
        std::string xStr;
        std::string yStr;

        if (!std::getline(ss, indexStr, ','))
            continue;

        if (!std::getline(ss, xStr, ','))
            continue;

        if (!std::getline(ss, yStr, ','))
            continue;

        try
        {
            double x = 0.001*(std::stod(xStr));
            double y = 0.001*(std::stod(yStr));

            points.emplace_back(x, y);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Bad line: " << line << '\n';
        }
    }

    return points;
}

//Transform a point from the 2D space to the robot's coordinate system using an affine transformation
PointR transformPointAffine(const Point& p) {
    double x = p.first;
    double y = p.second;

    double xp = -0.0006017 * x + 0.000684 * y + 0.11;
    double yp =  0.001048  * x + 0.0003646 * y - 0.645;

    return {xp, yp};
}

std::vector<double> directionToRotvec(const PointR& dir)
{
    double dx = dir.first;
    double dy = dir.second;

    
    // Compute yaw angle from 2D direction
    

    double yaw = std::atan2(dy, dx) + M_PI_2;

    
    // Rotate around Z (twist)
    

    Eigen::AngleAxisd yaw_rot(
        yaw,
        Eigen::Vector3d::UnitZ()
    );

    
    // Point tool downward
    //
    // Change UnitX() to UnitY()
    // if your TCP is rotated differently
    

    Eigen::AngleAxisd down_rot(
        M_PI,
        Eigen::Vector3d::UnitX()
    );
    
    
    // Final rotation matrix
    

    Eigen::Matrix3d R =
        yaw_rot.toRotationMatrix() *
        down_rot.toRotationMatrix();

    
    // Convert to axis-angle rotvec
    

    Eigen::AngleAxisd aa(R);

    Eigen::Vector3d rotvec =
        aa.axis() * aa.angle();

    
    // Return rx ry rz

    return
    {
        rotvec.x(),
        rotvec.y(),
        rotvec.z()
    };
}

std::vector<PointR> generateXYrVectors(const std::vector<PointR>& p, double radAngle) {
    std::vector<PointR> result;

    if (p.size() < 2)
        return result;

    result.reserve(p.size());

    for (size_t i = 0; i < p.size(); ++i)
    {
        double vx, vy;

        // First point
        if (i == 0)
        {
            vx = p[i + 1].first  - p[i].first;
            vy = p[i + 1].second - p[i].second;
        }
        // Last point
        else if (i == p.size() - 1)
        {
            vx = p[i].first  - p[i - 1].first;
            vy = p[i].second - p[i - 1].second;
        }
        // Middle points
        else
        {
            vx = p[i + 1].first  - p[i - 1].first;
            vy = p[i + 1].second - p[i - 1].second;
        }

        // Rotate vector
        double rx = (vx * std::cos(radAngle)) -
                    (vy * std::sin(radAngle));

        double ry = (vx * std::sin(radAngle)) +
                    (vy * std::cos(radAngle));

        
        result.emplace_back(rx, ry);
    }

    return result;
}

int setup_serial(const char* port) {

    int serial_port = open(port, O_RDWR);

    if (serial_port < 0) {
        std::cerr << "Failed to open " << port << std::endl;
        return -1;
    }

    termios tty{};

    tcgetattr(serial_port, &tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // Raw mode
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tcsetattr(serial_port, TCSANOW, &tty);

    return serial_port;
}

bool send_and_receive(int serial, const char* name, char cmd) {

    tcflush(serial, TCIFLUSH);

    write(serial, &cmd, 1);

    std::cout << "Sent to " << name << ": " << cmd << std::endl;

    usleep(50000); // optional 50ms delay

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));

    int total = 0;

    while (true) {

        int n = read(serial,
                     buffer + total,
                     sizeof(buffer) - total - 1);

        if (n <= 0)
            break;

        total += n;

        // Stop on newline
        if (buffer[total - 1] == '\n')
            break;
    }

    if (total > 0) {
        buffer[total] = '\0';
        std::cout << name << " says: " << buffer << std::endl;
        if (std::strstr(buffer, "FEJL: Der mangler brikker!") != nullptr) {
            return true;
        };
    }
    return false;
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



int main(int argc, char* argv[]) {

    // Get current position
    RTDEReceiveInterface rtde_receive("192.168.1.11");
    RTDEControlInterface rtde_control("192.168.1.11");


    // Define the dimensions of the 2D space
    const int WIDTH = 223;
    const int HEIGHT = 288;


    //Define the radian angle that the robot should place the dominos at relative to the line.
    double angle = pi/2;

    //Deffine the hight flange will be mooving at height it sinks to when placing
    const double operationHeight = 0.1;
    const double placeHeight  = 0.0097;

    // Variable to determine the distance between consecutive points on the line
    double point_distance = 1.0; // Default distance; can be changed

    //Vector the rrobot uses to find correct rotation for placement;
    const PointR refVector = {0.0,-1.0};

    // Pico dispenser
    const char* pico_port = "/dev/ttyACM0";

    // Second serial device
    const char* second_port = "/dev/ttyACM1";

    int pico_serial = setup_serial(pico_port);
    int second_serial = setup_serial(second_port);

    if (pico_serial < 0 || second_serial < 0) {
        return 1;
    }

    std::cout << "Press:\n";
    std::cout << "1 = Send to Pico\n";
    std::cout << "2 = Send to ttyACM1\n";
    std::cout << "q = Quit\n";


    
    while(true){  
        
        char programMode;

        // Prompt user for the distance between points
        std::cout << "Please select program mode:" << std::endl <<
        "Press '0' to draw a line between two points" << std::endl <<
        "Press '1' to follow the line in dominos.csv" << std::endl;
        std::cin >> programMode;

        std::vector<Point> points;
        std::vector<PointR> transformed_points;

        if (programMode == '0') {

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
            points = get_line_points(x1, y1, x2, y2, point_distance);
            transformed_points = transformPointsAffine(points);
        }
        else if (programMode == '1') {
            transformed_points = loadPoints("dominoes.csv");
        }
        
        for (const auto& p : transformed_points)
        {
            std::cout << p.first << ", " << p.second << '\n';
        }

        auto pointRotations = generateXYrVectors(transformed_points,angle);

        std::cout << "transformed_points: " << transformed_points.size() << '\n';
        std::cout << "pointRotations: " << pointRotations.size() << '\n';

        // Output the number of points and the list of points
        std::cout << "Line points (" << transformed_points.size() << " points):" << std::endl;
        for (int i = 0; i < transformed_points.size(); ++i) {
            std::cout << "(" << transformed_points[i].first << ", " << transformed_points[i].second << ") "<<
            "Rotation: ("<< pointRotations[i].first << ", " << pointRotations[i].second << ")"<<
            std::endl;
        if (i < points.size() - 1) std::cout << ", ";
        }
        
        std::cout << std::endl;



        //Move arm to each point and then linearly go down at said point
        for (std::size_t i = 0; i < transformed_points.size(); ++i) {
            double x = transformed_points[i].first;
            double y = transformed_points[i].second;

            std::vector<double> rotationVector =  directionToRotvec(pointRotations[i]);
            
            

            char readyNext = '0';
            
            //go dispenser
            rtde_control.moveL_FK({-1.187, -1.257, -1.92, -1.553, 1.623, 0},0.2,0.5);

            while (send_and_receive(second_serial, "ttyACM1", '2')){
                char loaded = '0';
                std::cout << "Press 1 when new dominos have been loaded" << std::endl;
                if (loaded != '1'){
                    std::cin >> loaded;
                }
            }
            
            send_and_receive(pico_serial, "Pico", '0');


            rtde_control.moveJ_IK({-0.287, -0.217, 0.054, 2.699, -1.565, -2.521},1.0);
            rtde_control.moveL({-0.334, -0.149, 0.03, 2.584, -1.685, -2.509},0.2);

            
            rtde_control.moveL({-0.345, -0.096, 0.03, 2.584, -1.685, -2.509},0.05);
            rtde_control.moveL({-0.340, -0.096, 0.03, 2.584, -1.685, -2.509},0.005);

            
            std::cout << "Grippuh Cluh" << std::endl;
            send_and_receive(pico_serial, "Pico", '1');

            rtde_control.moveL({-0.340, -0.096, 0.05, 2.584, -1.685, -2.509},0.005);
            rtde_control.moveL({-0.334, -0.149, 0.05, 2.584, -1.685, -2.509},0.05);

            
            
            rtde_control.moveJ_IK({-0.203, -0.312, 0.201, 2.46, -1.955, -1.492},0.3);
            


            rtde_control.moveJ({-1.187, -1.257, -1.92, -1.553, 1.623, 0},0.5);

            rtde_control.moveJ_IK({x, y, operationHeight, rotationVector[0], rotationVector[1], rotationVector[2]},1.0);

            

            rtde_control.moveL({x, y, placeHeight, rotationVector[0], rotationVector[1], rotationVector[2]},0.1,0.1);

            std::cout << "Grippuh Opuh" << std::endl;
            send_and_receive(pico_serial, "Pico", '0');

            rtde_control.moveL({x, y, operationHeight, rotationVector[0], rotationVector[1], rotationVector[2]},0.1,0.1);

            


            /*
            while (readyNext == '0') {
                std::cout << "Press 1 when ready for next point" << std::endl;
                if (readyNext != '1'){
                    std::cin >> readyNext;

            }
            }
            */
        }

        char knock = '0';
        std::cout << "Press 1 to colaps the Dominos" << std::endl;
        std::cin >> knock;
        if (knock == '1') {
            double x = (2*transformed_points[0].first) - transformed_points[1].first;
            double y = (2*transformed_points[0].second) - transformed_points[1].second;

            
            PointR dir = {transformed_points[1].first  - transformed_points[0].first,transformed_points[1].second - transformed_points[0].second};
            std::vector<double> rotationVector =  directionToRotvec(dir);
            

            rtde_control.moveJ_IK({x, y, operationHeight, rotationVector[0], rotationVector[1], rotationVector[2]},0.2);
            rtde_control.moveL({x, y, placeHeight, rotationVector[0], rotationVector[1], rotationVector[2]},0.1,0.1);

            x = transformed_points[0].first;
            y = transformed_points[0].second;

            rtde_control.moveL({x, y, placeHeight, rotationVector[0], rotationVector[1], rotationVector[2]},0.1,0.1);
            rtde_control.moveL({x, y, operationHeight, rotationVector[0], rotationVector[1], rotationVector[2]},0.1,0.1);
        }
         

    }
    

    rtde_control.stopScript();
    close(pico_serial);
    close(second_serial);
    
    return 0;
}
