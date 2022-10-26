#include <iostream>
#include <opencv2/opencv.hpp>
#include "source/AStar.hpp"

using namespace cv;
using namespace std;
RNG rng(12345);

#define N_TILES_X 80
#define N_TILES_Y 50
#define THRESHOLD_WHITE_PIXELS 0.01
#define POINTS_PER_LINE 10
#define KERNEL_SIZE 6

struct line_connection {
    float cost;
    AStar::CoordinateList line_points;
    AStar::Vec2i initial_point;
    AStar::Vec2i final_point;
};

bool check_line_in_tile(cv::Mat& tile) 
{
    uint16_t total_pixels = tile.rows * tile.cols;
    uint16_t sum = 0;
    for(int i=0; i < tile.rows; i++) {
        const uint8_t* row_i = tile.ptr<uint8_t>(i);
        for(int j=0; j < tile.cols; j++) {
            if(row_i[j] == 255) {
                sum++;
                if((float)sum/total_pixels > THRESHOLD_WHITE_PIXELS) {
                    return true;
                }
            }
        }
    }
    return false;
}

float calculate_slope(const AStar::Vec2i& pt_i, const AStar::Vec2i& pt_f) 
{
    float angle = atan2(pt_f.y-pt_i.y, pt_f.x-pt_i.x);
    return angle;
}

float calculate_orientation_error(float& angle_1, float& angle_2)
{
    return std::abs(angle_2 - angle_1);
}

AStar::CoordinateList extract_line_points(AStar::CoordinateList& line_points) {
    unsigned int num_line_points = line_points.size();
    AStar::CoordinateList extracted_points;
    extracted_points.push_back(line_points[num_line_points-1]);
    unsigned int ref = num_line_points - (num_line_points/POINTS_PER_LINE);
    unsigned int ref_dec = num_line_points/POINTS_PER_LINE;
    for(uint8_t i=1; i < POINTS_PER_LINE-1; i++) {
        extracted_points.push_back(line_points[ref]);
        ref -= ref_dec;
    }
    extracted_points.push_back(line_points[0]);
    return extracted_points;
}

float calculate_line_cost(AStar::CoordinateList extracted_points) {
    // Calculate total consecutive orientation errors
    float angle_1, angle_2;
    float cost=0;

    for(int i=0; i < extracted_points.size()-4; i=i+1) {
        angle_1 = calculate_slope(extracted_points[i], extracted_points[i+2]);
        angle_2 = calculate_slope(extracted_points[i+2], extracted_points[i+4]);
        cost += calculate_orientation_error(angle_1, angle_2);
    }
        angle_1 = calculate_slope(extracted_points[0], extracted_points[3]);
        angle_2 = calculate_slope(extracted_points[extracted_points.size()-4], extracted_points[extracted_points.size()-1]);
        cost += 0.8*calculate_orientation_error(angle_1, angle_2);
    return cost;
}

cv::Mat create_occupancy_grid_map(cv::Mat& cropped_image, AStar::Generator& generator, uint8_t n_tiles_x, uint8_t n_tiles_y) {
    cv::Mat occupancy_grid_map;
    uint16_t occupancy_map_row=0, occupancy_map_col=0;
    uint16_t n_pixels_x, n_pixels_y;
    bool exists_line;

    // Initialize occupancy grid map
    occupancy_grid_map = cv::Mat::zeros(N_TILES_Y, N_TILES_X, CV_8UC1);

    // Fill map 
    for(int i=0; i < cropped_image.rows;) {
        uint8_t* occupancy_map_row_i = occupancy_grid_map.ptr<uint8_t>(occupancy_map_row);
        occupancy_map_col = 0;
        for(int j=0; j < cropped_image.cols;) {
            n_pixels_x = (uint16_t)(((float)cropped_image.cols / N_TILES_X)*(occupancy_map_col+1)) - j;
            n_pixels_y = (uint16_t)(((float)cropped_image.rows / N_TILES_Y)*(occupancy_map_row+1)) - i;
            if(occupancy_map_col == N_TILES_X-1) n_pixels_x = cropped_image.cols-j;
            if(occupancy_map_row == N_TILES_Y-1) n_pixels_y = cropped_image.rows-i;
            cv::Mat tile = cropped_image(cv::Rect(j,i, n_pixels_x, n_pixels_y));
            exists_line = check_line_in_tile(tile);
            if(!exists_line) {
                generator.addCollision({occupancy_map_col, occupancy_map_row});
                occupancy_map_row_i[occupancy_map_col] = 0;
            }
            else {
                occupancy_map_row_i[occupancy_map_col] = 255;
            }
            occupancy_map_col++;
            j=j+n_pixels_x;
        }
        occupancy_map_row++;
        i=i+n_pixels_y;
    }
    return occupancy_grid_map;
}

bool compare_costs(const line_connection& a, const line_connection& b)
{
    return a.cost < b.cost;
}

int convert_coordinates_to_occupancy_map(const AStar::CoordinateList& original_input_pos, const AStar::CoordinateList& original_output_pos, AStar::CoordinateList& occupancy_initial_points, AStar::CoordinateList& occupancy_final_points, cv::Mat& cropped_image, cv::Mat& occupancy_map) 
{
    float pixels_per_tile_x = (float)cropped_image.cols / N_TILES_X;
    float pixels_per_tile_y = (float)cropped_image.rows / N_TILES_Y;

    AStar::Vec2i estimated_pos;
    bool stop;
    // Input points
    for(uint8_t i=0; i < original_input_pos.size(); i++) {
        estimated_pos.x = original_input_pos[i].x / pixels_per_tile_x;
        estimated_pos.y = original_input_pos[i].y / pixels_per_tile_y;
        stop = false;
        uint16_t c_initial = std::max(estimated_pos.x - 2, 0);
        if(c_initial + KERNEL_SIZE > occupancy_map.cols) {
            c_initial = occupancy_map.cols-7;
        }
        uint16_t r_initial = std::max(estimated_pos.y - 2, 0);
        if(r_initial + KERNEL_SIZE > occupancy_map.rows) {
            r_initial = occupancy_map.rows-7;
        }
        for(uint16_t c=c_initial; c < c_initial + KERNEL_SIZE; c++) {
            if(stop) break;
            for(uint16_t r=r_initial; r < r_initial + KERNEL_SIZE; r++) {
                if(occupancy_map.at<uint8_t>(r, c) == 255) {
                    stop = true;
                    occupancy_initial_points.push_back({c,r});
                    break;
                }
            }
        }
        if(stop == false) {
            return -1;
        }
    }
    // Output points
    for(uint8_t i=0; i < original_output_pos.size(); i++) {
        estimated_pos.x = original_output_pos[i].x / pixels_per_tile_x;
        estimated_pos.y = original_output_pos[i].y / pixels_per_tile_y;
        stop = false;
        uint16_t c_initial = std::max(estimated_pos.x - 2, 0);
        if(c_initial + KERNEL_SIZE >= occupancy_map.cols) {
            c_initial = occupancy_map.cols-7;
        }
        uint16_t r_initial = std::max(estimated_pos.y - 2, 0);
        if(r_initial + KERNEL_SIZE >= occupancy_map.rows) {
            r_initial = occupancy_map.rows-7;
        }
        for(uint16_t c=c_initial + KERNEL_SIZE; c > c_initial; c--) {
            if(stop) break;
            for(uint16_t r=r_initial; r < r_initial + KERNEL_SIZE; r++) {
                if(occupancy_map.at<uint8_t>(r, c) == 255) {
                    stop = true;
                    occupancy_final_points.push_back({c,r});
                    break;
                }
            }
        }
        if(stop == false) {
            return -1;
        }
    }
}

std::vector<line_connection> find_line_connections(AStar::Generator& generator, cv::Mat& occupancy_grid_map, std::vector<AStar::Vec2i>& initial_points, std::vector<AStar::Vec2i>& final_points)
{
    AStar::CoordinateList path;
    std::vector<line_connection> detected_connections;
    std::vector<std::vector<line_connection>> all_lines_costs;
    float cost;

    // Calculate the cost for each possible connection
    for(uint8_t i=0; i < initial_points.size(); i++) {
        std::vector<line_connection> vec_line_costs;
        for(uint8_t j=0; j < final_points.size(); j++) {
            // Find optimal path between each initial and final point
            path = generator.findPath(initial_points[i], final_points[j]);
            if(path.size() <= 1) { // No Path found
                std::vector<line_connection> v;
                return v;
            }
            cv::Mat path_matrix = cv::Mat::zeros(occupancy_grid_map.rows, occupancy_grid_map.cols, CV_8UC3);
            // Get some points (equidistants) from the line
            AStar::CoordinateList extracted_points = extract_line_points(path);
            // Create structure to save line info
            line_connection struct_line;
            struct_line.cost =  calculate_line_cost(extracted_points);
            struct_line.initial_point = initial_points[i];
            struct_line.final_point = final_points[j];
            struct_line.line_points = path;
            // Add results to the vector
            vec_line_costs.push_back(struct_line);
        }
        all_lines_costs.push_back(vec_line_costs);
    }

    // Find best connections
    uint8_t better_connection = 0;
    AStar::Vec2i final_point;
    float better_cost;

    float min;
    int better_match;
    bool is_line_selected[final_points.size()] = {false};
    for(uint8_t i=0; i < final_points.size(); i++) {
        min = all_lines_costs[0][i].cost;
        better_match = -1;
        for(uint8_t j=0; j < all_lines_costs.size(); j++) {
            if(all_lines_costs[j][i].cost < min && all_lines_costs[j][i].cost == std::min_element(all_lines_costs[j].begin(), all_lines_costs[j].end(), compare_costs)->cost) {
                min = all_lines_costs[j][i].cost;
                better_match = j;
            }
        }
        if(better_match > -1) {
            detected_connections.push_back(all_lines_costs[better_match][i]);
            all_lines_costs.erase(all_lines_costs.begin()+better_match);
            is_line_selected[i] = true;
        }
    }

    for(uint8_t i=0; i < final_points.size(); i++) {
        if(is_line_selected[i] == false) {
            min = all_lines_costs[0][i].cost / std::min_element(all_lines_costs[0].begin(), all_lines_costs[0].end(), compare_costs)->cost;
            better_match = 0;
            for(uint8_t j=1; j < all_lines_costs.size(); j++) {
                if(all_lines_costs[j][i].cost / std::min_element(all_lines_costs[j].begin(), all_lines_costs[j].end(), compare_costs)->cost < min) {
                    min = all_lines_costs[j][i].cost / std::min_element(all_lines_costs[j].begin(), all_lines_costs[j].end(), compare_costs)->cost;
                    better_match = j;
                }
            }
            detected_connections.push_back(all_lines_costs[better_match][i]);
            all_lines_costs.erase(all_lines_costs.begin()+better_match);
        }
    }
    return detected_connections;
}

int main(int argc, char **argv)
{
    // Initialize A-star
    AStar::Generator generator;
    generator.setWorldSize({N_TILES_X, N_TILES_Y});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    // Read image
    Mat image = imread("/home/jorge/Pictures/imagem_teste.png", 1);
    
    // Binarize image
    Mat img_gray;
    cvtColor(image, img_gray, COLOR_BGR2GRAY);
    Mat thresh;
    threshold(img_gray, thresh, 127, 255, THRESH_BINARY_INV);

    // detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    // draw contours on the original image
    Mat image_copy = image.clone();
    drawContours(image_copy, contours, -1, Scalar(0, 255, 0), 2);

    //bounding box
    vector<vector<Point>> contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> centers(contours.size());
    vector<float> radius(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = boundingRect(contours_poly[i]);
    }
    Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(drawing, contours_poly, (int)i, color);
        rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
    }

    int tlx = boundRect[0].tl().x; // mudar 0 para i quando existir multiplos contours
    int tly = boundRect[0].tl().y;
    int brx = boundRect[0].br().x;
    int bry = boundRect[0].br().y;
    Rect myROI(tlx, tly, brx - tlx, bry - tly);
    Mat croppedImage = thresh(myROI);
    imshow("Crop", croppedImage);


    /********************* Find best connection between the blocks **********************/
    AStar::CoordinateList initial_points, final_points;

    /*initial_points.push_back({0,13});
    initial_points.push_back({1,28});
    initial_points.push_back({2,48});
    final_points.push_back({79,0});
    final_points.push_back({79,11});
    final_points.push_back({79,24});*/
    AStar::CoordinateList original_input_points, original_output_points;
    original_input_points.push_back({3,146});
    original_input_points.push_back({11,306});
    original_input_points.push_back({7,524});
    original_output_points.push_back({517,3});
    original_output_points.push_back({510,121});
    original_output_points.push_back({512,258});

    // Create occupancy grid map
    cv::Mat occupancy_grid_map = create_occupancy_grid_map(croppedImage, generator, std::min(N_TILES_X, croppedImage.cols), std::min(N_TILES_Y, croppedImage.rows));
    convert_coordinates_to_occupancy_map(original_input_points, original_output_points, initial_points, final_points, croppedImage, occupancy_grid_map);
    for(int i=0; i < initial_points.size(); i++) {
        printf("%d %d\n", initial_points[i].x, initial_points[i].y);
    }
    for(int i=0; i < final_points.size(); i++) {
        printf("%d %d\n", final_points[i].x, final_points[i].y);
    }
    std::vector<line_connection> detected_connections;
    // Find best connections
    detected_connections = find_line_connections(generator, occupancy_grid_map, initial_points, final_points);
    if(detected_connections.size() == 0) {
        printf("No path found for all connections...\n");
        return 0;
    }
    
    // DEBUG: Show detected lines
    cv::Mat detected_lines = cv::Mat::zeros(occupancy_grid_map.rows, occupancy_grid_map.cols, CV_8UC3);
    for(int i=0; i < detected_connections.size(); i++) {
        printf("Initial Point: %d,%d\t Final Point: %d,%d\n", detected_connections[i].initial_point.x, detected_connections[i].initial_point.y, detected_connections[i].final_point.x, detected_connections[i].final_point.y);
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;
        for(int j=0; j < detected_connections[i].line_points.size(); j++) {
            detected_lines.at<Vec3b>(detected_connections[i].line_points[j].y, detected_connections[i].line_points[j].x)[0] = b;
            detected_lines.at<Vec3b>(detected_connections[i].line_points[j].y, detected_connections[i].line_points[j].x)[1] = g;
            detected_lines.at<Vec3b>(detected_connections[i].line_points[j].y, detected_connections[i].line_points[j].x)[2] = r;
        }
    }
    cv::resize(detected_lines, detected_lines, cv::Size(croppedImage.cols, croppedImage.rows));
    cv::imshow("Detected lines", detected_lines);
    cv::waitKey(0);

    return 0;
}