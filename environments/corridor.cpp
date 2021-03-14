// I'm going to build an example here and then we can separate things out later

// TODO:
// - command line arguments for: start location, probabilities, other?

#include <algorithm>
using std::fill;
using std::for_each;
using std::transform;

#include <functional>
using std::multiplies;

#include <iostream>
using std::boolalpha;
using std::cerr;
using std::cout;
using std::endl;

#include <numeric>
using std::accumulate;

#include <random>
using std::default_random_engine;
using std::random_device;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

#include <sstream>
using std::ostringstream;

#include <string>
using std::string;

#include <vector>
using std::vector;

// Normalize the vector values to [0, 1]
void transform_to_probs(std::vector<double> &vec)
{
    auto sum = accumulate(vec.begin(), vec.end(), 0.0);
    transform(vec.begin(), vec.end(), vec.begin(), [sum](auto val) {
        return val / sum;
    });
}

// Convert a vector to space separated list in string form
string vec_to_string(const vector<double> &vec, const string &title = "")
{
    ostringstream oss(title);
    for (const auto &val : vec)
        oss << "  " << val;
    return oss.str();
}

int main(/*int argc, char const *argv[]*/)
{
    // Create a function for generating random, uniform numbers in [0, 1]
    random_device rd;
    default_random_engine rand_eng(rd());

    // Distribution for computing success/failure
    uniform_real_distribution uniform_dist(0.0, 1.0);
    auto uniform = [&rand_eng, &uniform_dist]() {
        return uniform_dist(rand_eng);
    };

    // Doorway environment representation
    constexpr uint8_t WALL = 0;
    constexpr uint8_t DOOR = 1;

    // clang-format off
    // Create environment similar to the book (Figure 1.1, page 6)
    // 8 walls, door, 1 wall, door, 10 walls, door, 8 walls
    // vector<uint8_t> corridor{
    //     WALL, WALL, WALL, WALL, WALL, WALL, WALL, WALL,
    //     DOOR,
    //     WALL,
    //     DOOR,
    //     WALL, WALL, WALL, WALL, WALL, WALL, WALL, WALL, WALL, WALL,
    //     DOOR,
    //     WALL, WALL, WALL, WALL, WALL, WALL, WALL, WALL,
    // };
    // Easier to debug version
    vector<uint8_t> corridor{
        WALL, WALL,
        DOOR, WALL,
        DOOR, WALL,
        DOOR, DOOR,
        WALL, WALL,
    };
    // clang-format on

    auto corridor_length = corridor.size();

    // Distribution for computing the starting location
    uniform_int_distribution<size_t> uniform_start(0, corridor_length - 1);
    size_t actual_location = uniform_start(rand_eng);

    // Our belief map (the probability that the robot is at any
    // specific location along the corridor).
    // Initialized with an equal probability of being at any position
    // along the corridor.
    double initial_probability = 1.0 / corridor_length;
    vector<double> location_beliefs(corridor_length, initial_probability);
    vector<double> location_beliefs_bar(corridor_length, 0);

    // Let's estimate our position in the corridor.
    // Thus, x_t denotes the probability that we are in
    // any given (discrete) corridor location.

    // The robot can sense the wall/door
    double sensedWall_byWall = 0.999;                 // p(Z = Wall | X = Wall)
    double sensedDoor_byWall = 1 - sensedWall_byWall; // p(Z = Door | X = Wall)
    double sensedDoor_byDoor = 0.999;                 // p(Z = Wall | X = Door)
    double sensedWall_byDoor = 1 - sensedDoor_byDoor; // p(Z = Door | X = Door)

    // The robot can attempt to move one cell at a time, and
    // it tries to move every time step.
    double moveSuccess_byWall = 0.999;
    double moveFail_byWall = 1 - moveSuccess_byWall;
    double moveSuccess_byDoor = 0.999;
    double moveFail_byDoor = 1 - moveSuccess_byDoor;

    bool moved, sensed;
    size_t num_steps = 10;
    for (size_t step = 0; step < num_steps; step++)
    {
        // --------------------------------
        // Move the robot
        // --------------------------------
        auto actual_prob_moving = corridor.at(actual_location) == WALL ? moveSuccess_byWall : moveSuccess_byDoor;
        if (actual_prob_moving > uniform())
        {
            moved = true;
            actual_location = (actual_location + 1) % corridor_length;
        }

        // --------------------------------
        // Sense the environment
        // --------------------------------
        auto actual_prob_sensingWall = corridor.at(actual_location) == WALL ? sensedWall_byWall : sensedWall_byDoor;
        auto sensorValue = actual_prob_sensingWall > uniform() ? WALL : DOOR;
        sensed = sensorValue == corridor.at(actual_location);

        // --------------------------------
        // Measurement update based on random z_t (sensor model)
        // --------------------------------
        for (size_t i = 0; i < corridor_length; i++)
        {
            auto sense_prob = corridor.at(i) == WALL ? sensedWall_byWall : sensedDoor_byDoor;
            auto scale = sense_prob / (1 - sense_prob);
            auto likelihood = corridor.at(i) == sensorValue ? scale : 1;

            location_beliefs_bar.at(i) = likelihood * location_beliefs.at(i);
        }
        // cerr << vec_to_string(location_beliefs_bar, "location_beliefs_bar") << endl;
        transform_to_probs(location_beliefs_bar);
        // cerr << vec_to_string(location_beliefs_bar, "normalized location_beliefs_bar") << endl;

        // --------------------------------
        // Prediction based on u_t = 1 (motion model)
        // --------------------------------

        // TODO: should this be bar?

        if (step < num_steps - 1)
        {
            for (size_t i = 0; i < corridor_length; i++)
            {
                location_beliefs.at(i) = 0;

                // Get to corridor.at(i) from previous spot (wrapping around end)
                auto prev_index = i == 0 ? corridor_length - 1 : i - 1;
                auto move_prob = corridor.at(prev_index) == WALL ? moveSuccess_byWall : moveSuccess_byDoor;
                location_beliefs.at(i) += location_beliefs_bar.at(prev_index) * move_prob;

                // Stay at corridor.at(i) by not moving away
                auto no_move_prob = corridor.at(i) == WALL ? moveFail_byWall : moveFail_byDoor;
                location_beliefs.at(i) += location_beliefs_bar.at(i) * no_move_prob;
            }
        }
        // cerr << vec_to_string(location_beliefs, "location_beliefs") << endl;

        cout << step << " " << actual_location
             << vec_to_string(location_beliefs)
             << (sensorValue == WALL ? " wall" : " door")
             << boolalpha << " " << moved << " " << sensed << endl;

        moved = false;
        sensed = false;
    }

    return 0;
}
