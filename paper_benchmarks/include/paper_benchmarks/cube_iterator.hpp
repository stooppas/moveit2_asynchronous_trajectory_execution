#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <moveit/move_group_interface/move_group_interface.h> 

typedef moveit_msgs::msg::CollisionObject CollisionObject;

struct Point3D {
    float x;
    float y;
    float z;

    // Constructor
    Point3D(float px, float py, float pz) : x(px), y(py), z(pz) {}
};

template<typename Policy>
class CubeIterator;

struct EuclideanDistancePolicy{
    static float distance(const CollisionObject &cube, const Point3D &point){
        float dx = cube.pose.position.x - point.x;
        float dy = cube.pose.position.y - point.y;
        float dz = cube.pose.position.z - point.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

struct RandomCubePolicy{
    static int getRandomCube(size_t size){
        // explicit randomization is not necessary since already the cubes are randomized

        //CollisionObject cube = cubes.front();
        //cubes.erase(cubes.begin());
        //std::random_device rd;
        //std::mt19937 gen(rd());
        //std::uniform_int_distribution<> dist(0, size); // Modify the range as needed

        //int index  = dist(gen);

        return 0;
    }
};


template<typename Policy>
class CubeIterator{

private:
    const std::vector<CollisionObject> points;
    size_t currentIndex;
    const Point3D endEffector;
    float previous_min;
    float counter;

public:
    CubeIterator(const std::vector<CollisionObject> p, const Point3D e, 
        size_t index = 0): points(p), currentIndex(index), endEffector(e), previous_min(0), counter(0){}

    //Dereferece operator
    const CollisionObject& operator*(){
        return points[currentIndex];
    }

    //pre-increment operator
    CubeIterator& operator++(){
        if constexpr(std::is_same_v<Policy, RandomCubePolicy>){
            //int index = Policy::getRandomCube(points.size());
            currentIndex++;

        }else{
            float minDistance = std::numeric_limits<float>::max();
            size_t nextIndex = currentIndex;

            for(size_t i=0; i < points.size(); ++i){
                if(i == currentIndex) continue; 

                float distance = Policy::distance(points[i], endEffector);
                std::cout << "distance    " << distance << std::endl;
                if(distance > previous_min){
                    if(distance < minDistance){
                        minDistance = distance;
                        nextIndex = i;
                    }
                }
            }
            counter++;
            currentIndex = nextIndex;
            previous_min = Policy::distance(points[currentIndex], endEffector);
            std::cout << "current    " << currentIndex << std::endl;

        }

        return *this;
    }
    
    //post-increment operator
    // CubeIterator& operator++(int){
    //     CubeIterator temp = *this;
    //     ++(*this);
    //     return temp;
    // }

    // Equality operator
    bool operator==(const CubeIterator& other) const {
        return currentIndex == other.currentIndex;
    }

    // Inequality operator
    bool operator!=(const CubeIterator& other) const {
        return currentIndex != other.currentIndex;
    }

};


class CubeContainer{

private:
    std::vector<moveit_msgs::msg::CollisionObject> cubes;

public:
    void addCubes(moveit_msgs::msg::CollisionObject cube){
        cubes.emplace_back(cube);
    }

    CubeIterator<RandomCubePolicy> beginRandom(const Point3D e){
        return CubeIterator<RandomCubePolicy>(cubes, e);
    }

    CubeIterator<RandomCubePolicy> endRandom(const Point3D e){
        return CubeIterator<RandomCubePolicy>(cubes, e, cubes.size());
    }

    CubeIterator<EuclideanDistancePolicy> beginEuclidean(const Point3D e){
        return CubeIterator<EuclideanDistancePolicy>(cubes, e);
    }

    CubeIterator<EuclideanDistancePolicy> endEuclidean(const Point3D e){
        size_t current_index = 0;
        float maxDistance = 0;

        for(size_t i=0; i < cubes.size(); ++i){
            float distance = EuclideanDistancePolicy::distance(cubes[i], e);
            if(distance > maxDistance){
                maxDistance = distance;
                current_index = i;
            }
        }
        return CubeIterator<EuclideanDistancePolicy>(cubes, e, current_index);
    }
};