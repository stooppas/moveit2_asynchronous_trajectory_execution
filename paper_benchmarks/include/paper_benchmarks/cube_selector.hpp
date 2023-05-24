#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <queue>
#include <moveit/move_group_interface/move_group_interface.h> 

typedef moveit_msgs::msg::CollisionObject CollisionObject;

struct Point3D {
    float x;
    float y;
    float z;

    // Constructor
    Point3D(float px, float py, float pz) : x(px), y(py), z(pz) {}
};

struct EuclideanDistanceComparator{
    Point3D endEffector;

    void updateEndEffector(const Point3D &e){
        endEffector = e;
    }

    explicit EuclideanDistanceComparator(const Point3D &e): endEffector(e){}

    bool operator()(const CollisionObject &obj1, const CollisionObject &obj2) const{
        float distance1 = calculateEuclideanDistance(obj1, endEffector);
        float distance2 = calculateEuclideanDistance(obj2, endEffector);
        return distance1 > distance2;

    }

private:
    float calculateEuclideanDistance(const CollisionObject &cube, const Point3D &point) const{
        float dx = cube.pose.position.x - point.x;
        float dy = cube.pose.position.y - point.y;
        float dz = cube.pose.position.z - point.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

// Functor for comparing two points based on random order
struct RandomComparator {
    Point3D endEffector;

    explicit RandomComparator(const Point3D &e): endEffector(e){}

    bool operator()(const CollisionObject &obj1, const CollisionObject &obj2) const {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 1);
        return dis(gen) == 0; // Randomly select between p1 and p2
    }
};

template<typename Comparator>
class ThreadSafeCubeQueue{
private:
    std::priority_queue<CollisionObject, std::vector<CollisionObject>, Comparator> priority_queue;
    Comparator comparator;
    mutable std::mutex mutex;

public:
    explicit ThreadSafeCubeQueue(const Comparator &com): comparator(com), priority_queue(comparator){
    }

    void push(CollisionObject &cube){
        std::lock_guard<std::mutex> lock(mutex);
        priority_queue.push(cube);
    }

    bool empty() const{
        std::lock_guard<std::mutex> lock(mutex);
        return priority_queue.empty();
    }

    size_t size() const{
        std::lock_guard<std::mutex> lock(mutex);
        return priority_queue.size();
    }

    void pop(){
        std::lock_guard<std::mutex> lock(mutex);
        priority_queue.pop();
    }

    CollisionObject top(){
        std::lock_guard<std::mutex> lock(mutex);
        return priority_queue.top();
    }

    Comparator& topComparator(){
        return comparator;
    }

};

class ThreadSafeCubeQueueFactory {
public:
    template<typename Comparator>
    static ThreadSafeCubeQueue<Comparator> createPriorityQueue(const Point3D& referencePoint) {
        return ThreadSafeCubeQueue<Comparator>(Comparator(referencePoint));
    }
};