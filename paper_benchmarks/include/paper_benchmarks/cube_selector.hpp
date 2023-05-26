#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <queue>
#include <limits>
#include <moveit/move_group_interface/move_group_interface.h>

typedef moveit_msgs::msg::CollisionObject CollisionObject;

struct Point3D
{
    float x;
    float y;
    float z;

    // Constructor
    Point3D(float px, float py, float pz) : x(px), y(py), z(pz) {}
};

class ThreadSafeCubeQueue
{
private:
    std::vector<CollisionObject> priority_queue;
    Point3D point;
    mutable std::mutex mutex;

    float calculateEuclideanDistance(const CollisionObject &cube, const Point3D &point) const
    {
        float dx = cube.pose.position.x - point.x;
        float dy = cube.pose.position.y - point.y;
        float dz = cube.pose.position.z - point.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

public:
    explicit ThreadSafeCubeQueue(const Point3D &p) : point(p) {}

    void push(CollisionObject &cube)
    {
        std::lock_guard<std::mutex> lock(mutex);
        priority_queue.push_back(cube);
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return priority_queue.empty();
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return priority_queue.size();
    }

    void updatePoint(const Point3D &p)
    {
        std::lock_guard<std::mutex> lock(mutex);
        point = p;
    }

    CollisionObject pop()
    {
        std::lock_guard<std::mutex> lock(mutex);
        size_t current_index = 0;
        float minDistance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < priority_queue.size(); ++i)
        {
            float distance = calculateEuclideanDistance(priority_queue[i], point);
            if (distance < minDistance)
            {
                minDistance = distance;
                current_index = i;
            }
        }
        CollisionObject minObject = priority_queue[current_index];
        priority_queue.erase(priority_queue.begin() + current_index);
        return minObject;
    }
};