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

struct CollisionPlanningObject
{
    CollisionObject collisionObject;
    int robot_1_planned_times;
    int robot_2_planned_times;

    CollisionPlanningObject() {}

    CollisionPlanningObject(CollisionObject c, int r_1, int r_2) : collisionObject(c), robot_1_planned_times(r_1), robot_2_planned_times(r_2) {}

    CollisionPlanningObject &operator=(const CollisionPlanningObject &other)
    {
        if (this != &other)
        {
            collisionObject = other.collisionObject;
            robot_1_planned_times = other.robot_1_planned_times;
            robot_2_planned_times = other.robot_2_planned_times;
        }
        return *this;
    }
};

class ThreadSafeCubeQueue
{
private:
    std::vector<CollisionPlanningObject> priority_queue;
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

    void push(CollisionPlanningObject &cube)
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

    CollisionPlanningObject pop(std::string robot_planning)
    {
        std::lock_guard<std::mutex> lock(mutex);
        size_t current_index = 0;
        float minDistance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < priority_queue.size(); ++i)
        {
            float distance = calculateEuclideanDistance(priority_queue[i].collisionObject, point);
            if (distance < minDistance)
            {
                if (robot_planning.compare("robot_1") && priority_queue[i].robot_1_planned_times >= 5 ||
                    robot_planning.compare("robot_2") && priority_queue[i].robot_2_planned_times >= 5)
                {
                    continue;
                }
                minDistance = distance;
                current_index = i;
            }
        }

        CollisionPlanningObject minObject = priority_queue[current_index];
        if (robot_planning.compare("robot_1"))
        {
            minObject.robot_1_planned_times++;
        }
        else
        {
            minObject.robot_2_planned_times++;
        }
        priority_queue.erase(priority_queue.begin() + current_index);
        return minObject;
    }
};